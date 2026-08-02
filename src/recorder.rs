//! SER video-file recorder.
//!
//! SER is the de-facto container for planetary/solar video capture: a fixed 178-byte header,
//! then uncompressed frames written back-to-back, then an optional trailer of one timestamp per
//! frame. We write the **native sensor payload** — 8- or 16-bit mono, or 8-bit RGB — so
//! downstream tools (AutoStakkert!, PIPP, SER Player) can stack and debayer the true data rather
//! than a lossy 8-bit preview.
//!
//! The writer is generic over `Write + Seek` so unit tests can drive it with an in-memory
//! `Cursor`; the worker uses a `BufWriter<File>`. `FrameCount` isn't known up front, so it is
//! written as a placeholder and patched by [`SerRecorder::finalize`] once recording stops.

use std::fs::File;
use std::io::{BufWriter, Result as IoResult, Seek, SeekFrom, Write};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};

/// SER v3 header length in bytes; frame data starts here.
const HEADER_LEN: usize = 178;
/// Byte offset of the `FrameCount` field, patched on finalize.
const FRAME_COUNT_OFFSET: u64 = 38;
/// Seconds between the FILETIME epoch (1601-01-01) and the Unix epoch (1970-01-01).
const FILETIME_UNIX_OFFSET_SECS: i64 = 11_644_473_600;

/// Pixel layout of the recorded frames, mapped to a SER `ColorID`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SerColor {
    /// Single grayscale plane (the common solar/planetary case).
    Mono,
    /// Three interleaved 8-bit planes (used for MJPEG-decoded frames).
    Rgb,
    /// A Bayer-matrix sensor recorded raw; downstream tools debayer using the pattern.
    BayerRGGB,
    BayerGRBG,
    BayerGBRG,
    BayerBGGR,
}

impl SerColor {
    /// Pick the color layout from a camera's `CCD_CFA` `CFA_TYPE` string. An absent or
    /// unrecognized pattern is treated as mono (the mono-first default).
    pub fn from_cfa(cfa: Option<&str>) -> SerColor {
        match cfa.map(|s| s.trim().to_ascii_uppercase()).as_deref() {
            Some("RGGB") => SerColor::BayerRGGB,
            Some("GRBG") => SerColor::BayerGRBG,
            Some("GBRG") => SerColor::BayerGBRG,
            Some("BGGR") => SerColor::BayerBGGR,
            _ => SerColor::Mono,
        }
    }

    /// The SER `ColorID` code written into the header.
    fn color_id(self) -> i32 {
        match self {
            SerColor::Mono => 0,
            SerColor::BayerRGGB => 8,
            SerColor::BayerGRBG => 9,
            SerColor::BayerGBRG => 10,
            SerColor::BayerBGGR => 11,
            SerColor::Rgb => 100,
        }
    }

    /// Number of samples per pixel (RGB is 3 interleaved planes; everything else is 1).
    fn planes(self) -> usize {
        if matches!(self, SerColor::Rgb) {
            3
        } else {
            1
        }
    }
}

/// Writes a single SER file. Create it, feed each frame's native bytes to
/// [`write_frame`](SerRecorder::write_frame), then [`finalize`](SerRecorder::finalize).
pub struct SerRecorder<W: Write + Seek> {
    out: W,
    frame_len: usize,
    frame_count: u32,
    /// One FILETIME per written frame, flushed as the trailer on finalize.
    timestamps: Vec<i64>,
}

impl SerRecorder<BufWriter<File>> {
    /// Create a SER file at `path`. `depth` is bits-per-sample (8 or 16).
    pub fn create_file(
        path: &Path,
        width: u32,
        height: u32,
        color: SerColor,
        depth: u8,
    ) -> Result<Self> {
        let file = File::create(path)
            .map_err(|e| anyhow!("creating SER file {}: {e}", path.display()))?;
        SerRecorder::create(BufWriter::new(file), width, height, color, depth)
    }
}

impl<W: Write + Seek> SerRecorder<W> {
    /// Write the 178-byte header and return a recorder ready for frames. `FrameCount` is left at
    /// zero and patched by [`finalize`](SerRecorder::finalize).
    pub fn create(mut out: W, width: u32, height: u32, color: SerColor, depth: u8) -> Result<Self> {
        let bytes_per_sample = if depth > 8 { 2 } else { 1 };
        let frame_len = width as usize * height as usize * color.planes() * bytes_per_sample;

        let mut header = [0u8; HEADER_LEN];
        header[0..14].copy_from_slice(b"LUCAM-RECORDER");
        // LuID (offset 14) stays 0.
        put_i32(&mut header, 18, color.color_id());
        // LittleEndian (offset 22): written 0 to match the de-facto ecosystem convention —
        // Windows capture software writes 0 while emitting little-endian 16-bit data, and the
        // common readers (AutoStakkert!/PIPP/SER Player) expect exactly that. Our u16 samples
        // are native little-endian, so this reads back correctly.
        put_i32(&mut header, 22, 0);
        put_i32(&mut header, 26, width as i32);
        put_i32(&mut header, 30, height as i32);
        put_i32(&mut header, 34, depth as i32);
        // FrameCount (offset 38) stays 0 until finalize().
        // Observer/Instrument/Telescope (offsets 42/82/122) stay blank.
        let now = filetime_now();
        // DateTime (local): we don't track the timezone, so mirror the UTC value.
        put_i64(&mut header, 162, now);
        put_i64(&mut header, 170, now);
        out.write_all(&header)
            .map_err(|e| anyhow!("writing SER header: {e}"))?;

        Ok(SerRecorder {
            out,
            frame_len,
            frame_count: 0,
            timestamps: Vec::new(),
        })
    }

    /// Expected byte length of one frame's payload (width·height·planes·bytes-per-sample).
    pub fn frame_len(&self) -> usize {
        self.frame_len
    }

    /// Frames written so far.
    pub fn frame_count(&self) -> u32 {
        self.frame_count
    }

    /// Append one frame. `bytes` must be exactly [`frame_len`](SerRecorder::frame_len) long;
    /// a mismatch (e.g. geometry changed mid-recording) is rejected so the file stays valid.
    pub fn write_frame(&mut self, bytes: &[u8]) -> Result<()> {
        if bytes.len() != self.frame_len {
            return Err(anyhow!(
                "SER frame size {} != expected {}",
                bytes.len(),
                self.frame_len
            ));
        }
        self.out
            .write_all(bytes)
            .map_err(|e| anyhow!("writing SER frame: {e}"))?;
        self.timestamps.push(filetime_now());
        self.frame_count += 1;
        Ok(())
    }

    /// Append the per-frame timestamp trailer, patch the header `FrameCount`, and flush. Returns
    /// the number of frames written.
    pub fn finalize(self) -> Result<u32> {
        self.finalize_inner().map(|(count, _)| count)
    }

    /// Shared finalize path that also yields the underlying writer (used by tests to inspect the
    /// produced bytes).
    fn finalize_inner(mut self) -> Result<(u32, W)> {
        for ts in &self.timestamps {
            self.out
                .write_all(&ts.to_le_bytes())
                .map_err(|e| anyhow!("writing SER trailer: {e}"))?;
        }
        self.out
            .seek(SeekFrom::Start(FRAME_COUNT_OFFSET))
            .map_err(|e| anyhow!("seeking to SER FrameCount: {e}"))?;
        self.out
            .write_all(&(self.frame_count as i32).to_le_bytes())
            .map_err(|e| anyhow!("patching SER FrameCount: {e}"))?;
        self.out.flush().map_err(|e| anyhow!("flushing SER: {e}"))?;
        Ok((self.frame_count, self.out))
    }
}

fn put_i32(buf: &mut [u8], off: usize, v: i32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

fn put_i64(buf: &mut [u8], off: usize, v: i64) {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
}

/// Current time as a Windows FILETIME (100-nanosecond ticks since 1601-01-01 UTC).
fn filetime_now() -> i64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => (d.as_secs() as i64 + FILETIME_UNIX_OFFSET_SECS) * 10_000_000
            + (d.subsec_nanos() as i64) / 100,
        Err(_) => 0,
    }
}

/// Inflate a zlib-compressed `.stream.z` raw frame. The `indi` crate only base64-decodes BLOBs;
/// compressed raw streams arrive still deflated, so we inflate them here.
pub fn inflate_zlib(data: &[u8]) -> IoResult<Vec<u8>> {
    use std::io::Read;
    let mut decoder = flate2::read::ZlibDecoder::new(data);
    let mut out = Vec::new();
    decoder.read_to_end(&mut out)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn read_i32(buf: &[u8], off: usize) -> i32 {
        i32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
    }

    #[test]
    fn header_records_geometry_and_depth() {
        let rec =
            SerRecorder::create(Cursor::new(Vec::new()), 640, 480, SerColor::Mono, 16).unwrap();
        assert_eq!(rec.frame_len(), 640 * 480 * 2);
        let (_, cur) = rec.finalize_inner().unwrap();
        let bytes = cur.into_inner();
        assert_eq!(read_i32(&bytes, 26), 640); // width
        assert_eq!(read_i32(&bytes, 30), 480); // height
        assert_eq!(read_i32(&bytes, 34), 16); // depth
        assert_eq!(read_i32(&bytes, 18), 0); // Mono ColorID
    }

    #[test]
    fn finalize_patches_count_and_writes_trailer() {
        let mut rec = SerRecorder::create(Cursor::new(Vec::new()), 4, 2, SerColor::Mono, 8).unwrap();
        assert_eq!(rec.frame_len(), 8);
        for _ in 0..3 {
            rec.write_frame(&[0u8; 8]).unwrap();
        }
        let (count, cur) = rec.finalize_inner().unwrap();
        assert_eq!(count, 3);
        let bytes = cur.into_inner();
        // header + 3 frames*8 + 3 timestamps*8
        assert_eq!(bytes.len(), HEADER_LEN + 3 * 8 + 3 * 8);
        assert_eq!(&bytes[0..14], b"LUCAM-RECORDER");
        assert_eq!(read_i32(&bytes, 18), 0); // Mono ColorID
        assert_eq!(read_i32(&bytes, 26), 4); // width
        assert_eq!(read_i32(&bytes, 30), 2); // height
        assert_eq!(read_i32(&bytes, 34), 8); // depth
        assert_eq!(read_i32(&bytes, 38), 3); // FrameCount patched
    }

    #[test]
    fn rejects_wrong_frame_size() {
        let mut rec =
            SerRecorder::create(Cursor::new(Vec::new()), 4, 2, SerColor::Mono, 8).unwrap();
        assert!(rec.write_frame(&[0u8; 7]).is_err());
        assert_eq!(rec.frame_count(), 0);
    }

    #[test]
    fn rgb_frame_len_is_three_planes() {
        let rec = SerRecorder::create(Cursor::new(Vec::new()), 10, 10, SerColor::Rgb, 8).unwrap();
        assert_eq!(rec.frame_len(), 300);
    }

    #[test]
    fn cfa_maps_to_bayer_color_id() {
        assert_eq!(SerColor::from_cfa(Some("RGGB")), SerColor::BayerRGGB);
        assert_eq!(SerColor::from_cfa(Some("bggr")), SerColor::BayerBGGR);
        assert_eq!(SerColor::from_cfa(None), SerColor::Mono);
        assert_eq!(SerColor::from_cfa(Some("weird")), SerColor::Mono);
    }
}
