//! A decoded video frame ready for display.

use std::time::Instant;

use anyhow::{anyhow, Result};
use egui::{Color32, ColorImage};
use image::ImageFormat;

/// A single decoded frame from the camera video stream, stored as RGBA8 so it can be
/// uploaded to an egui texture directly. The `seq` counter lets the GUI detect new frames
/// without comparing pixel buffers.
pub struct Frame {
    pub width: usize,
    pub height: usize,
    /// Tightly packed RGBA8, `width * height * 4` bytes.
    pub rgba: Vec<u8>,
    /// Monotonic frame sequence number.
    pub seq: u64,
    /// Full-scale ADU of the source samples: 255 for an 8-bit stream (and MJPEG), 65535 for a
    /// 16-bit stream. The preview `rgba` is always 8-bit — for 16-bit it holds the high byte — so
    /// this records the *original* dynamic range, letting the histogram label its x-axis in ADU.
    pub max_adu: u32,
    /// When the frame was decoded (for FPS + capture timestamps).
    #[allow(dead_code)]
    pub decoded_at: Instant,
}

impl Frame {
    pub fn new(width: usize, height: usize, rgba: Vec<u8>, seq: u64) -> Self {
        Frame {
            width,
            height,
            rgba,
            seq,
            max_adu: 255,
            decoded_at: Instant::now(),
        }
    }

    /// Decode a video-stream BLOB into a `Frame`. MJPEG (`.stream_jpg`) is the primary path;
    /// raw `.stream`/`.stream.z` are not yet supported (M1 selects the MJPEG encoder).
    pub fn from_stream_blob(fmt: Option<&str>, data: &[u8], seq: u64) -> Result<Frame> {
        let is_jpeg = fmt
            .map(|f| f.contains("jpg") || f.contains("jpeg"))
            .unwrap_or(false);
        let img = if is_jpeg {
            image::load_from_memory_with_format(data, ImageFormat::Jpeg)?
        } else {
            image::load_from_memory(data)
                .map_err(|e| anyhow!("unsupported stream format {fmt:?}: {e}"))?
        };
        let rgba = img.to_rgba8();
        let (w, h) = (rgba.width() as usize, rgba.height() as usize);
        Ok(Frame::new(w, h, rgba.into_raw(), seq))
    }

    /// Decode a **raw** (`.stream` / decompressed `.stream.z`) mono video frame into a display
    /// `Frame`. A raw blob carries no dimensions, so `width`/`height` come from the camera's
    /// current readout region. The bytes-per-sample is inferred from the payload length so the
    /// same path serves 8- and 16-bit streams:
    ///
    /// - `len == width*height`   → 8-bit mono, used directly as gray.
    /// - `len == width*height*2` → 16-bit little-endian mono, shown via the high byte (the live
    ///   view's auto-stretch/gain then scales it); the full 16 bits are preserved for recording.
    ///
    /// This is mono-first: Bayer/OSC sensors still record correctly (the raw bytes are written
    /// verbatim with the right ColorID), but here they preview as grayscale — no debayer.
    pub fn from_raw_stream(data: &[u8], width: usize, height: usize, seq: u64) -> Result<Frame> {
        let px = width.checked_mul(height).unwrap_or(0);
        if px == 0 {
            return Err(anyhow!("raw stream: unknown frame geometry"));
        }
        let mut rgba = Vec::with_capacity(px * 4);
        let is_16bit = data.len() == px * 2;
        if data.len() == px {
            for &g in data {
                rgba.extend_from_slice(&[g, g, g, 255]);
            }
        } else if is_16bit {
            for s in data.chunks_exact(2) {
                let g = u16::from_le_bytes([s[0], s[1]]);
                let hi = (g >> 8) as u8;
                rgba.extend_from_slice(&[hi, hi, hi, 255]);
            }
        } else {
            return Err(anyhow!(
                "raw stream: {} bytes doesn't match {}×{} mono at 8 or 16 bit",
                data.len(),
                width,
                height
            ));
        }
        let mut frame = Frame::new(width, height, rgba, seq);
        if is_16bit {
            frame.max_adu = 65535;
        }
        Ok(frame)
    }

    /// Luminance histogram of the raw frame: 256 bins over the pixels' Rec. 601 luma. Used by the
    /// preview histogram to judge exposure — computed off the un-stretched `rgba` so it reflects the
    /// true sensor levels, not the display gain. A single pass over the pixels.
    pub fn luma_histogram(&self) -> [u32; 256] {
        let mut bins = [0u32; 256];
        for p in self.rgba.chunks_exact(4) {
            // Integer Rec. 601 luma (77·R + 150·G + 29·B) / 256.
            let luma = (77 * p[0] as u32 + 150 * p[1] as u32 + 29 * p[2] as u32) >> 8;
            bins[luma.min(255) as usize] += 1;
        }
        bins
    }

    /// Apply the live-view display stretch and produce an egui image ready for texture upload.
    ///
    /// This is the per-frame CPU hot path (a full scan for `auto`, then one pass building the
    /// `Color32` buffer). It is intentionally kept off the GUI thread — the worker calls it and
    /// publishes the result via [`crate::bus::Bus::publish_display`], so the UI only uploads.
    pub fn to_display_image(&self, auto: bool, gain: f32) -> ColorImage {
        let mut scale = gain.max(0.0);
        if auto {
            let max = self
                .rgba
                .chunks_exact(4)
                .flat_map(|p| [p[0], p[1], p[2]])
                .max()
                .unwrap_or(255);
            if max > 0 {
                scale *= 255.0 / max as f32;
            }
        }
        let size = [self.width, self.height];
        // Fast path: nothing to stretch, convert straight to Color32.
        if (scale - 1.0).abs() < f32::EPSILON {
            return ColorImage::from_rgba_unmultiplied(size, &self.rgba);
        }
        let s = |c: u8| (c as f32 * scale).round().clamp(0.0, 255.0) as u8;
        let pixels = self
            .rgba
            .chunks_exact(4)
            .map(|p| Color32::from_rgba_unmultiplied(s(p[0]), s(p[1]), s(p[2]), p[3]))
            .collect();
        ColorImage::new(size, pixels)
    }
}

/// Inflate a zlib-compressed `.stream.z` raw frame. The `indi` crate only base64-decodes BLOBs;
/// compressed raw streams arrive still deflated, so we inflate them here before decoding.
pub fn inflate_zlib(data: &[u8]) -> std::io::Result<Vec<u8>> {
    use std::io::Read;
    let mut decoder = flate2::read::ZlibDecoder::new(data);
    let mut out = Vec::new();
    decoder.read_to_end(&mut out)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(rgba: Vec<u8>) -> Frame {
        Frame::new(rgba.len() / 4, 1, rgba, 1)
    }

    #[test]
    fn identity_when_no_stretch() {
        let f = frame(vec![10, 20, 30, 255, 40, 50, 60, 255]);
        let img = f.to_display_image(false, 1.0);
        assert_eq!(img.size, [2, 1]);
        assert_eq!(
            (img.pixels[0].r(), img.pixels[0].g(), img.pixels[0].b()),
            (10, 20, 30)
        );
        assert_eq!(img.pixels[1].b(), 60);
    }

    #[test]
    fn auto_stretch_maps_brightest_to_255() {
        // Brightest channel is 128 → scale 2.0; 64 → 128, 128 → 255, alpha preserved.
        let f = frame(vec![64, 0, 128, 255]);
        let img = f.to_display_image(true, 1.0);
        let p = img.pixels[0];
        assert_eq!((p.r(), p.g(), p.b(), p.a()), (128, 0, 255, 255));
    }

    #[test]
    fn manual_gain_clamps() {
        let f = frame(vec![200, 10, 0, 255]);
        let img = f.to_display_image(false, 2.0);
        let p = img.pixels[0];
        assert_eq!((p.r(), p.g(), p.b()), (255, 20, 0));
    }

    #[test]
    fn raw_stream_8bit_mono_replicates_gray() {
        // 2×1 8-bit mono: two gray pixels.
        let f = Frame::from_raw_stream(&[40, 200], 2, 1, 7).unwrap();
        assert_eq!((f.width, f.height), (2, 1));
        assert_eq!(&f.rgba[0..4], &[40, 40, 40, 255]);
        assert_eq!(&f.rgba[4..8], &[200, 200, 200, 255]);
    }

    #[test]
    fn raw_stream_16bit_mono_uses_high_byte() {
        // 1×1 16-bit LE mono: 0x1234 → high byte 0x12.
        let f = Frame::from_raw_stream(&[0x34, 0x12], 1, 1, 7).unwrap();
        assert_eq!(&f.rgba[0..4], &[0x12, 0x12, 0x12, 255]);
    }

    #[test]
    fn raw_stream_rejects_mismatched_length() {
        assert!(Frame::from_raw_stream(&[1, 2, 3], 2, 1, 7).is_err());
        assert!(Frame::from_raw_stream(&[1, 2], 0, 0, 7).is_err());
    }
}
