//! A decoded video frame ready for display.

use std::time::Instant;

use anyhow::{anyhow, Result};
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
}
