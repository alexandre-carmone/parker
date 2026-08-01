//! Per-frame target detection for guiding. Two modes:
//! - **Disk**: thresholded intensity-weighted centroid — the bright planet/disk on dark sky.
//! - **Surface**: normalized cross-correlation of a locked reference patch, with a sub-pixel
//!   parabolic peak fit — solar/lunar surface detail when the disk overfills the frame.
//!
//! The pure functions ([`centroid_threshold`], [`ncc_shift`], [`to_gray`]) are unit-tested on
//! synthetic buffers; [`GuideDetector`] holds the cross-frame state (the reference patch) and is
//! driven from the worker's decode thread.

use std::sync::atomic::Ordering;

use crate::bus::Bus;
use crate::frame::Frame;

/// How the target position is measured each frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuideMode {
    /// Intensity-weighted centroid of the bright region (planets, whole bright disks).
    Disk,
    /// Cross-correlation of a locked reference patch (solar/lunar surface).
    Surface,
}

impl GuideMode {
    pub fn as_u8(self) -> u8 {
        match self {
            GuideMode::Disk => 0,
            GuideMode::Surface => 1,
        }
    }

    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => GuideMode::Surface,
            _ => GuideMode::Disk,
        }
    }
}

// Disk-mode threshold: fraction of the (max−min) span above which pixels count toward the
// centroid.
const DISK_THRESH: f32 = 0.5;
/// Fewest above-threshold pixels for a trustworthy centroid.
const MIN_DISK_PIXELS: usize = 8;
/// Surface-mode reference patch size and how far (± px) we search for it each frame.
const PATCH: usize = 64;
const SEARCH: usize = 24;
/// Lowest NCC peak we accept as a real match (below → lost the feature).
const NCC_MIN_PEAK: f32 = 0.3;

/// Convert a tightly-packed RGBA8 buffer to 8-bit luma.
pub fn to_gray(rgba: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut gray = vec![0u8; w * h];
    for (g, px) in gray.iter_mut().zip(rgba.chunks_exact(4)) {
        // Rec. 601 luma; integer weights to stay cheap on the hot path.
        let y = (px[0] as u32 * 77 + px[1] as u32 * 150 + px[2] as u32 * 29) >> 8;
        *g = y as u8;
    }
    gray
}

/// Intensity-weighted centroid of the pixels brighter than `min + thresh_frac*(max−min)`.
/// Returns the centroid in pixel coordinates, or `None` if too few pixels clear the threshold.
pub fn centroid_threshold(gray: &[u8], w: usize, h: usize, thresh_frac: f32) -> Option<(f32, f32)> {
    if gray.len() != w * h || w == 0 || h == 0 {
        return None;
    }
    let (mut lo, mut hi) = (255u8, 0u8);
    for &v in gray {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi <= lo {
        return None; // flat frame — nothing to lock
    }
    let thresh = lo as f32 + thresh_frac * (hi - lo) as f32;
    let mut sum_w = 0.0f64;
    let mut sum_x = 0.0f64;
    let mut sum_y = 0.0f64;
    let mut count = 0usize;
    for j in 0..h {
        let row = j * w;
        for i in 0..w {
            let v = gray[row + i] as f32;
            if v >= thresh {
                // Weight by intensity above threshold so background contributes nothing.
                let wgt = (v - thresh) as f64;
                sum_w += wgt;
                sum_x += wgt * i as f64;
                sum_y += wgt * j as f64;
                count += 1;
            }
        }
    }
    if count < MIN_DISK_PIXELS || sum_w <= 0.0 {
        return None;
    }
    Some(((sum_x / sum_w) as f32, (sum_y / sum_w) as f32))
}

/// Find the sub-pixel shift of the `rw×rh` reference patch within `gray`, searching ±`search`
/// pixels around the patch's original top-left `anchor`. Returns `(dx, dy)` — how far the
/// content has moved from where the patch was captured — or `None` if the search window falls
/// outside the frame or no strong match is found.
pub fn ncc_shift(
    reference: &[u8],
    (rw, rh): (usize, usize),
    gray: &[u8],
    w: usize,
    h: usize,
    anchor: (usize, usize),
    search: usize,
) -> Option<(f32, f32)> {
    if rw == 0 || rh == 0 || reference.len() != rw * rh || gray.len() != w * h {
        return None;
    }
    let (ax, ay) = anchor;
    // The search window must stay inside the frame for every candidate offset.
    if ax < search || ay < search || ax + search + rw > w || ay + search + rh > h {
        return None;
    }

    // Zero-mean reference + its norm (computed once).
    let n = (rw * rh) as f32;
    let rmean = reference.iter().map(|&v| v as f32).sum::<f32>() / n;
    let ref_zm: Vec<f32> = reference.iter().map(|&v| v as f32 - rmean).collect();
    let ref_norm = ref_zm.iter().map(|x| x * x).sum::<f32>().sqrt();
    if ref_norm <= 0.0 {
        return None; // featureless reference
    }

    let s = search as isize;
    let span = (2 * search + 1) as isize;
    let idx = |di: isize, dj: isize| ((dj + s) * span + (di + s)) as usize;
    let mut grid = vec![f32::MIN; (span * span) as usize];
    let (mut best, mut bi, mut bj) = (f32::MIN, 0isize, 0isize);
    for dj in -s..=s {
        for di in -s..=s {
            let x0 = (ax as isize + di) as usize;
            let y0 = (ay as isize + dj) as usize;
            let ncc = ncc_at(&ref_zm, ref_norm, gray, w, x0, y0, rw, rh);
            grid[idx(di, dj)] = ncc;
            if ncc > best {
                best = ncc;
                bi = di;
                bj = dj;
            }
        }
    }
    if best < NCC_MIN_PEAK {
        return None;
    }

    // Parabolic sub-pixel refinement along each axis (skip if the peak is at the search edge).
    let sub = |m: f32, c: f32, p: f32| -> f32 {
        let denom = m - 2.0 * c + p;
        if denom.abs() < 1e-6 {
            0.0
        } else {
            (0.5 * (m - p) / denom).clamp(-1.0, 1.0)
        }
    };
    let sub_x = if bi > -s && bi < s {
        sub(grid[idx(bi - 1, bj)], best, grid[idx(bi + 1, bj)])
    } else {
        0.0
    };
    let sub_y = if bj > -s && bj < s {
        sub(grid[idx(bi, bj - 1)], best, grid[idx(bi, bj + 1)])
    } else {
        0.0
    };
    Some((bi as f32 + sub_x, bj as f32 + sub_y))
}

/// Normalized cross-correlation of the zero-mean reference against the `rw×rh` window of `gray`
/// with top-left `(x0, y0)`. Returns a value in `[-1, 1]` (0 if the window is flat).
#[allow(clippy::too_many_arguments)]
fn ncc_at(
    ref_zm: &[f32],
    ref_norm: f32,
    gray: &[u8],
    w: usize,
    x0: usize,
    y0: usize,
    rw: usize,
    rh: usize,
) -> f32 {
    let n = (rw * rh) as f32;
    let mut sum = 0.0f32;
    for j in 0..rh {
        let row = (y0 + j) * w + x0;
        for i in 0..rw {
            sum += gray[row + i] as f32;
        }
    }
    let mean = sum / n;
    let mut cross = 0.0f32;
    let mut var = 0.0f32;
    let mut k = 0;
    for j in 0..rh {
        let row = (y0 + j) * w + x0;
        for i in 0..rw {
            let iv = gray[row + i] as f32 - mean;
            cross += ref_zm[k] * iv;
            var += iv * iv;
            k += 1;
        }
    }
    if var <= 0.0 {
        return 0.0;
    }
    cross / (ref_norm * var.sqrt())
}

/// A captured reference patch and where it was taken from, in frame pixels.
struct Patch {
    data: Vec<u8>,
    w: usize,
    h: usize,
    /// Top-left of the patch in the frame it was captured from.
    anchor: (usize, usize),
}

/// Stateful detector owned by the decode thread. It reads the current mode and reference
/// generation from the [`Bus`] each frame; a bumped generation (guide start / re-lock) makes it
/// recapture the Surface reference patch.
pub struct GuideDetector {
    ref_generation: u64,
    reference: Option<Patch>,
}

impl Default for GuideDetector {
    fn default() -> Self {
        GuideDetector {
            ref_generation: u64::MAX, // force a first capture when Surface mode is used
            reference: None,
        }
    }
}

impl GuideDetector {
    /// Measure the target position in frame pixels for this frame, or `None` if no lock.
    pub fn measure(&mut self, frame: &Frame, bus: &Bus) -> Option<(f32, f32)> {
        let mode = GuideMode::from_u8(bus.guide_mode.load(Ordering::Relaxed));
        let generation = bus.ref_generation.load(Ordering::Relaxed);
        let (w, h) = (frame.width, frame.height);
        let gray = to_gray(&frame.rgba, w, h);

        match mode {
            GuideMode::Disk => centroid_threshold(&gray, w, h, DISK_THRESH),
            GuideMode::Surface => {
                if self.reference.is_none() || self.ref_generation != generation {
                    self.capture_reference(&gray, w, h, generation);
                }
                let patch = self.reference.as_ref()?;
                let shift = ncc_shift(&patch.data, (patch.w, patch.h), &gray, w, h, patch.anchor, SEARCH)?;
                let cx = patch.anchor.0 as f32 + patch.w as f32 / 2.0 + shift.0;
                let cy = patch.anchor.1 as f32 + patch.h as f32 / 2.0 + shift.1;
                Some((cx, cy))
            }
        }
    }

    /// Capture a fresh reference patch centered in the frame (leaving room for the search
    /// window). If the frame is too small, clears the reference so `measure` reports no lock.
    fn capture_reference(&mut self, gray: &[u8], w: usize, h: usize, generation: u64) {
        self.ref_generation = generation;
        let pw = PATCH.min(w.saturating_sub(2 * SEARCH));
        let ph = PATCH.min(h.saturating_sub(2 * SEARCH));
        if pw < 8 || ph < 8 {
            self.reference = None;
            return;
        }
        let ax = (w - pw) / 2;
        let ay = (h - ph) / 2;
        let mut data = vec![0u8; pw * ph];
        for j in 0..ph {
            let src = (ay + j) * w + ax;
            data[j * pw..j * pw + pw].copy_from_slice(&gray[src..src + pw]);
        }
        self.reference = Some(Patch {
            data,
            w: pw,
            h: ph,
            anchor: (ax, ay),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A smooth, single-peaked Gaussian bump centered at `(cx, cy)` — gives NCC an unambiguous
    /// match so a known shift can be recovered exactly.
    fn bump(w: usize, h: usize, cx: f32, cy: f32, sigma: f32) -> Vec<u8> {
        let mut g = vec![0u8; w * h];
        for j in 0..h {
            for i in 0..w {
                let d2 = (i as f32 - cx).powi(2) + (j as f32 - cy).powi(2);
                g[j * w + i] = (255.0 * (-d2 / (2.0 * sigma * sigma)).exp()) as u8;
            }
        }
        g
    }

    #[test]
    fn centroid_finds_a_bright_square() {
        // 20×20 dark frame with a bright 4×4 square whose center is (10.5, 6.5).
        let (w, h) = (20usize, 20usize);
        let mut gray = vec![0u8; w * h];
        for j in 5..9 {
            for i in 9..13 {
                gray[j * w + i] = 255;
            }
        }
        let (cx, cy) = centroid_threshold(&gray, w, h, 0.5).unwrap();
        assert!((cx - 10.5).abs() < 0.01, "cx {cx}");
        assert!((cy - 6.5).abs() < 0.01, "cy {cy}");
    }

    #[test]
    fn centroid_rejects_a_flat_frame() {
        assert!(centroid_threshold(&[42u8; 100], 10, 10, 0.5).is_none());
    }

    #[test]
    fn ncc_recovers_an_integer_shift() {
        let (w, h) = (100usize, 100usize);
        // Reference frame: bump at (50,50). Shifted frame: same bump moved +3, −2.
        let base = bump(w, h, 50.0, 50.0, 6.0);
        let shifted = bump(w, h, 53.0, 48.0, 6.0);
        // Reference patch of the bump, anchored so its window stays in-bounds.
        let (rw, rh) = (32usize, 32usize);
        let (ax, ay) = (34usize, 34usize); // patch center (50,50)
        let mut reference = vec![0u8; rw * rh];
        for j in 0..rh {
            for i in 0..rw {
                reference[j * rw + i] = base[(ay + j) * w + (ax + i)];
            }
        }
        let (dx, dy) = ncc_shift(&reference, (rw, rh), &shifted, w, h, (ax, ay), 8).unwrap();
        assert!((dx - 3.0).abs() < 0.5, "dx {dx}");
        assert!((dy + 2.0).abs() < 0.5, "dy {dy}");
    }

    #[test]
    fn ncc_out_of_bounds_is_none() {
        let g = vec![0u8; 100];
        // Anchor too close to the edge for the search window.
        assert!(ncc_shift(&[1u8; 16], (4, 4), &g, 10, 10, (0, 0), 8).is_none());
    }

    #[test]
    fn gray_matches_luma() {
        // Pure red pixel → ~77/256 of 255 ≈ 76.
        let rgba = vec![255, 0, 0, 255];
        assert_eq!(to_gray(&rgba, 1, 1)[0], 76);
    }
}
