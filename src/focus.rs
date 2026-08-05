//! Focus measurement: a per-frame sharpness metric for manual-focus assist.
//!
//! The live view shimmers with seeing, so a raw sharpness number jitters too much to focus
//! against. The metric here is deliberately simple and cheap (one pass); the decode thread
//! smooths it over many frames (EMA) and peak-holds the best value, which is what actually
//! lets the user hit best focus with less error.

/// Normalized **Brenner gradient** of an 8-bit luma image — the standard robust sharpness
/// metric for planetary/solar imaging.
///
/// Brenner sums the squared difference between each pixel and the one two columns to its right;
/// sharp, high-contrast detail (in focus) produces large local differences, a blurred frame
/// produces small ones. We divide by the pixel count so the value is a per-pixel *density* —
/// changing the ROI size doesn't rescale it, so readings stay comparable.
///
/// Returns 0.0 for a flat or empty frame.
pub fn sharpness(gray: &[u8], w: usize, h: usize) -> f64 {
    if w < 3 || h == 0 || gray.len() != w * h {
        return 0.0;
    }
    let mut acc: f64 = 0.0;
    for row in gray.chunks_exact(w) {
        for x in 0..(w - 2) {
            let d = row[x] as f64 - row[x + 2] as f64;
            acc += d * d;
        }
    }
    acc / (w * h) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A high-contrast vertical-stripe pattern, period 4 (two dark columns, two bright). Period 2
    /// would be a Brenner blind spot — pixel `x` and `x+2` would be identical — so a period-4
    /// pattern is what actually exercises the stride-2 gradient.
    fn stripes(w: usize, h: usize) -> Vec<u8> {
        let mut g = vec![0u8; w * h];
        for row in g.chunks_exact_mut(w) {
            for (x, p) in row.iter_mut().enumerate() {
                *p = if (x / 2) % 2 == 0 { 0 } else { 255 };
            }
        }
        g
    }

    /// A cheap 1-D box blur across columns, to simulate defocus.
    fn blur(gray: &[u8], w: usize, h: usize) -> Vec<u8> {
        let mut out = vec![0u8; w * h];
        for (r_in, r_out) in gray.chunks_exact(w).zip(out.chunks_exact_mut(w)) {
            for (x, p) in r_out.iter_mut().enumerate() {
                let lo = x.saturating_sub(1);
                let hi = (x + 1).min(w - 1);
                let sum: u32 = (lo..=hi).map(|i| r_in[i] as u32).sum();
                *p = (sum / (hi - lo + 1) as u32) as u8;
            }
        }
        out
    }

    #[test]
    fn flat_frame_scores_zero() {
        assert_eq!(sharpness(&[128; 64], 8, 8), 0.0);
    }

    #[test]
    fn sharp_beats_blurred() {
        let (w, h) = (16, 16);
        let sharp = stripes(w, h);
        let blurred = blur(&sharp, w, h);
        let s = sharpness(&sharp, w, h);
        let b = sharpness(&blurred, w, h);
        assert!(s > 0.0);
        assert!(s > b, "sharp {s} should exceed blurred {b}");
    }

    #[test]
    fn rejects_bad_geometry() {
        assert_eq!(sharpness(&[0; 4], 8, 8), 0.0); // wrong length
        assert_eq!(sharpness(&[], 0, 0), 0.0); // empty
        assert_eq!(sharpness(&[0; 2], 2, 1), 0.0); // too narrow for stride-2
    }
}
