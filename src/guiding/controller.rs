//! Guiding control math: the pixel→mount calibration matrix and the proportional correction
//! that turns a pixel error into timed pulse-guide commands. Everything here is pure and
//! device-free so it can be unit-tested without an INDI connection.

use crate::bus::Dir;

/// Maps mount pulse durations to the pixel displacement they produce, as a 2×2 matrix built by
/// [`Calibration::from_pulses`]. Column 0 is the per-millisecond displacement of a **West**
/// pulse (the RA axis), column 1 that of a **North** pulse (the DEC axis). Storing the real
/// measured axes (rather than assuming N/S/E/W align with the sensor) is what makes guiding
/// robust to camera rotation and a meridian flip.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// Column-major: `[a, c, b, d]` for `M = [[a, b], [c, d]]`; columns `(a,c)`=RA, `(b,d)`=DEC.
    m: [f32; 4],
}

/// Smallest calibration displacement (pixels) we trust: a shorter move than this over the whole
/// calibration pulse means the target barely moved (bad SNR, mount not responding, wrong axis).
const MIN_CALIB_PX: f32 = 2.0;

impl Calibration {
    /// Build the calibration from the measured pixel displacements of a West pulse and a North
    /// pulse, each of `millis` duration. Returns `None` if the pulses didn't move the target
    /// enough, or the two axes are near-parallel (degenerate matrix — can't invert reliably).
    pub fn from_pulses(dpx_west: (f32, f32), dpx_north: (f32, f32), millis: f32) -> Option<Self> {
        if !millis.is_finite() || millis <= 0.0 {
            return None;
        }
        if mag(dpx_west) < MIN_CALIB_PX || mag(dpx_north) < MIN_CALIB_PX {
            return None;
        }
        // Per-millisecond axes.
        let (a, c) = (dpx_west.0 / millis, dpx_west.1 / millis);
        let (b, d) = (dpx_north.0 / millis, dpx_north.1 / millis);
        let det = a * d - b * c;
        // Reject a near-singular matrix (axes almost parallel). Scale the threshold by the axis
        // magnitudes so it's about the *angle* between them, not their absolute size.
        let scale = mag((a, c)) * mag((b, d));
        if !det.is_finite() || scale <= 0.0 || det.abs() < 0.05 * scale {
            return None;
        }
        Some(Calibration { m: [a, c, b, d] })
    }

    /// Construct directly from a column-major matrix (used by tests / persistence).
    #[cfg(test)]
    pub fn from_matrix(m: [f32; 4]) -> Self {
        Calibration { m }
    }

    pub fn matrix(&self) -> [f32; 4] {
        self.m
    }

    /// Pixel displacement produced by pulsing West for `ra_ms` and North for `dec_ms`.
    /// (The forward map `M · [ra, dec]`; used in tests to check the inverse.)
    pub fn displacement(&self, ra_ms: f32, dec_ms: f32) -> (f32, f32) {
        let [a, c, b, d] = self.m;
        (a * ra_ms + b * dec_ms, c * ra_ms + d * dec_ms)
    }

    /// Solve for the pulse durations `(ra_ms, dec_ms)` that would move the target by `-error`,
    /// i.e. back onto the lock point. Positive `ra_ms` means pulse West, positive `dec_ms` North;
    /// negative means the opposite direction. This is the closed-form 2×2 inverse of `M`.
    pub fn correct(&self, error_px: (f32, f32)) -> (f32, f32) {
        let [a, c, b, d] = self.m;
        let det = a * d - b * c;
        if det.abs() < f32::EPSILON {
            return (0.0, 0.0);
        }
        let (ex, ey) = error_px;
        // x = M⁻¹ · (−error)
        let ra_ms = (-d * ex + b * ey) / det;
        let dec_ms = (c * ex - a * ey) / det;
        (ra_ms, dec_ms)
    }
}

/// Tunable guiding-loop parameters, adjustable live from the UI.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuideParams {
    /// Fraction of the full correction applied each cycle (0–1). Lower = gentler, more stable.
    pub aggressiveness: f32,
    /// Upper bound on any single pulse (ms), so a spurious huge error can't fling the mount.
    pub max_pulse_ms: f32,
    /// Deadband: don't correct while the error magnitude is below this many pixels (avoids
    /// chasing seeing/noise).
    pub min_move_px: f32,
    /// Control-loop period (ms) — how often corrections are issued, independent of frame rate.
    pub cadence_ms: u64,
}

impl Default for GuideParams {
    fn default() -> Self {
        GuideParams {
            aggressiveness: 0.5,
            max_pulse_ms: 500.0,
            min_move_px: 0.5,
            cadence_ms: 1000,
        }
    }
}

/// Turn a pixel error into the pulse-guide commands to issue this cycle: apply the calibration,
/// scale by aggressiveness, honour the deadband, clamp to `max_pulse_ms`, and map the signed
/// RA/DEC durations onto cardinal [`Dir`]s (West/East for RA, North/South for DEC). Returns at
/// most one pulse per axis; an empty vec means "inside the deadband, do nothing".
pub fn pulses_for(cal: &Calibration, params: &GuideParams, error_px: (f32, f32)) -> Vec<(Dir, f64)> {
    let mut out = Vec::new();
    if !error_px.0.is_finite() || !error_px.1.is_finite() {
        return out;
    }
    if mag(error_px) < params.min_move_px {
        return out;
    }
    let (ra_ms, dec_ms) = cal.correct(error_px);
    let ra = (ra_ms * params.aggressiveness) as f64;
    let dec = (dec_ms * params.aggressiveness) as f64;
    let max = params.max_pulse_ms as f64;

    // RA axis: +West / −East.
    if let Some((dir, ms)) = axis_pulse(ra, Dir::West, Dir::East, max) {
        out.push((dir, ms));
    }
    // DEC axis: +North / −South.
    if let Some((dir, ms)) = axis_pulse(dec, Dir::North, Dir::South, max) {
        out.push((dir, ms));
    }
    out
}

/// Map a signed pulse duration to a direction + clamped magnitude, dropping sub-millisecond
/// pulses (too short for the mount to act on meaningfully).
fn axis_pulse(ms: f64, positive: Dir, negative: Dir, max: f64) -> Option<(Dir, f64)> {
    let mag = ms.abs().min(max);
    if mag < 1.0 {
        return None;
    }
    Some((if ms >= 0.0 { positive } else { negative }, mag))
}

fn mag((x, y): (f32, f32)) -> f32 {
    (x * x + y * y).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_aligned_calibration_corrects_opposite_direction() {
        // West 1000ms → +10px in x; North 1000ms → +10px in y (no rotation).
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // Target drifted +5px in x. To bring it back we must move −x → pulse East.
        let pulses = pulses_for(
            &cal,
            &GuideParams {
                aggressiveness: 1.0,
                ..Default::default()
            },
            (5.0, 0.0),
        );
        assert_eq!(pulses.len(), 1);
        assert_eq!(pulses[0].0, Dir::East);
        assert!((pulses[0].1 - 500.0).abs() < 1e-3, "got {}", pulses[0].1);
    }

    #[test]
    fn correct_inverts_the_calibration_matrix() {
        // A rotated + scaled calibration; correct(e) must produce pulses whose predicted
        // displacement is exactly −e.
        let cal = Calibration::from_matrix([0.007, 0.007, -0.007, 0.007]);
        let err = (3.5, -1.2);
        let (ra, dec) = cal.correct(err);
        let disp = cal.displacement(ra, dec);
        assert!((disp.0 + err.0).abs() < 1e-3, "dx {} vs {}", disp.0, -err.0);
        assert!((disp.1 + err.1).abs() < 1e-3, "dy {} vs {}", disp.1, -err.1);
    }

    #[test]
    fn deadband_suppresses_tiny_errors() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        let params = GuideParams {
            min_move_px: 0.5,
            ..Default::default()
        };
        assert!(pulses_for(&cal, &params, (0.1, 0.2)).is_empty());
    }

    #[test]
    fn pulses_are_clamped_to_max() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        let params = GuideParams {
            aggressiveness: 1.0,
            max_pulse_ms: 200.0,
            ..Default::default()
        };
        // A 100px error would want a 10 000ms pulse; must clamp to 200ms.
        let pulses = pulses_for(&cal, &params, (100.0, 0.0));
        assert_eq!(pulses.len(), 1);
        assert_eq!(pulses[0], (Dir::East, 200.0));
    }

    #[test]
    fn both_axes_pulse_on_diagonal_error() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        let params = GuideParams {
            aggressiveness: 1.0,
            ..Default::default()
        };
        // Drift +x and −y → correct with East (−x) and North (+y).
        let pulses = pulses_for(&cal, &params, (2.0, -2.0));
        assert_eq!(pulses.len(), 2);
        assert_eq!(pulses[0].0, Dir::East);
        assert_eq!(pulses[1].0, Dir::North);
    }

    #[test]
    fn degenerate_calibration_is_rejected() {
        // Target didn't move.
        assert!(Calibration::from_pulses((0.0, 0.0), (0.0, 10.0), 1000.0).is_none());
        // Parallel axes (both along +x) → singular matrix.
        assert!(Calibration::from_pulses((10.0, 0.0), (8.0, 0.0), 1000.0).is_none());
        // Non-positive duration.
        assert!(Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 0.0).is_none());
    }
}
