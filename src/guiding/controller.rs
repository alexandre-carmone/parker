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

    /// Per-axis scale in pixels-per-millisecond: `(RA, DEC)` = the magnitude of each column.
    /// Used to turn a per-axis pixel deadband into a pulse-length threshold, and to report the
    /// guide rate in the UI (`× 1000` gives px/s).
    pub fn axis_scales(&self) -> (f32, f32) {
        let [a, c, b, d] = self.m;
        (mag((a, c)), mag((b, d)))
    }

    /// Unit direction vectors of the RA and DEC axes on the sensor, `((rx, ry), (dx, dy))`.
    /// Projecting a pixel error onto these gives the error decomposed into mount axes.
    pub fn axis_units(&self) -> ((f32, f32), (f32, f32)) {
        let [a, c, b, d] = self.m;
        (unit((a, c)), unit((b, d)))
    }

    /// Orientation of the RA and DEC axes on the sensor, in degrees (`atan2` of each column).
    pub fn axis_angles_deg(&self) -> (f32, f32) {
        let [a, c, b, d] = self.m;
        (c.atan2(a).to_degrees(), d.atan2(b).to_degrees())
    }

    /// Angle between the RA and DEC axes, in degrees (90° = perfectly orthogonal). A value far
    /// from 90° means a skewed calibration — worth re-running.
    pub fn orthogonality_deg(&self) -> f32 {
        let ((rx, ry), (dx, dy)) = self.axis_units();
        (rx * dx + ry * dy).clamp(-1.0, 1.0).acos().to_degrees()
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

/// How the declination axis is allowed to be corrected. Restricting DEC to a single direction is
/// the standard way to defeat gear backlash: once the mount is driving DEC one way, reversing
/// wastes pulses taking up slack, so many setups guide DEC in only the direction drift requires.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DecMode {
    /// Correct DEC in whichever direction the error calls for.
    #[default]
    Auto,
    /// Only ever pulse North (drop South corrections).
    NorthOnly,
    /// Only ever pulse South (drop North corrections).
    SouthOnly,
}

/// Tunable guiding-loop parameters, adjustable live from the UI. RA and DEC are tuned
/// independently because the two axes behave differently (RA has smooth periodic error; DEC has
/// backlash and only drifts slowly).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuideParams {
    /// Fraction of the full correction applied each cycle (0–1) per axis. Lower = gentler.
    pub ra_aggr: f32,
    pub dec_aggr: f32,
    /// Upper bound on any single pulse (ms) per axis, so a spurious error can't fling the mount.
    pub ra_max_pulse_ms: f32,
    pub dec_max_pulse_ms: f32,
    /// Per-axis deadband: don't correct while that axis is off by fewer than this many pixels
    /// (avoids chasing seeing/noise).
    pub ra_min_move_px: f32,
    pub dec_min_move_px: f32,
    /// Control-loop period (ms) — how often corrections are issued, independent of frame rate.
    pub cadence_ms: u64,
    /// Which DEC directions may be corrected (see [`DecMode`]).
    pub dec_mode: DecMode,
}

impl Default for GuideParams {
    fn default() -> Self {
        GuideParams {
            ra_aggr: 0.5,
            dec_aggr: 0.5,
            ra_max_pulse_ms: 500.0,
            dec_max_pulse_ms: 500.0,
            ra_min_move_px: 0.5,
            dec_min_move_px: 0.5,
            cadence_ms: 1000,
            dec_mode: DecMode::Auto,
        }
    }
}

/// Turn a pixel error into the pulse-guide commands to issue this cycle. The calibration inverse
/// gives the RA/DEC pulse durations that would null the error; each axis then independently
/// honours its own deadband, aggressiveness, and max-pulse clamp, and DEC additionally obeys
/// [`DecMode`]. Returns at most one pulse per axis; an empty vec means "nothing to do".
pub fn pulses_for(cal: &Calibration, params: &GuideParams, error_px: (f32, f32)) -> Vec<(Dir, f64)> {
    let mut out = Vec::new();
    if !error_px.0.is_finite() || !error_px.1.is_finite() {
        return out;
    }
    let (ra_ms, dec_ms) = cal.correct(error_px);
    let (ra_scale, dec_scale) = cal.axis_scales();

    // RA axis: +West / −East.
    if let Some((dir, ms)) = axis_pulse(
        ra_ms,
        ra_scale,
        params.ra_min_move_px,
        params.ra_aggr,
        params.ra_max_pulse_ms,
        Dir::West,
        Dir::East,
    ) {
        out.push((dir, ms));
    }
    // DEC axis: +North / −South, subject to the DEC mode.
    if let Some((dir, ms)) = axis_pulse(
        dec_ms,
        dec_scale,
        params.dec_min_move_px,
        params.dec_aggr,
        params.dec_max_pulse_ms,
        Dir::North,
        Dir::South,
    ) {
        let allowed = match params.dec_mode {
            DecMode::Auto => true,
            DecMode::NorthOnly => dir == Dir::North,
            DecMode::SouthOnly => dir == Dir::South,
        };
        if allowed {
            out.push((dir, ms));
        }
    }
    out
}

/// Map a signed correction pulse (`ms`, from the calibration inverse) to a direction + magnitude.
/// `px_per_ms` is the axis scale, used to express the deadband `min_move_px` in pixels: the axis
/// is `ms.abs() * px_per_ms` pixels off. Applies the deadband first, then aggressiveness, then the
/// max clamp; drops sub-millisecond pulses (too short for the mount to act on).
#[allow(clippy::too_many_arguments)]
fn axis_pulse(
    ms: f32,
    px_per_ms: f32,
    min_move_px: f32,
    aggr: f32,
    max_ms: f32,
    positive: Dir,
    negative: Dir,
) -> Option<(Dir, f64)> {
    if ms.abs() * px_per_ms < min_move_px {
        return None;
    }
    let scaled = (ms * aggr) as f64;
    let mag = scaled.abs().min(max_ms as f64);
    if mag < 1.0 {
        return None;
    }
    Some((if scaled >= 0.0 { positive } else { negative }, mag))
}

fn mag((x, y): (f32, f32)) -> f32 {
    (x * x + y * y).sqrt()
}

/// Normalize a vector to unit length; returns `(0, 0)` for a zero vector.
fn unit((x, y): (f32, f32)) -> (f32, f32) {
    let m = mag((x, y));
    if m > 0.0 {
        (x / m, y / m)
    } else {
        (0.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Full-aggressiveness params on both axes, everything else default.
    fn full_aggr() -> GuideParams {
        GuideParams {
            ra_aggr: 1.0,
            dec_aggr: 1.0,
            ..Default::default()
        }
    }

    #[test]
    fn axis_aligned_calibration_corrects_opposite_direction() {
        // West 1000ms → +10px in x; North 1000ms → +10px in y (no rotation).
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // Target drifted +5px in x. To bring it back we must move −x → pulse East.
        let pulses = pulses_for(&cal, &full_aggr(), (5.0, 0.0));
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
            ra_min_move_px: 0.5,
            dec_min_move_px: 0.5,
            ..Default::default()
        };
        // Off by only 0.1px in x and 0.2px in y → both axes inside their deadband.
        assert!(pulses_for(&cal, &params, (0.1, 0.2)).is_empty());
    }

    #[test]
    fn pulses_are_clamped_to_max() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        let params = GuideParams {
            ra_max_pulse_ms: 200.0,
            ..full_aggr()
        };
        // A 100px error would want a 10 000ms pulse; must clamp to 200ms.
        let pulses = pulses_for(&cal, &params, (100.0, 0.0));
        assert_eq!(pulses.len(), 1);
        assert_eq!(pulses[0], (Dir::East, 200.0));
    }

    #[test]
    fn both_axes_pulse_on_diagonal_error() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // Drift +x and −y → correct with East (−x) and North (+y).
        let pulses = pulses_for(&cal, &full_aggr(), (2.0, -2.0));
        assert_eq!(pulses.len(), 2);
        assert_eq!(pulses[0].0, Dir::East);
        assert_eq!(pulses[1].0, Dir::North);
    }

    #[test]
    fn per_axis_aggressiveness_scales_each_pulse() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // Full RA aggressiveness, half on DEC. A ±2px error wants a 200ms pulse per axis.
        let params = GuideParams {
            ra_aggr: 1.0,
            dec_aggr: 0.5,
            ..Default::default()
        };
        let pulses = pulses_for(&cal, &params, (2.0, -2.0));
        assert_eq!(pulses.len(), 2);
        assert_eq!(pulses[0], (Dir::East, 200.0)); // RA at 1.0×
        assert_eq!(pulses[1], (Dir::North, 100.0)); // DEC at 0.5×
    }

    #[test]
    fn dec_mode_suppresses_the_wrong_direction() {
        // North 1000ms moves the target +y, so a +y drift is corrected with a South pulse and a
        // −y drift with a North pulse. (RA error is zero here → no RA pulse.)
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // +y drift wants South; NorthOnly must drop it.
        let north_only = GuideParams {
            dec_mode: DecMode::NorthOnly,
            ..full_aggr()
        };
        assert!(pulses_for(&cal, &north_only, (0.0, 2.0)).is_empty());
        // −y drift wants North; SouthOnly must drop it.
        let south_only = GuideParams {
            dec_mode: DecMode::SouthOnly,
            ..full_aggr()
        };
        assert!(pulses_for(&cal, &south_only, (0.0, -2.0)).is_empty());
        // ...but the allowed direction still fires.
        assert_eq!(
            pulses_for(&cal, &north_only, (0.0, -2.0)),
            vec![(Dir::North, 200.0)]
        );
    }

    #[test]
    fn per_axis_deadband_is_independent() {
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        // RA generously deadbanded, DEC tight. A 1px error on each axis: RA suppressed, DEC fires.
        let params = GuideParams {
            ra_min_move_px: 5.0,
            dec_min_move_px: 0.5,
            ..full_aggr()
        };
        let pulses = pulses_for(&cal, &params, (1.0, 1.0));
        assert_eq!(pulses.len(), 1);
        assert_eq!(pulses[0].0, Dir::South); // +y drift → South
    }

    #[test]
    fn axis_scales_and_units_of_known_calibration() {
        // West 1000ms → +10px x; North 1000ms → +10px y. Scales 0.01 px/ms, axes along +x/+y.
        let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();
        let (sr, sd) = cal.axis_scales();
        assert!((sr - 0.01).abs() < 1e-6 && (sd - 0.01).abs() < 1e-6);
        let ((rx, ry), (dx, dy)) = cal.axis_units();
        assert!((rx - 1.0).abs() < 1e-6 && ry.abs() < 1e-6);
        assert!(dx.abs() < 1e-6 && (dy - 1.0).abs() < 1e-6);
    }

    #[test]
    fn angles_of_rotated_calibration() {
        // RA along +45°, DEC along +135°: orthogonal, but rotated 45° from the sensor axes.
        let cal = Calibration::from_matrix([0.007, 0.007, -0.007, 0.007]);
        let (ra_deg, dec_deg) = cal.axis_angles_deg();
        assert!((ra_deg - 45.0).abs() < 1e-3, "ra {ra_deg}");
        assert!((dec_deg - 135.0).abs() < 1e-3, "dec {dec_deg}");
        assert!((cal.orthogonality_deg() - 90.0).abs() < 1e-3);
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
