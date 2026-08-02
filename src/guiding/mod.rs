//! Stream-based guiding (M2): detect the target on the main video stream and issue timed
//! pulse-guide corrections to keep it locked. See the submodules for the pure math; this root
//! holds the shared [`GuideSample`] type and the two async routines the worker drives — the
//! automatic [`run_calibration`] and the [`run_guide_loop`] control loop.

pub mod controller;
pub mod detector;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::time::sleep;

pub use controller::{pulses_for, Calibration, DecMode, GuideParams};
pub use detector::{GuideDetector, GuideMode};

use crate::bus::{Bus, Dir};
use crate::indi::mount::Mount;

/// A single target-position measurement produced by the detector, in frame pixels.
#[derive(Clone, Copy, Debug)]
pub struct GuideSample {
    pub x: f32,
    pub y: f32,
    /// Frame sequence number the measurement came from (lets consumers wait for a *fresh* one).
    pub seq: u64,
}

/// Default calibration-pulse duration (ms) — long enough to move the target measurably at guide
/// rate. The UI can override it per run; a longer pulse gives a bigger, more precise displacement
/// on slow mounts, a shorter one keeps the target inside the frame on fast ones.
pub const DEFAULT_CALIB_MS: f32 = 1500.0;
/// How long to wait for the mount to settle and a fresh frame to arrive after a pulse.
const SETTLE: Duration = Duration::from_millis(800);
/// Cap on the guide-error history retained for the graph.
const HISTORY_CAP: usize = 600;

/// Poll for the next detector sample newer than `after_seq`, up to `timeout`.
pub(crate) async fn next_sample(bus: &Bus, after_seq: u64, timeout: Duration) -> Option<GuideSample> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(s) = bus.guide_sample.load_full() {
            if s.seq > after_seq {
                return Some(*s);
            }
        }
        if Instant::now() >= deadline {
            return None;
        }
        sleep(Duration::from_millis(50)).await;
    }
}

/// Pulse `dir` for `pulse_ms`, wait for the mount to settle, and return the resulting target
/// position (the next fresh sample), or `None` if the pulse failed or no sample arrived.
async fn pulse_and_measure(
    mount: &Mount,
    bus: &Bus,
    dir: Dir,
    after_seq: u64,
    pulse_ms: f32,
) -> Option<GuideSample> {
    if let Err(e) = mount.pulse_guide(dir, pulse_ms as f64).await {
        bus.log(format!("calibration pulse {dir:?} failed: {e}"));
        return None;
    }
    // The pulse itself takes ~pulse_ms, then let the frame catch up.
    sleep(Duration::from_millis(pulse_ms as u64) + SETTLE).await;
    next_sample(bus, after_seq, Duration::from_secs(3)).await
}

/// Automatic pulse-based calibration: measure how a West pulse and a North pulse move the target
/// on the sensor, build the 2×2 calibration matrix, and store it in [`crate::bus::Shared`].
/// `pulse_ms` is the per-move pulse duration. Detection must already be enabled by the caller.
/// Best-effort returns the mount near its start.
pub async fn run_calibration(mount: Mount, bus: Bus, ctx: egui::Context, pulse_ms: f32) {
    {
        let mut sh = bus.shared.lock().unwrap();
        sh.calibrating = true;
    }
    bus.refresh_detect(); // ensure detection is running for the measurements
    ctx.request_repaint();
    bus.log(format!("calibrating… ({pulse_ms:.0}ms pulses)"));

    let result = calibrate(&mount, &bus, pulse_ms).await;
    {
        // NOTE: use `sh.log` (not `bus.log`) here — we hold `shared` and the std `Mutex` is not
        // reentrant, so `bus.log` (which re-locks) would deadlock.
        let mut sh = bus.shared.lock().unwrap();
        sh.calibrating = false;
        match result {
            Some(cal) => {
                sh.guide_calib = Some(cal);
                sh.calibrated = true;
                sh.log(format!("calibrated: matrix {:?}", cal.matrix()));
            }
            None => sh.log("calibration failed — check the target is visible and try again"),
        }
    }
    bus.refresh_detect(); // may turn detection back off if overlay/guiding are inactive
    ctx.request_repaint();
}

/// The calibration measurement sequence (West/East out-and-back, then North/South).
async fn calibrate(mount: &Mount, bus: &Bus, pulse_ms: f32) -> Option<Calibration> {
    let m0 = next_sample(bus, 0, Duration::from_secs(3)).await?;
    // RA axis: pulse West, measure displacement, pulse East to return.
    let m_w = pulse_and_measure(mount, bus, Dir::West, m0.seq, pulse_ms).await?;
    let dpx_west = (m_w.x - m0.x, m_w.y - m0.y);
    let m_back = pulse_and_measure(mount, bus, Dir::East, m_w.seq, pulse_ms).await?;

    // DEC axis: pulse North, measure, pulse South to return.
    let m_n = pulse_and_measure(mount, bus, Dir::North, m_back.seq, pulse_ms).await?;
    let dpx_north = (m_n.x - m_back.x, m_n.y - m_back.y);
    let _ = pulse_and_measure(mount, bus, Dir::South, m_n.seq, pulse_ms).await;

    bus.log(format!(
        "calibration moves: W {:.1},{:.1}px  N {:.1},{:.1}px",
        dpx_west.0, dpx_west.1, dpx_north.0, dpx_north.1
    ));
    Calibration::from_pulses(dpx_west, dpx_north, pulse_ms)
}

/// The guiding control loop: at the configured cadence, read the latest target measurement,
/// compute the error against the lock point, and issue the correcting pulse-guides. Runs until
/// `stop` is set. Reads params/lock/calibration live from [`crate::bus::Shared`].
pub async fn run_guide_loop(mount: Mount, bus: Bus, ctx: egui::Context, stop: Arc<AtomicBool>) {
    while !stop.load(Ordering::Relaxed) {
        let (params, lock, calib) = {
            let sh = bus.shared.lock().unwrap();
            (sh.guide_params, sh.lock_point, sh.guide_calib)
        };
        sleep(Duration::from_millis(params.cadence_ms.max(50))).await;
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let (Some(lock), Some(calib)) = (lock, calib) else {
            continue;
        };
        let Some(sample) = bus.guide_sample.load_full() else {
            continue;
        };
        // Raw sensor error drives the correction; the telemetry is the same error decomposed onto
        // the mount's RA/DEC axes (what guiders actually graph), which is meaningful even when the
        // camera is rotated relative to the sensor.
        let err_px = (sample.x - lock.0, sample.y - lock.1);
        let ((rx, ry), (dx, dy)) = calib.axis_units();
        let err_mount = (
            err_px.0 * rx + err_px.1 * ry,
            err_px.0 * dx + err_px.1 * dy,
        );

        {
            let mut sh = bus.shared.lock().unwrap();
            sh.guide_err = Some(err_mount);
            sh.guide_history.push_back(err_mount);
            while sh.guide_history.len() > HISTORY_CAP {
                sh.guide_history.pop_front();
            }
            let s = stats(&sh.guide_history);
            sh.guide_rms = s.total;
            sh.guide_rms_ra = s.ra;
            sh.guide_rms_dec = s.dec;
            sh.guide_peak = s.peak;
        }

        for (dir, ms) in pulses_for(&calib, &params, err_px) {
            if let Err(e) = mount.pulse_guide(dir, ms).await {
                bus.log(format!("guide pulse {dir:?} failed: {e}"));
            }
        }
        ctx.request_repaint();
    }
}

/// Error statistics over the history window, in mount-frame pixels.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GuideStats {
    /// RMS of the combined error magnitude.
    pub total: f32,
    /// RMS of the RA (x) component.
    pub ra: f32,
    /// RMS of the DEC (y) component.
    pub dec: f32,
    /// Largest single error magnitude in the window.
    pub peak: f32,
}

/// Compute per-axis and total RMS plus the peak error over the history window.
fn stats(history: &std::collections::VecDeque<(f32, f32)>) -> GuideStats {
    if history.is_empty() {
        return GuideStats::default();
    }
    let n = history.len() as f32;
    let (mut sx, mut sy, mut peak) = (0.0f32, 0.0f32, 0.0f32);
    for &(x, y) in history {
        sx += x * x;
        sy += y * y;
        peak = peak.max((x * x + y * y).sqrt());
    }
    GuideStats {
        total: ((sx + sy) / n).sqrt(),
        ra: (sx / n).sqrt(),
        dec: (sy / n).sqrt(),
        peak,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn stats_of_known_errors() {
        let mut h = VecDeque::new();
        h.push_back((3.0, 4.0)); // magnitude 5
        h.push_back((0.0, 0.0)); // magnitude 0
        let s = stats(&h);
        // total = sqrt((9+16)/2) = sqrt(12.5); ra = sqrt(9/2); dec = sqrt(16/2); peak = 5.
        assert!((s.total - 12.5f32.sqrt()).abs() < 1e-4);
        assert!((s.ra - 4.5f32.sqrt()).abs() < 1e-4);
        assert!((s.dec - 8.0f32.sqrt()).abs() < 1e-4);
        assert!((s.peak - 5.0).abs() < 1e-4);
    }
}
