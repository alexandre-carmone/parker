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

pub use controller::{pulses_for, Calibration, GuideParams};
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

/// Duration of each calibration pulse. Long enough to move the target measurably at guide rate.
const CALIB_MS: f32 = 1500.0;
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

/// Pulse `dir` for `CALIB_MS`, wait for the mount to settle, and return the resulting target
/// position (the next fresh sample), or `None` if the pulse failed or no sample arrived.
async fn pulse_and_measure(mount: &Mount, bus: &Bus, dir: Dir, after_seq: u64) -> Option<GuideSample> {
    if let Err(e) = mount.pulse_guide(dir, CALIB_MS as f64).await {
        bus.log(format!("calibration pulse {dir:?} failed: {e}"));
        return None;
    }
    // The pulse itself takes ~CALIB_MS, then let the frame catch up.
    sleep(Duration::from_millis(CALIB_MS as u64) + SETTLE).await;
    next_sample(bus, after_seq, Duration::from_secs(3)).await
}

/// Automatic pulse-based calibration: measure how a West pulse and a North pulse move the target
/// on the sensor, build the 2×2 calibration matrix, and store it in [`crate::bus::Shared`].
/// Detection must already be enabled by the caller. Best-effort returns the mount near its start.
pub async fn run_calibration(mount: Mount, bus: Bus, ctx: egui::Context) {
    {
        let mut sh = bus.shared.lock().unwrap();
        sh.calibrating = true;
    }
    bus.refresh_detect(); // ensure detection is running for the measurements
    ctx.request_repaint();
    bus.log("calibrating…");

    let result = calibrate(&mount, &bus).await;
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
async fn calibrate(mount: &Mount, bus: &Bus) -> Option<Calibration> {
    let m0 = next_sample(bus, 0, Duration::from_secs(3)).await?;
    // RA axis: pulse West, measure displacement, pulse East to return.
    let m_w = pulse_and_measure(mount, bus, Dir::West, m0.seq).await?;
    let dpx_west = (m_w.x - m0.x, m_w.y - m0.y);
    let m_back = pulse_and_measure(mount, bus, Dir::East, m_w.seq).await?;

    // DEC axis: pulse North, measure, pulse South to return.
    let m_n = pulse_and_measure(mount, bus, Dir::North, m_back.seq).await?;
    let dpx_north = (m_n.x - m_back.x, m_n.y - m_back.y);
    let _ = pulse_and_measure(mount, bus, Dir::South, m_n.seq).await;

    bus.log(format!(
        "calibration moves: W {:.1},{:.1}px  N {:.1},{:.1}px",
        dpx_west.0, dpx_west.1, dpx_north.0, dpx_north.1
    ));
    Calibration::from_pulses(dpx_west, dpx_north, CALIB_MS)
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
        let err = (sample.x - lock.0, sample.y - lock.1);

        {
            let mut sh = bus.shared.lock().unwrap();
            sh.guide_err = Some(err);
            sh.guide_history.push_back(err);
            while sh.guide_history.len() > HISTORY_CAP {
                sh.guide_history.pop_front();
            }
            sh.guide_rms = rms(&sh.guide_history);
        }

        for (dir, ms) in pulses_for(&calib, &params, err) {
            if let Err(e) = mount.pulse_guide(dir, ms).await {
                bus.log(format!("guide pulse {dir:?} failed: {e}"));
            }
        }
        ctx.request_repaint();
    }
}

/// RMS of the error-magnitude over the history window.
fn rms(history: &std::collections::VecDeque<(f32, f32)>) -> f32 {
    if history.is_empty() {
        return 0.0;
    }
    let sum: f32 = history.iter().map(|(x, y)| x * x + y * y).sum();
    (sum / history.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn rms_of_known_errors() {
        let mut h = VecDeque::new();
        h.push_back((3.0, 4.0)); // magnitude 5
        h.push_back((0.0, 0.0)); // magnitude 0
        // sqrt((25 + 0)/2) = sqrt(12.5)
        assert!((rms(&h) - 12.5f32.sqrt()).abs() < 1e-4);
    }
}
