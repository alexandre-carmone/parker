//! Message types and shared state connecting the GUI thread and the async INDI worker.
//!
//! - GUI -> worker: [`Command`] over an unbounded mpsc channel.
//! - worker -> GUI (frames): an [`arc_swap::ArcSwapOption`] holding only the latest [`Frame`]
//!   (high-FPS streams drop stale frames for the live view).
//! - worker -> GUI (display): a second `ArcSwapOption` holding the latest display-ready
//!   [`egui::ColorImage`]. The worker (a background thread) does the stretch + `Color32`
//!   conversion so the GUI thread only uploads it — keeping the UI responsive at high FPS.
//! - worker -> GUI (state): [`Shared`] behind a std `Mutex`, read each repaint.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwapOption;
use egui::ColorImage;

use crate::frame::Frame;
use crate::guiding::{GuideMode, GuideParams, GuideSample};

/// Cardinal nudge directions for manual mount slewing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    North,
    South,
    East,
    West,
}

/// Commands sent from the GUI to the async INDI worker.
#[derive(Clone, Debug)]
pub enum Command {
    Connect { addr: String },
    Disconnect,
    StartStream,
    StopStream,
    /// Bind the camera to a different CCD device (by INDI device name).
    SelectCamera(String),
    /// Bind the mount to a different telescope device (by INDI device name).
    SelectMount(String),
    SetGain(f64),
    SetExposure(f64),
    /// Press (`active = true`) or release (`false`) a directional nudge.
    Nudge { dir: Dir, active: bool },
    /// Slew rate as an index into the driver's `TELESCOPE_SLEW_RATE` switch (0 = slowest).
    SetSlewRate(usize),
    SetTracking(bool),
    Abort,
    /// Save the current live frame to a timestamped PNG in `dir`.
    CaptureFrame { dir: String },
    /// Restrict the camera readout to a subframe (ROI) in sensor pixels via `CCD_FRAME`.
    SetRoi { x: u32, y: u32, w: u32, h: u32 },
    /// Reset the camera readout to the full sensor.
    ResetRoi,

    // ---- guiding (M2) ----
    /// Run the automatic pulse-based calibration (measures the pixel→mount mapping).
    Calibrate,
    /// Begin auto-guiding: lock onto the current target and correct drift with pulse-guides.
    StartGuiding,
    /// Stop auto-guiding.
    StopGuiding,
    /// Re-acquire the lock point (and Surface reference patch) at the current target position.
    Relock,
    /// Choose the detection mode (Disk centroid vs. Surface cross-correlation).
    SetGuideMode(GuideMode),
    /// Update the live guiding-loop parameters.
    SetGuideParams(GuideParams),
    /// Turn per-frame target detection on/off for the on-screen overlay (independent of guiding).
    SetDetectionOverlay(bool),
}

/// Coarse connection lifecycle state shown in the UI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnState {
    Disconnected,
    Connecting,
    Connected,
    Failed,
}

/// Snapshot of worker/device state that the GUI renders. Kept small; locked only briefly.
pub struct Shared {
    pub conn: ConnState,
    pub streaming: bool,
    pub fps: f32,
    pub frame_count: u64,
    /// Camera gain (driver units) and exposure (seconds), as last known.
    pub gain: f64,
    pub exposure: f64,
    /// All available CCD / telescope device names, and the currently-selected one of each.
    pub cameras: Vec<String>,
    pub mounts: Vec<String>,
    pub camera_sel: String,
    pub mount_sel: String,
    /// Available slew-rate labels and the selected index.
    pub slew_rates: Vec<String>,
    pub slew_rate_idx: usize,
    pub tracking: bool,
    pub last_capture: Option<String>,
    /// Full sensor size in pixels (`CCD_INFO`), 0 until a camera is bound. Bounds the ROI controls.
    pub sensor_w: u32,
    pub sensor_h: u32,
    /// Currently-applied readout region `(x, y, w, h)` in sensor pixels (full sensor by default).
    pub roi: (u32, u32, u32, u32),

    // ---- guiding (M2) telemetry, in frame pixels ----
    /// Whether the guide loop is currently running.
    pub guiding: bool,
    /// Whether a calibration run is in progress.
    pub calibrating: bool,
    /// Whether a valid calibration has been obtained (required before guiding).
    pub calibrated: bool,
    /// The current pixel→mount calibration (present once `calibrated`).
    pub guide_calib: Option<crate::guiding::Calibration>,
    /// The locked reference position guiding steers the target back toward.
    pub lock_point: Option<(f32, f32)>,
    /// Most recent detected target position (for the on-screen overlay).
    pub detected: Option<(f32, f32)>,
    /// Most recent guide error `(dx, dy) = detected − lock_point`.
    pub guide_err: Option<(f32, f32)>,
    /// RMS of the recent guide error magnitude (pixels).
    pub guide_rms: f32,
    /// Rolling guide-error history `(dx, dy)` for the graph, capped.
    pub guide_history: VecDeque<(f32, f32)>,
    /// Live guiding-loop parameters (edited in the UI, read by the guide loop).
    pub guide_params: GuideParams,
    /// Current detection mode (mirrors [`Bus::guide_mode`] for the UI).
    pub guide_mode: GuideMode,
    /// UI toggle: run detection for the on-screen overlay even when not guiding.
    pub detect_overlay: bool,

    /// Rolling log (most recent last), capped.
    pub log: VecDeque<String>,
}

impl Default for Shared {
    fn default() -> Self {
        Shared {
            conn: ConnState::Disconnected,
            streaming: false,
            fps: 0.0,
            frame_count: 0,
            gain: 0.0,
            exposure: 0.0,
            cameras: Vec::new(),
            mounts: Vec::new(),
            camera_sel: String::new(),
            mount_sel: String::new(),
            slew_rates: Vec::new(),
            slew_rate_idx: 0,
            tracking: false,
            last_capture: None,
            sensor_w: 0,
            sensor_h: 0,
            roi: (0, 0, 0, 0),
            guiding: false,
            calibrating: false,
            calibrated: false,
            guide_calib: None,
            lock_point: None,
            detected: None,
            guide_err: None,
            guide_rms: 0.0,
            guide_history: VecDeque::new(),
            guide_params: GuideParams::default(),
            guide_mode: GuideMode::Disk,
            detect_overlay: false,
            log: VecDeque::new(),
        }
    }
}

impl Shared {
    pub fn log(&mut self, msg: impl Into<String>) {
        let msg = msg.into();
        tracing::info!("{msg}");
        self.log.push_back(msg);
        while self.log.len() > 200 {
            self.log.pop_front();
        }
    }
}

/// Handles shared between the GUI and the worker.
#[derive(Clone)]
pub struct Bus {
    pub shared: Arc<Mutex<Shared>>,
    /// Latest raw decoded frame (kept for full-quality captures + paused re-stretch).
    pub latest_frame: Arc<ArcSwapOption<Frame>>,
    /// Latest display-ready image, produced off the GUI thread by the worker.
    pub display: Arc<ArcSwapOption<ColorImage>>,
    /// Bumped whenever `display` is replaced, so the GUI re-uploads only on a new image.
    pub display_seq: Arc<AtomicU64>,
    /// Live-view stretch settings, read lock-free by the worker each frame. `display_gain`
    /// holds an `f32` via its bit pattern.
    pub auto_stretch: Arc<AtomicBool>,
    pub display_gain: Arc<AtomicU32>,

    // ---- guiding (M2), read lock-free by the decode thread / guide loop ----
    /// Newest target measurement from the detector (latest-wins, like `latest_frame`).
    pub guide_sample: Arc<ArcSwapOption<GuideSample>>,
    /// When set, the decode thread runs target detection (guiding, calibrating, or overlay on).
    pub detect_enabled: Arc<AtomicBool>,
    /// Detection mode as [`GuideMode::as_u8`], read by the decode-thread detector.
    pub guide_mode: Arc<AtomicU8>,
    /// Bumped on guide start / re-lock to make the Surface detector recapture its reference patch.
    pub ref_generation: Arc<AtomicU64>,
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            shared: Arc::new(Mutex::new(Shared::default())),
            latest_frame: Arc::new(ArcSwapOption::empty()),
            display: Arc::new(ArcSwapOption::empty()),
            display_seq: Arc::new(AtomicU64::new(0)),
            auto_stretch: Arc::new(AtomicBool::new(true)),
            display_gain: Arc::new(AtomicU32::new(1.0f32.to_bits())),
            guide_sample: Arc::new(ArcSwapOption::empty()),
            detect_enabled: Arc::new(AtomicBool::new(false)),
            guide_mode: Arc::new(AtomicU8::new(GuideMode::Disk.as_u8())),
            ref_generation: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Convenience: append a log line, ignoring lock poisoning.
    pub fn log(&self, msg: impl Into<String>) {
        if let Ok(mut s) = self.shared.lock() {
            s.log(msg);
        }
    }

    /// GUI -> worker: update the live-view stretch settings the worker applies to new frames.
    pub fn set_display_settings(&self, auto: bool, gain: f32) {
        self.auto_stretch.store(auto, Ordering::Relaxed);
        self.display_gain.store(gain.to_bits(), Ordering::Relaxed);
    }

    /// Worker: current stretch settings to apply to the next frame.
    pub fn display_settings(&self) -> (bool, f32) {
        (
            self.auto_stretch.load(Ordering::Relaxed),
            f32::from_bits(self.display_gain.load(Ordering::Relaxed)),
        )
    }

    /// Worker: publish a freshly rendered display image and signal the GUI to upload it.
    pub fn publish_display(&self, img: ColorImage) {
        self.display.store(Some(Arc::new(img)));
        self.display_seq.fetch_add(1, Ordering::Release);
    }

    /// Recompute whether the decode thread should run detection from the current state:
    /// on while guiding, calibrating, or the overlay toggle is set. Must NOT be called while
    /// holding `shared` (it locks it).
    pub fn refresh_detect(&self) {
        let on = if let Ok(sh) = self.shared.lock() {
            sh.detect_overlay || sh.calibrating || sh.guiding
        } else {
            false
        };
        self.detect_enabled.store(on, Ordering::Relaxed);
    }

    /// Whether the decode thread should run detection this frame.
    pub fn detect_enabled(&self) -> bool {
        self.detect_enabled.load(Ordering::Relaxed)
    }

    /// Set the detection mode for the decode-thread detector.
    pub fn set_guide_mode(&self, mode: GuideMode) {
        self.guide_mode.store(mode.as_u8(), Ordering::Relaxed);
    }

    /// Signal the Surface detector to recapture its reference patch, returning the new generation.
    pub fn bump_ref_generation(&self) -> u64 {
        self.ref_generation.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// Decode thread: publish the newest target measurement.
    pub fn publish_guide_sample(&self, sample: GuideSample) {
        self.guide_sample.store(Some(Arc::new(sample)));
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_settings_round_trip() {
        let bus = Bus::new();
        assert_eq!(bus.display_settings(), (true, 1.0)); // defaults
        bus.set_display_settings(false, 4.5);
        assert_eq!(bus.display_settings(), (false, 4.5));
    }

    #[test]
    fn publish_display_bumps_seq_and_stores() {
        let bus = Bus::new();
        assert!(bus.display.load_full().is_none());
        let before = bus.display_seq.load(Ordering::Acquire);
        bus.publish_display(ColorImage::new([1, 1], vec![egui::Color32::WHITE]));
        assert_eq!(bus.display_seq.load(Ordering::Acquire), before + 1);
        assert_eq!(bus.display.load_full().unwrap().size, [1, 1]);
    }
}
