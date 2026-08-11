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
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering};
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
    /// Tracking mode as an index into the driver's `TELESCOPE_TRACK_MODE` switch (sidereal/solar/…).
    SetTrackMode(usize),
    SetTracking(bool),
    /// Park (`true`) or unpark (`false`) the mount via `TELESCOPE_PARK`.
    SetPark(bool),
    /// Slew to (and track) a solar-system object, computing its current RA/Dec from the system
    /// clock and the mount's geographic location.
    GotoObject(crate::ephemeris::SolarObject),
    Abort,
    /// Save the current live frame to a timestamped PNG in `dir`.
    CaptureFrame { dir: String },
    /// Restrict the camera readout to a subframe (ROI) in sensor pixels via `CCD_FRAME`.
    SetRoi { x: u32, y: u32, w: u32, h: u32 },
    /// Reset the camera readout to the full sensor.
    ResetRoi,
    /// Set symmetric on-sensor binning (`CCD_BINNING` `HOR_BIN`/`VER_BIN`), 1 = unbinned.
    SetBinning { bin: u32 },

    // ---- recording (M3) ----
    /// Turn on element `elem` of the camera switch property `prop` — used for the driver's
    /// stream-format controls (encoder, video format, bit depth, sensor mode). These are
    /// driver-specific and govern the streamed bit depth the SER recorder captures.
    SetCameraSwitch { prop: String, elem: String },
    /// Begin recording the stream to SER — a sequence of `count` videos (see [`RecordConfig`]).
    StartRecording(RecordConfig),
    /// Stop recording (finalizing the current SER file).
    StopRecording,

    // ---- guiding (M2) ----
    /// Run the automatic pulse-based calibration (measures the pixel→mount mapping). `pulse_ms`
    /// is the per-move pulse duration.
    Calibrate { pulse_ms: f32 },
    /// Discard the current calibration (forces a re-calibrate before guiding again).
    ClearCalibration,
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

    // ---- generic INDI control panel ----
    /// Set switch element(s) `elems` of switch property `prop` on device `device` to the given
    /// on/off states. For `OneOfMany`/`AtMostOne` rules the driver clears siblings; for
    /// `AnyOfMany` each element toggles independently.
    SetIndiSwitch {
        device: String,
        prop: String,
        elems: Vec<(String, bool)>,
    },
    /// Set number element(s) `elems` of number property `prop` on device `device`.
    SetIndiNumber {
        device: String,
        prop: String,
        elems: Vec<(String, f64)>,
    },
    /// Set text element(s) `elems` of text property `prop` on device `device`.
    SetIndiText {
        device: String,
        prop: String,
        elems: Vec<(String, String)>,
    },
}

/// A device's full INDI property tree, mirrored from the driver into plain (crate-decoupled)
/// types for the generic control panel. Rebuilt wholesale by the worker on each refresh and
/// handed to the GUI behind an `Arc` (see [`Shared::camera_panel`]).
#[derive(Clone, Debug, Default)]
pub struct IndiPanel {
    pub device: String,
    pub groups: Vec<IndiGroup>,
}

/// One INDI property group (rendered as a collapsing section).
#[derive(Clone, Debug, Default)]
pub struct IndiGroup {
    pub name: String,
    pub props: Vec<IndiProp>,
}

/// One INDI property (a vector of typed elements) plus its display metadata.
#[derive(Clone, Debug)]
pub struct IndiProp {
    /// INDI property name (used when sending a change).
    pub name: String,
    /// Human-friendly label (falls back to the name).
    pub label: String,
    /// Property state, for the status LED.
    pub state: IndiState,
    /// Whether the property is writable (`RW`/`WO`).
    pub writable: bool,
    /// The typed value(s).
    pub value: IndiValue,
}

/// INDI property state (`PropertyState`), for the status LED.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum IndiState {
    #[default]
    Idle,
    Ok,
    Busy,
    Alert,
}

/// The typed elements of an INDI property, mirrored by kind.
#[derive(Clone, Debug)]
pub enum IndiValue {
    Number(Vec<IndiNumber>),
    Switch {
        rule: IndiSwitchRule,
        items: Vec<IndiSwitch>,
    },
    Text(Vec<IndiText>),
    Light(Vec<IndiLight>),
    /// BLOB properties: element labels only (read-only, not editable here).
    Blob(Vec<String>),
}

#[derive(Clone, Debug)]
pub struct IndiNumber {
    pub name: String,
    pub label: String,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub step: f64,
    /// INDI printf/sexagesimal format string.
    pub format: String,
}

#[derive(Clone, Debug)]
pub struct IndiSwitch {
    pub name: String,
    pub label: String,
    pub on: bool,
}

/// INDI switch rule (`SwitchRule`), driving radio vs. checkbox rendering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IndiSwitchRule {
    OneOfMany,
    AtMostOne,
    AnyOfMany,
}

#[derive(Clone, Debug)]
pub struct IndiText {
    pub name: String,
    pub label: String,
    pub value: String,
}

#[derive(Clone, Debug)]
pub struct IndiLight {
    pub name: String,
    pub label: String,
    pub state: IndiState,
}

/// One of the camera's stream-format switch properties (e.g. `CCD_STREAM_ENCODER`,
/// `CCD_VIDEO_FORMAT`, `STREAM_FULL_DEPTH`, `SENSOR_MODE`) surfaced to the UI so the user can
/// set the streamed pixel format / bit depth. Names are driver-specific and shown as-is.
#[derive(Clone, Debug, Default)]
pub struct CameraSwitch {
    /// INDI property name (used when sending a change).
    pub prop: String,
    /// Human-friendly label for the dropdown.
    pub label: String,
    /// Element names (options), sorted.
    pub options: Vec<String>,
    /// Currently-selected element name.
    pub selected: String,
}

/// What ends each video in a recording sequence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RecordStop {
    /// Stop after this many frames have been written.
    Frames(u64),
    /// Stop after this many seconds of recording.
    Seconds(f64),
}

/// Parameters for a recording run: a sequence of `count` SER videos, each ended by `stop`, with
/// `delay_secs` of (still-streaming) pause between them.
#[derive(Clone, Debug)]
pub struct RecordConfig {
    /// Output folder for the `.ser` files.
    pub dir: String,
    /// Number of videos in the sequence (>= 1).
    pub count: usize,
    /// Per-video stop condition.
    pub stop: RecordStop,
    /// Delay between consecutive videos (seconds); ignored after the last one.
    pub delay_secs: f64,
}

/// Phase of the recording orchestrator, for the UI.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RecordPhase {
    #[default]
    Idle,
    /// Actively writing frames to the current video.
    Recording,
    /// Between videos, waiting out the inter-video delay (stream still running).
    Waiting,
}

/// Recording progress mirrored to the GUI.
#[derive(Clone, Debug, Default)]
pub struct RecordStatus {
    /// Whether a recording sequence is currently running.
    pub active: bool,
    pub phase: RecordPhase,
    /// 1-based index of the current video and total in the sequence.
    pub current: usize,
    pub total: usize,
    /// Frames written to the current video.
    pub frames_written: u64,
    /// Frames skipped because their size didn't match the SER header (e.g. geometry changed).
    pub dropped: u64,
    /// Seconds elapsed in the current video.
    pub elapsed_secs: f64,
    /// Path of the most recently finalized `.ser` file.
    pub last_file: Option<String>,
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
    /// Available slew rates as `(element name, display label)` pairs, and the selected index.
    pub slew_rates: Vec<(String, String)>,
    pub slew_rate_idx: usize,
    /// Available tracking modes as `(element name, display label)` pairs, and the selected index.
    /// Empty when the mount doesn't expose `TELESCOPE_TRACK_MODE`.
    pub track_modes: Vec<(String, String)>,
    pub track_mode_idx: usize,
    pub tracking: bool,
    /// Whether the mount exposes `TELESCOPE_PARK` (park controls hidden when false).
    pub can_park: bool,
    /// Whether the mount is currently parked.
    pub parked: bool,
    pub last_capture: Option<String>,
    /// Full sensor size in pixels (`CCD_INFO`), 0 until a camera is bound. Bounds the ROI controls.
    pub sensor_w: u32,
    pub sensor_h: u32,
    /// Currently-applied readout region `(x, y, w, h)` in sensor pixels (full sensor by default).
    pub roi: (u32, u32, u32, u32),
    /// Currently-applied symmetric binning factor (`CCD_BINNING`), 1 = unbinned.
    pub binning: u32,

    // ---- recording (M3) ----
    /// The camera's stream-format switch properties (encoder, video format, bit depth, sensor
    /// mode) that the driver exposes — whichever are present. Populated on connect / camera swap.
    pub stream_switches: Vec<CameraSwitch>,
    /// Live recording progress.
    pub recording: RecordStatus,

    // ---- generic INDI control panel ----
    /// Full property tree of the bound camera / mount, mirrored for the generic control panel.
    /// `None` until bound; refreshed on connect/select, after writes, and on a ~1 Hz tick.
    pub camera_panel: Option<Arc<IndiPanel>>,
    pub mount_panel: Option<Arc<IndiPanel>>,

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
    /// Most recent guide error `(ra, dec)`, in mount-frame pixels (the sensor error projected onto
    /// the calibration axes).
    pub guide_err: Option<(f32, f32)>,
    /// RMS of the recent guide error magnitude (pixels).
    pub guide_rms: f32,
    /// RMS of the RA and DEC error components separately (mount-frame pixels).
    pub guide_rms_ra: f32,
    pub guide_rms_dec: f32,
    /// Largest single error magnitude in the history window (pixels).
    pub guide_peak: f32,
    /// Rolling guide-error history `(ra, dec)` (mount-frame pixels) for the graph, capped.
    pub guide_history: VecDeque<(f32, f32)>,
    /// Live guiding-loop parameters (edited in the UI, read by the guide loop).
    pub guide_params: GuideParams,
    /// Current detection mode (mirrors [`Bus::guide_mode`] for the UI).
    pub guide_mode: GuideMode,
    /// UI toggle: run detection for the on-screen overlay even when not guiding.
    pub detect_overlay: bool,

    // ---- focus measurement ----
    /// Smoothed (EMA) sharpness metric of the current frame/ROI. 0 until measuring.
    pub focus_metric: f32,
    /// Best (highest) smoothed metric seen since the last reset — the peak-hold.
    pub focus_peak: f32,
    /// Rolling history of the smoothed metric for the focus curve, capped.
    pub focus_history: VecDeque<f32>,

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
            track_modes: Vec::new(),
            track_mode_idx: 0,
            tracking: false,
            can_park: false,
            parked: false,
            last_capture: None,
            sensor_w: 0,
            sensor_h: 0,
            roi: (0, 0, 0, 0),
            binning: 1,
            stream_switches: Vec::new(),
            recording: RecordStatus::default(),
            camera_panel: None,
            mount_panel: None,
            guiding: false,
            calibrating: false,
            calibrated: false,
            guide_calib: None,
            lock_point: None,
            detected: None,
            guide_err: None,
            guide_rms: 0.0,
            guide_rms_ra: 0.0,
            guide_rms_dec: 0.0,
            guide_peak: 0.0,
            guide_history: VecDeque::new(),
            guide_params: GuideParams::default(),
            guide_mode: GuideMode::Disk,
            detect_overlay: false,
            focus_metric: 0.0,
            focus_peak: 0.0,
            focus_history: VecDeque::new(),
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
    /// Max preview refresh rate (fps), read lock-free by the decode thread to rate-limit the
    /// stretch + upload. Holds an `f32` via its bit pattern. `<= 0` means "every frame".
    pub preview_fps: Arc<AtomicU32>,

    // ---- guiding (M2), read lock-free by the decode thread / guide loop ----
    /// Newest target measurement from the detector (latest-wins, like `latest_frame`).
    pub guide_sample: Arc<ArcSwapOption<GuideSample>>,
    /// When set, the decode thread runs target detection (guiding, calibrating, or overlay on).
    pub detect_enabled: Arc<AtomicBool>,
    /// Detection mode as [`GuideMode::as_u8`], read by the decode-thread detector.
    pub guide_mode: Arc<AtomicU8>,
    /// Bumped on guide start / re-lock to make the Surface detector recapture its reference patch.
    pub ref_generation: Arc<AtomicU64>,

    // ---- recording (M3), read lock-free by the decode thread ----
    /// Current readout geometry, so the decode thread can interpret dimensionless raw frames.
    pub frame_w: Arc<AtomicU32>,
    pub frame_h: Arc<AtomicU32>,
    /// Full sensor size (`(0, 0)` until known). Some drivers stream the full sensor even after a
    /// subframe is set, so the decode thread uses this as a fallback geometry when a raw frame's
    /// byte length doesn't match the requested ROI.
    pub sensor_w: Arc<AtomicU32>,
    pub sensor_h: Arc<AtomicU32>,
    /// Byte length of the most recent raw (decompressed) frame — lets the UI infer the stream's
    /// bit depth (bytes-per-pixel) for display.
    pub last_raw_len: Arc<AtomicUsize>,

    // ---- focus measurement, read lock-free by the decode thread ----
    /// When set, the decode thread measures per-frame sharpness (over the current ROI/frame).
    pub focus_enabled: Arc<AtomicBool>,
    /// Set by the GUI to ask the decode thread to clear its peak-hold / EMA / history.
    pub focus_reset: Arc<AtomicBool>,

    /// Wall-clock (epoch millis) when the frame reader last handed a raw frame to the decoder.
    /// Stamped by the reader task at wire speed; read by the worker's 1 Hz stall watchdog to
    /// detect a stream that has silently stopped delivering frames. `0` until the first frame.
    pub last_frame_ms: Arc<AtomicU64>,
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
            preview_fps: Arc::new(AtomicU32::new(1.0f32.to_bits())),
            guide_sample: Arc::new(ArcSwapOption::empty()),
            detect_enabled: Arc::new(AtomicBool::new(false)),
            guide_mode: Arc::new(AtomicU8::new(GuideMode::Disk.as_u8())),
            ref_generation: Arc::new(AtomicU64::new(0)),
            frame_w: Arc::new(AtomicU32::new(0)),
            frame_h: Arc::new(AtomicU32::new(0)),
            sensor_w: Arc::new(AtomicU32::new(0)),
            sensor_h: Arc::new(AtomicU32::new(0)),
            last_raw_len: Arc::new(AtomicUsize::new(0)),
            focus_enabled: Arc::new(AtomicBool::new(false)),
            focus_reset: Arc::new(AtomicBool::new(false)),
            last_frame_ms: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Reader task: record that a frame was just delivered (epoch millis). Lock-free.
    pub fn mark_frame(&self) {
        let ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        self.last_frame_ms.store(ms, Ordering::Relaxed);
    }

    /// Worker watchdog: epoch millis of the last delivered frame (`0` = none yet).
    pub fn last_frame_ms(&self) -> u64 {
        self.last_frame_ms.load(Ordering::Relaxed)
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

    /// GUI -> worker: set the max preview refresh rate in fps (`<= 0` = every frame).
    pub fn set_preview_fps(&self, fps: f32) {
        self.preview_fps.store(fps.to_bits(), Ordering::Relaxed);
    }

    /// Worker: max preview refresh rate in fps (`<= 0` = every frame).
    pub fn preview_fps(&self) -> f32 {
        f32::from_bits(self.preview_fps.load(Ordering::Relaxed))
    }

    /// GUI -> worker: turn per-frame focus measurement on/off.
    pub fn set_focus_enabled(&self, on: bool) {
        self.focus_enabled.store(on, Ordering::Relaxed);
    }

    /// Whether the decode thread should measure focus this frame.
    pub fn focus_enabled(&self) -> bool {
        self.focus_enabled.load(Ordering::Relaxed)
    }

    /// GUI -> worker: request the focus peak-hold / history be reset.
    pub fn request_focus_reset(&self) {
        self.focus_reset.store(true, Ordering::Relaxed);
    }

    /// Decode thread: consume a pending focus-reset request (returns true once per request).
    pub fn take_focus_reset(&self) -> bool {
        self.focus_reset.swap(false, Ordering::Relaxed)
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

    /// Worker: record the current readout geometry so the decode thread can interpret raw frames.
    pub fn set_stream_geometry(&self, w: u32, h: u32) {
        self.frame_w.store(w, Ordering::Relaxed);
        self.frame_h.store(h, Ordering::Relaxed);
    }

    /// Decode thread: the current readout geometry `(w, h)` (`(0, 0)` until known).
    pub fn frame_geometry(&self) -> (u32, u32) {
        (
            self.frame_w.load(Ordering::Relaxed),
            self.frame_h.load(Ordering::Relaxed),
        )
    }

    /// Worker: record the full sensor size (fallback geometry for drivers that stream full frames
    /// even when a subframe is requested).
    pub fn set_sensor_size(&self, w: u32, h: u32) {
        self.sensor_w.store(w, Ordering::Relaxed);
        self.sensor_h.store(h, Ordering::Relaxed);
    }

    /// Decode thread: the full sensor size `(w, h)` (`(0, 0)` until known).
    pub fn sensor_size(&self) -> (u32, u32) {
        (
            self.sensor_w.load(Ordering::Relaxed),
            self.sensor_h.load(Ordering::Relaxed),
        )
    }

    /// Decode thread: remember the newest raw frame's decompressed byte length.
    pub fn set_last_raw_len(&self, len: usize) {
        self.last_raw_len.store(len, Ordering::Relaxed);
    }

    /// The newest raw frame's decompressed byte length (for bit-depth inference in the UI).
    pub fn last_raw_len(&self) -> usize {
        self.last_raw_len.load(Ordering::Relaxed)
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
