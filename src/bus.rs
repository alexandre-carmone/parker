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
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwapOption;
use egui::ColorImage;

use crate::frame::Frame;

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
