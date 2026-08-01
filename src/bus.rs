//! Message types and shared state connecting the GUI thread and the async INDI worker.
//!
//! - GUI -> worker: [`Command`] over an unbounded mpsc channel.
//! - worker -> GUI (frames): an [`arc_swap::ArcSwapOption`] holding only the latest [`Frame`]
//!   (high-FPS streams drop stale frames for the live view).
//! - worker -> GUI (state): [`Shared`] behind a std `Mutex`, read each repaint.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use arc_swap::ArcSwapOption;

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
    pub latest_frame: Arc<ArcSwapOption<Frame>>,
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            shared: Arc::new(Mutex::new(Shared::default())),
            latest_frame: Arc::new(ArcSwapOption::empty()),
        }
    }

    /// Convenience: append a log line, ignoring lock poisoning.
    pub fn log(&self, msg: impl Into<String>) {
        if let Ok(mut s) = self.shared.lock() {
            s.log(msg);
        }
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}
