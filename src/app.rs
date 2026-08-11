//! egui front-end: live view, camera controls, mount controls, and status/log.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::Ordering;
use std::sync::Arc;

use eframe::egui;
use tokio::sync::mpsc::UnboundedSender;

use crate::bus::{
    Bus, CameraSwitch, Command, ConnState, Dir, IndiPanel, IndiState, IndiSwitchRule, IndiValue,
    RecordConfig, RecordPhase, RecordStatus, RecordStop,
};
use crate::ephemeris::SolarObject;
use crate::guiding::{Calibration, DecMode, GuideMode, GuideParams, DEFAULT_CALIB_MS};

/// The 8 directional buttons of the nudge pad, in row-major 3×3 order (the center cell is a Stop
/// button, handled separately). Each button fires one `Nudge` per listed cardinal direction, so a
/// corner drives both the NS and WE motion axes at once for a true diagonal slew.
const NUDGE_BUTTONS: [(&str, &[Dir]); 8] = [
    ("↖", &[Dir::North, Dir::West]),
    ("N", &[Dir::North]),
    ("↗", &[Dir::North, Dir::East]),
    ("W", &[Dir::West]),
    ("E", &[Dir::East]),
    ("↙", &[Dir::South, Dir::West]),
    ("S", &[Dir::South]),
    ("↘", &[Dir::South, Dir::East]),
];

pub struct App {
    bus: Bus,
    tx: UnboundedSender<Command>,
    /// Kept alive so the worker's tokio runtime lives as long as the app.
    _rt: tokio::runtime::Runtime,

    addr: String,
    capture_dir: String,
    texture: Option<egui::TextureHandle>,
    /// Last display image seq uploaded to the texture (the streaming fast path).
    last_display_seq: u64,
    /// Per-button "currently held" state for press-and-hold nudging, indexed into
    /// [`NUDGE_BUTTONS`] (8 directions: 4 cardinal + 4 diagonal).
    nudge_down: [bool; 8],
    gain_input: f64,
    exposure_input: f64,

    /// Live-view display stretch (raw stream frames are very dark).
    auto_stretch: bool,
    display_gain: f32,
    /// Max preview refresh rate (fps); caps the stretch + upload to spare CPU.
    preview_fps: f32,
    /// Force a texture re-upload when stretch settings change (even without a new frame).
    stretch_dirty: bool,
    /// Cached luminance histogram (256 bins) of the latest frame, the frame seq it was computed
    /// from, and that frame's full-scale ADU (255 for 8-bit, 65535 for 16-bit) for the x-axis.
    /// Recomputed lazily only while the Histogram section is expanded (see `histogram_ui`), so a
    /// collapsed panel costs nothing.
    hist_bins: [u32; 256],
    hist_seq: u64,
    hist_max_adu: u32,

    /// Focus-measurement UI state: whether the decode thread is measuring sharpness. Mirrors
    /// [`Bus::focus_enabled`]; toggled from the Focus section.
    focus_enabled: bool,

    /// Pending ROI numeric inputs (sensor pixels): x, y, width, height. Seeded to the full
    /// sensor once its size is known, and overwritten when the user drags a rectangle.
    roi_x: i64,
    roi_y: i64,
    roi_w: i64,
    roi_h: i64,
    /// Sensor size the `roi_*` inputs were last seeded from, so a camera swap (new geometry)
    /// reseeds them to the new full frame. `(0, 0)` until first seeded.
    roi_seeded_for: (u32, u32),
    /// Pending symmetric binning selection (1 = unbinned). Seeded from the driver alongside the
    /// ROI inputs; applied immediately on change.
    bin_input: u32,
    /// In-progress ROI drag on the live view: (start, current) in screen coordinates.
    roi_drag: Option<(egui::Pos2, egui::Pos2)>,

    // ---- guiding (M2) UI state ----
    /// Editable copies backing the guiding controls; changes are pushed to the worker.
    guide_mode: GuideMode,
    guide_params: GuideParams,
    /// Show the detected-target / lock-point overlay on the live view.
    detect_overlay: bool,
    /// Calibration pulse duration (ms) for the next `Calibrate` run.
    calib_pulse_ms: f32,
    /// Image scale (arcsec/pixel), user-entered. When > 0, guide errors are also shown in
    /// arcseconds. Display-only — never sent to the worker.
    pixel_scale: f32,

    // ---- recording (M3) UI state ----
    /// Stop each video by frame count (true) or by elapsed seconds (false).
    record_by_frames: bool,
    record_target_frames: u64,
    record_target_secs: f64,
    /// Number of videos in the sequence and the delay (s) between them.
    record_count: usize,
    record_delay_secs: f64,

    // ---- generic INDI control panel edit buffers ----
    /// Pending numeric edits, keyed `"{device}/{prop}/{elem}"`, lazily seeded from the driver
    /// value and pushed only when the user clicks the property's Set button.
    indi_num_edits: HashMap<String, f64>,
    /// Pending text edits, keyed `"{device}/{prop}/{elem}"`.
    indi_txt_edits: HashMap<String, String>,

    // ---- test/automation hooks (driven by env vars) ----
    autoconnect: bool,
    autostream: bool,
    did_autoconnect: bool,
    did_autostream: bool,
    /// If set, save a GUI screenshot to this path once frames are flowing, then close.
    screenshot_path: Option<String>,
    screenshot_requested: bool,
}

impl App {
    pub fn new(bus: Bus, tx: UnboundedSender<Command>, rt: tokio::runtime::Runtime) -> Self {
        let autoconnect = std::env::var("SOLAR_AUTOCONNECT").is_ok();
        let screenshot_path = std::env::var("SOLAR_SCREENSHOT").ok();
        // Seed the worker's stretch settings from our defaults below.
        bus.set_display_settings(true, 1.0);
        bus.set_preview_fps(1.0);
        App {
            bus,
            tx,
            _rt: rt,
            addr: "127.0.0.1:7624".to_owned(),
            capture_dir: "captures".to_owned(),
            texture: None,
            last_display_seq: 0,
            nudge_down: [false; 8],
            gain_input: 215.0,
            exposure_input: 0.002,
            auto_stretch: true,
            display_gain: 1.0,
            preview_fps: 1.0,
            hist_bins: [0; 256],
            hist_seq: 0,
            hist_max_adu: 255,
            focus_enabled: false,
            stretch_dirty: false,
            roi_x: 0,
            roi_y: 0,
            roi_w: 0,
            roi_h: 0,
            roi_seeded_for: (0, 0),
            bin_input: 1,
            roi_drag: None,
            guide_mode: GuideMode::Disk,
            guide_params: GuideParams::default(),
            detect_overlay: false,
            calib_pulse_ms: DEFAULT_CALIB_MS,
            pixel_scale: 0.0,
            record_by_frames: true,
            record_target_frames: 500,
            record_target_secs: 30.0,
            record_count: 1,
            record_delay_secs: 0.0,
            indi_num_edits: HashMap::new(),
            indi_txt_edits: HashMap::new(),
            // A screenshot run implies autoconnect + autostream so there is content to show.
            autoconnect: autoconnect || screenshot_path.is_some(),
            autostream: autoconnect || screenshot_path.is_some(),
            did_autoconnect: false,
            did_autostream: false,
            screenshot_path,
            screenshot_requested: false,
        }
    }

    fn send(&self, cmd: Command) {
        let _ = self.tx.send(cmd);
    }

    /// Refresh the GPU texture from the newest frame.
    ///
    /// Fast path (streaming): the worker has already done the stretch + `Color32` conversion off
    /// this thread, so we just upload its `Arc<ColorImage>` — no per-frame CPU work here, which is
    /// what keeps the UI responsive at high FPS. Slow path (`stretch_dirty` while paused): the user
    /// moved a display control with no new frames arriving, so re-render the last raw frame once on
    /// this thread — it is user-paced, never the streaming hot loop.
    fn refresh_texture(&mut self, ctx: &egui::Context, streaming: bool) {
        let dseq = self.bus.display_seq.load(Ordering::Acquire);
        if dseq != self.last_display_seq {
            if let Some(img) = self.bus.display.load_full() {
                self.last_display_seq = dseq;
                self.stretch_dirty = false;
                self.upload(ctx, img);
                return;
            }
        }
        // Nothing new from the worker; only re-render on this thread if a paused adjustment needs it.
        if self.stretch_dirty && !streaming {
            if let Some(frame) = self.bus.latest_frame.load_full() {
                self.stretch_dirty = false;
                let img = std::sync::Arc::new(frame.to_display_image(self.auto_stretch, self.display_gain));
                self.upload(ctx, img);
            }
        }
    }

    /// Upload a display image to the live texture (creating it on first use). Passing the
    /// `Arc<ColorImage>` in is zero-copy: egui wraps the same allocation.
    fn upload(&mut self, ctx: &egui::Context, img: std::sync::Arc<egui::ColorImage>) {
        match &mut self.texture {
            Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
            None => self.texture = Some(ctx.load_texture("live", img, egui::TextureOptions::LINEAR)),
        }
    }
}

/// Snapshot of the shared state needed for one frame of UI (avoids holding the lock).
struct Snap {
    conn: ConnState,
    streaming: bool,
    fps: f32,
    frame_count: u64,
    cameras: Vec<String>,
    mounts: Vec<String>,
    camera_sel: String,
    mount_sel: String,
    slew_rates: Vec<(String, String)>,
    slew_rate_idx: usize,
    track_modes: Vec<(String, String)>,
    track_mode_idx: usize,
    tracking: bool,
    last_capture: Option<String>,
    sensor_w: u32,
    sensor_h: u32,
    roi: (u32, u32, u32, u32),
    binning: u32,
    // recording (M3)
    stream_switches: Vec<CameraSwitch>,
    /// Effective streamed bit depth inferred from the raw payload (`None` for MJPEG/unknown).
    stream_depth: Option<u8>,
    recording: RecordStatus,
    // guiding (M2)
    guiding: bool,
    calibrating: bool,
    calibrated: bool,
    guide_calib: Option<Calibration>,
    lock_point: Option<(f32, f32)>,
    detected: Option<(f32, f32)>,
    guide_err: Option<(f32, f32)>,
    guide_rms: f32,
    guide_rms_ra: f32,
    guide_rms_dec: f32,
    guide_peak: f32,
    guide_history: VecDeque<(f32, f32)>,
    // focus measurement
    focus_metric: f32,
    focus_peak: f32,
    focus_history: VecDeque<f32>,
    log_tail: Vec<String>,
    // generic INDI control panel
    camera_panel: Option<Arc<IndiPanel>>,
    mount_panel: Option<Arc<IndiPanel>>,
}

impl App {
    fn snapshot(&self) -> Snap {
        let sh = self.bus.shared.lock().unwrap();
        Snap {
            conn: sh.conn,
            streaming: sh.streaming,
            fps: sh.fps,
            frame_count: sh.frame_count,
            cameras: sh.cameras.clone(),
            mounts: sh.mounts.clone(),
            camera_sel: sh.camera_sel.clone(),
            mount_sel: sh.mount_sel.clone(),
            slew_rates: sh.slew_rates.clone(),
            slew_rate_idx: sh.slew_rate_idx,
            track_modes: sh.track_modes.clone(),
            track_mode_idx: sh.track_mode_idx,
            tracking: sh.tracking,
            last_capture: sh.last_capture.clone(),
            sensor_w: sh.sensor_w,
            sensor_h: sh.sensor_h,
            roi: sh.roi,
            binning: sh.binning,
            stream_switches: sh.stream_switches.clone(),
            stream_depth: stream_depth(&self.bus),
            recording: sh.recording.clone(),
            guiding: sh.guiding,
            calibrating: sh.calibrating,
            calibrated: sh.calibrated,
            guide_calib: sh.guide_calib,
            lock_point: sh.lock_point,
            detected: sh.detected,
            guide_err: sh.guide_err,
            guide_rms: sh.guide_rms,
            guide_rms_ra: sh.guide_rms_ra,
            guide_rms_dec: sh.guide_rms_dec,
            guide_peak: sh.guide_peak,
            guide_history: sh.guide_history.clone(),
            focus_metric: sh.focus_metric,
            focus_peak: sh.focus_peak,
            focus_history: sh.focus_history.clone(),
            log_tail: sh.log.iter().rev().take(8).rev().cloned().collect(),
            camera_panel: sh.camera_panel.clone(),
            mount_panel: sh.mount_panel.clone(),
        }
    }
}

/// Color + hover text for an INDI property/light state LED.
fn led_color(state: IndiState) -> (egui::Color32, &'static str) {
    match state {
        IndiState::Idle => (egui::Color32::GRAY, "idle"),
        IndiState::Ok => (egui::Color32::from_rgb(0x3c, 0xb3, 0x71), "ok"),
        IndiState::Busy => (egui::Color32::from_rgb(0xd4, 0xa0, 0x17), "busy"),
        IndiState::Alert => (egui::Color32::from_rgb(0xc0, 0x39, 0x2b), "alert"),
    }
}

/// Format an INDI number value for read-only display: trims trailing zeros, keeps integers clean.
fn format_num(v: f64) -> String {
    if v == v.trunc() && v.abs() < 1e15 {
        format!("{v:.0}")
    } else {
        let s = format!("{v:.6}");
        let s = s.trim_end_matches('0').trim_end_matches('.');
        s.to_string()
    }
}

/// Infer the effective streamed bit depth from the latest raw payload size vs. frame geometry.
/// `None` when unknown — no raw frame yet, or an MJPEG stream (where `last_raw_len` stays 0).
fn stream_depth(bus: &Bus) -> Option<u8> {
    let (w, h) = bus.frame_geometry();
    let len = bus.last_raw_len();
    let px = (w as usize).checked_mul(h as usize).unwrap_or(0);
    if px == 0 || len == 0 {
        return None;
    }
    match len / px {
        2.. => Some(16),
        1 => Some(8),
        _ => None,
    }
}

impl eframe::App for App {
    // egui 0.35: the App trait provides a root `Ui`; panels attach to it (not the Context).
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let snap = self.snapshot();
        self.refresh_texture(&ctx, snap.streaming);
        let connected = snap.conn == ConnState::Connected;

        // Seed the ROI inputs to the full sensor the first time its size is reported, and again
        // if the sensor geometry changes (e.g. the user selects a different camera).
        if snap.sensor_w > 0 && (snap.sensor_w, snap.sensor_h) != self.roi_seeded_for {
            self.roi_seeded_for = (snap.sensor_w, snap.sensor_h);
            let (x, y, w, h) = snap.roi;
            self.roi_x = x as i64;
            self.roi_y = y as i64;
            self.roi_w = w as i64;
            self.roi_h = h as i64;
            self.bin_input = snap.binning.max(1);
        }

        // ---- Top bar: connection + status ----
        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("INDI server:");
                ui.add_enabled(
                    !connected,
                    egui::TextEdit::singleline(&mut self.addr).desired_width(160.0),
                );
                if ui
                    .add_enabled(!connected, egui::Button::new("Connect"))
                    .clicked()
                {
                    self.send(Command::Connect {
                        addr: self.addr.clone(),
                    });
                }
                if ui
                    .add_enabled(connected, egui::Button::new("Disconnect"))
                    .clicked()
                {
                    self.send(Command::Disconnect);
                }
                ui.separator();
                let (txt, color) = match snap.conn {
                    ConnState::Disconnected => ("disconnected", egui::Color32::GRAY),
                    ConnState::Connecting => ("connecting…", egui::Color32::YELLOW),
                    ConnState::Connected => ("connected", egui::Color32::GREEN),
                    ConnState::Failed => ("failed", egui::Color32::RED),
                };
                ui.colored_label(color, txt);
                ui.separator();
                ui.label(format!("{:.1} fps", snap.fps));
                ui.label(format!("frames: {}", snap.frame_count));
            });
        });

        // ---- Left: camera ----
        egui::Panel::left("camera").resizable(false).min_size(210.0).show(ui, |ui| {
            ui.heading("Camera");
            ui.add_enabled_ui(connected, |ui| {
                if !snap.cameras.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Device:");
                        egui::ComboBox::from_id_salt("camera_dev")
                            .selected_text(snap.camera_sel.clone())
                            .show_ui(ui, |ui| {
                                for name in &snap.cameras {
                                    if ui
                                        .selectable_label(*name == snap.camera_sel, name)
                                        .clicked()
                                        && *name != snap.camera_sel
                                    {
                                        self.send(Command::SelectCamera(name.clone()));
                                    }
                                }
                            });
                    });
                    ui.separator();
                }
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!snap.streaming, egui::Button::new("▶ Start stream"))
                        .clicked()
                    {
                        self.send(Command::StartStream);
                    }
                    if ui
                        .add_enabled(snap.streaming, egui::Button::new("⏹ Stop"))
                        .clicked()
                    {
                        self.send(Command::StopStream);
                    }
                });
                ui.separator();
                self.format_controls(ui, &snap);
                ui.label("Gain");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.gain_input).speed(1.0).range(0.0..=1000.0));
                    if ui.button("Apply").clicked() {
                        self.send(Command::SetGain(self.gain_input));
                    }
                });
                ui.label("Exposure (s)");
                ui.horizontal(|ui| {
                    ui.add(
                        egui::DragValue::new(&mut self.exposure_input)
                            .speed(0.005)
                            .range(0.0001..=30.0),
                    );
                    if ui.button("Apply").clicked() {
                        self.send(Command::SetExposure(self.exposure_input));
                    }
                });
                ui.separator();
                ui.label("Capture folder");
                ui.text_edit_singleline(&mut self.capture_dir);
                if ui.button("📷 Capture frame").clicked() {
                    self.send(Command::CaptureFrame {
                        dir: self.capture_dir.clone(),
                    });
                }
                if let Some(path) = &snap.last_capture {
                    ui.small(format!("saved: {path}"));
                }

                self.recording_controls(ui, &snap);

                ui.separator();
                ui.label("Region of interest");
                ui.add_enabled_ui(snap.sensor_w > 0, |ui| {
                    let sw = snap.sensor_w.max(1) as f64;
                    let sh = snap.sensor_h.max(1) as f64;
                    egui::Grid::new("roi_grid").num_columns(2).show(ui, |ui| {
                        ui.label("X");
                        ui.add(egui::DragValue::new(&mut self.roi_x).range(0.0..=sw - 1.0));
                        ui.end_row();
                        ui.label("Y");
                        ui.add(egui::DragValue::new(&mut self.roi_y).range(0.0..=sh - 1.0));
                        ui.end_row();
                        ui.label("W");
                        ui.add(egui::DragValue::new(&mut self.roi_w).range(1.0..=sw));
                        ui.end_row();
                        ui.label("H");
                        ui.add(egui::DragValue::new(&mut self.roi_h).range(1.0..=sh));
                        ui.end_row();
                    });
                    ui.small("Tip: drag a rectangle on the video to set this.");
                    ui.horizontal(|ui| {
                        if ui.button("Apply ROI").clicked() {
                            let (x, y, w, h) = clamp_roi(
                                self.roi_x,
                                self.roi_y,
                                self.roi_w,
                                self.roi_h,
                                snap.sensor_w,
                                snap.sensor_h,
                            );
                            self.roi_x = x as i64;
                            self.roi_y = y as i64;
                            self.roi_w = w as i64;
                            self.roi_h = h as i64;
                            self.send(Command::SetRoi { x, y, w, h });
                        }
                        if ui.button("Reset (full)").clicked() {
                            self.roi_x = 0;
                            self.roi_y = 0;
                            self.roi_w = snap.sensor_w as i64;
                            self.roi_h = snap.sensor_h as i64;
                            self.send(Command::ResetRoi);
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Binning");
                        egui::ComboBox::from_id_salt("binning")
                            .selected_text(format!("{0}×{0}", self.bin_input))
                            .show_ui(ui, |ui| {
                                for bin in 1..=4u32 {
                                    if ui
                                        .selectable_label(bin == self.bin_input, format!("{bin}×{bin}"))
                                        .clicked()
                                        && bin != self.bin_input
                                    {
                                        self.bin_input = bin;
                                        self.send(Command::SetBinning { bin });
                                    }
                                }
                            });
                    });
                });
            });

            ui.separator();
            ui.label("Display");
            let mut settings_changed = false;
            if ui.checkbox(&mut self.auto_stretch, "Auto-stretch").changed() {
                settings_changed = true;
            }
            if ui
                .add(egui::Slider::new(&mut self.display_gain, 0.1..=20.0).text("gain"))
                .changed()
            {
                settings_changed = true;
            }
            if settings_changed {
                // Push to the worker (applied to new frames), and mark dirty so a paused still
                // frame is re-rendered on the next repaint.
                self.bus.set_display_settings(self.auto_stretch, self.display_gain);
                self.stretch_dirty = true;
            }
            if ui
                .add(
                    egui::Slider::new(&mut self.preview_fps, 0.5..=30.0)
                        .text("preview fps")
                        .logarithmic(true),
                )
                .on_hover_text(
                    "Max preview refresh rate. Lower = less CPU. Recording and guiding are \
                     unaffected — every frame is still decoded and written.",
                )
                .changed()
            {
                self.bus.set_preview_fps(self.preview_fps);
            }

            ui.collapsing("Histogram", |ui| {
                self.histogram_ui(ui);
            });

            ui.collapsing("Focus", |ui| {
                self.focus_ui(ui, &snap);
            });

            ui.separator();
            let camera_panel = snap.camera_panel.clone();
            ui.collapsing("INDI Control Panel", |ui| {
                self.indi_panel_ui(ui, camera_panel.as_deref());
            });
        });

        // ---- Right: mount ----
        egui::Panel::right("mount").resizable(false).min_size(210.0).show(ui, |ui| {
            ui.heading("Mount");
            ui.add_enabled_ui(connected, |ui| {
                if !snap.mounts.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Device:");
                        egui::ComboBox::from_id_salt("mount_dev")
                            .selected_text(snap.mount_sel.clone())
                            .show_ui(ui, |ui| {
                                for name in &snap.mounts {
                                    if ui
                                        .selectable_label(*name == snap.mount_sel, name)
                                        .clicked()
                                        && *name != snap.mount_sel
                                    {
                                        self.send(Command::SelectMount(name.clone()));
                                    }
                                }
                            });
                    });
                    ui.separator();
                }
                // Slew rate
                if !snap.slew_rates.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Slew rate:");
                        let current = snap
                            .slew_rates
                            .get(snap.slew_rate_idx)
                            .map(|(_, label)| label.clone())
                            .unwrap_or_default();
                        egui::ComboBox::from_id_salt("slew")
                            .selected_text(current)
                            .show_ui(ui, |ui| {
                                for (i, (_, label)) in snap.slew_rates.iter().enumerate() {
                                    if ui
                                        .selectable_label(i == snap.slew_rate_idx, label)
                                        .clicked()
                                    {
                                        self.send(Command::SetSlewRate(i));
                                    }
                                }
                            });
                    });
                }

                let mut tracking = snap.tracking;
                if ui.checkbox(&mut tracking, "Tracking").clicked() {
                    self.send(Command::SetTracking(tracking));
                }
                // Tracking mode (sidereal/solar/lunar/…), if the mount exposes it.
                if !snap.track_modes.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Track mode:");
                        let current = snap
                            .track_modes
                            .get(snap.track_mode_idx)
                            .map(|(_, label)| label.clone())
                            .unwrap_or_default();
                        ui.add_enabled_ui(snap.tracking, |ui| {
                            egui::ComboBox::from_id_salt("track_mode")
                                .selected_text(current)
                                .show_ui(ui, |ui| {
                                    for (i, (_, label)) in snap.track_modes.iter().enumerate() {
                                        if ui
                                            .selectable_label(i == snap.track_mode_idx, label)
                                            .clicked()
                                        {
                                            self.send(Command::SetTrackMode(i));
                                        }
                                    }
                                });
                        });
                    });
                }

                ui.separator();
                ui.label("Go To");
                ui.horizontal_wrapped(|ui| {
                    for &obj in SolarObject::all() {
                        if ui.button(obj.label()).clicked() {
                            self.send(Command::GotoObject(obj));
                        }
                    }
                });

                ui.separator();
                ui.label("Manual slew (press & hold)");
                self.nudge_pad(ui);

                ui.separator();
                if ui.button("⛔ Abort motion").clicked() {
                    self.send(Command::Abort);
                }

                ui.separator();
                self.guiding_controls(ui, &snap);

                ui.separator();
                let mount_panel = snap.mount_panel.clone();
                ui.collapsing("INDI Control Panel", |ui| {
                    self.indi_panel_ui(ui, mount_panel.as_deref());
                });
            });
        });

        // ---- Bottom: log ----
        egui::Panel::bottom("log").show(ui, |ui| {
            ui.label("Log");
            for line in &snap.log_tail {
                ui.small(line);
            }
        });

        // ---- Bottom: guide-error graph (above the log) ----
        egui::Panel::bottom("guide_graph")
            .resizable(true)
            .show(ui, |ui| {
                self.guide_graph(ui, &snap);
            });

        // ---- Center: live view (drag to select an ROI) ----
        egui::CentralPanel::default().show(ui, |ui| {
            let Some((tex_id, tex_size)) = self.texture.as_ref().map(|t| (t.id(), t.size_vec2()))
            else {
                ui.centered_and_justified(|ui| {
                    ui.label("No video — connect and start the stream.");
                });
                return;
            };

            // Aspect-fit the frame into the available area (what maintain_aspect_ratio did,
            // but explicit so pointer↔pixel mapping is exact).
            let image_rect = fit_rect(ui.available_rect_before_wrap(), tex_size);
            let sized = egui::load::SizedTexture::new(tex_id, tex_size);
            egui::Image::new(sized).paint_at(ui, image_rect);

            // ROI selection is only meaningful once we know the sensor geometry.
            if snap.sensor_w > 0 {
                let resp = ui.interact(image_rect, ui.id().with("roi_sel"), egui::Sense::drag());
                if resp.drag_started() {
                    if let Some(p) = resp.interact_pointer_pos() {
                        self.roi_drag = Some((p, p));
                    }
                } else if resp.dragged() {
                    if let (Some((start, _)), Some(p)) =
                        (self.roi_drag, resp.interact_pointer_pos())
                    {
                        self.roi_drag = Some((start, p));
                    }
                } else if resp.drag_stopped() {
                    if let Some((start, end)) = self.roi_drag.take() {
                        let sel = egui::Rect::from_two_pos(start, end).intersect(image_rect);
                        // Ignore an accidental click (near-zero drag).
                        if sel.width() >= 3.0 && sel.height() >= 3.0 {
                            let img = [
                                image_rect.min.x,
                                image_rect.min.y,
                                image_rect.width(),
                                image_rect.height(),
                            ];
                            let drag = [sel.min.x, sel.min.y, sel.width(), sel.height()];
                            let (x, y, w, h) = drag_to_roi(img, drag, snap.roi);
                            self.roi_x = x as i64;
                            self.roi_y = y as i64;
                            self.roi_w = w as i64;
                            self.roi_h = h as i64;
                        }
                    }
                }
                // Draw the in-progress selection rectangle.
                if let Some((start, end)) = self.roi_drag {
                    let rect = egui::Rect::from_two_pos(start, end).intersect(image_rect);
                    ui.painter().rect_stroke(
                        rect,
                        0.0,
                        egui::Stroke::new(1.5, egui::Color32::YELLOW),
                        egui::StrokeKind::Inside,
                    );
                }
            }

            // Guiding overlay: map frame-pixel positions onto the displayed image. The texture
            // size equals the decoded frame size, so a pixel maps by its fraction of that size.
            let to_screen = |p: (f32, f32)| -> egui::Pos2 {
                image_rect.min
                    + egui::vec2(p.0 / tex_size.x, p.1 / tex_size.y) * image_rect.size()
            };
            let painter = ui.painter_at(image_rect);
            if let Some(lock) = snap.lock_point {
                let c = to_screen(lock);
                let stroke = egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 210, 0));
                painter.line_segment([c - egui::vec2(9.0, 0.0), c + egui::vec2(9.0, 0.0)], stroke);
                painter.line_segment([c - egui::vec2(0.0, 9.0), c + egui::vec2(0.0, 9.0)], stroke);
            }
            if let Some(det) = snap.detected {
                let c = to_screen(det);
                let col = if snap.guiding {
                    egui::Color32::from_rgb(80, 220, 120)
                } else {
                    egui::Color32::from_rgb(120, 180, 255)
                };
                painter.circle_stroke(c, 7.0, egui::Stroke::new(1.5, col));
            }
        });

        // ---- automation hooks (env-driven; used for headless verification) ----
        if self.autoconnect && !self.did_autoconnect {
            self.did_autoconnect = true;
            self.send(Command::Connect {
                addr: self.addr.clone(),
            });
        }
        if self.autostream && !self.did_autostream && connected {
            self.did_autostream = true;
            self.send(Command::StartStream);
        }
        if let Some(path) = self.screenshot_path.clone() {
            if !self.screenshot_requested && snap.frame_count >= 5 {
                self.screenshot_requested = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(
                    egui::UserData::default(),
                ));
            }
            let shot = ctx.input(|i| {
                i.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(image) = shot {
                save_screenshot(&image, &path);
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        }

        // Keep pulling frames / polling held buttons while streaming, nudging, or automating.
        if snap.streaming
            || self.nudge_down.iter().any(|d| *d)
            || (self.screenshot_path.is_some() && !self.screenshot_requested)
        {
            ctx.request_repaint();
        }
    }
}

/// Smallest ROI we will command, in sensor pixels (drivers reject degenerate frames).
const MIN_ROI: i64 = 16;

/// Centered aspect-fit of a texture of `tex_size` within `container` (replaces egui's
/// `maintain_aspect_ratio`, giving us the exact on-screen image rectangle for hit-testing).
fn fit_rect(container: egui::Rect, tex_size: egui::Vec2) -> egui::Rect {
    if tex_size.x <= 0.0 || tex_size.y <= 0.0 {
        return container;
    }
    let scale = (container.width() / tex_size.x)
        .min(container.height() / tex_size.y)
        .max(0.0);
    egui::Rect::from_center_size(container.center(), tex_size * scale)
}

/// Clamp user-entered ROI numbers to the sensor and enforce a minimum size, in sensor pixels.
fn clamp_roi(x: i64, y: i64, w: i64, h: i64, sensor_w: u32, sensor_h: u32) -> (u32, u32, u32, u32) {
    // Guard against an unknown sensor size (0): the clamps below would form `clamp(1, 0)` and
    // panic. Callers gate this on `sensor_w > 0` today, but don't rely on that here.
    if sensor_w == 0 || sensor_h == 0 {
        return (0, 0, 0, 0);
    }
    let sw = sensor_w as i64;
    let sh = sensor_h as i64;
    let x = x.clamp(0, (sw - 1).max(0));
    let y = y.clamp(0, (sh - 1).max(0));
    // Fit within the sensor, but grow up to MIN_ROI where there is room.
    let w = w.clamp(1, sw - x).max(MIN_ROI.min(sw - x));
    let h = h.clamp(1, sh - y).max(MIN_ROI.min(sh - y));
    (x as u32, y as u32, w as u32, h as u32)
}

/// Map a drag rectangle drawn on the displayed image to a sensor-pixel ROI.
///
/// The displayed frame *is* the currently-applied ROI, so we compose: the drag's fractional
/// position/size within the on-screen image rect is scaled by the current ROI and offset by its
/// origin. `image` and `drag` are `[min_x, min_y, width, height]` in the same screen coordinates;
/// `drag` is assumed already clipped to `image`.
fn drag_to_roi(
    image: [f32; 4],
    drag: [f32; 4],
    current: (u32, u32, u32, u32),
) -> (u32, u32, u32, u32) {
    let (cx, cy, cw, ch) = current;
    if image[2] <= 0.0 || image[3] <= 0.0 || cw == 0 || ch == 0 {
        return current;
    }
    let fx = ((drag[0] - image[0]) / image[2]).clamp(0.0, 1.0);
    let fy = ((drag[1] - image[1]) / image[3]).clamp(0.0, 1.0);
    let fw = (drag[2] / image[2]).clamp(0.0, 1.0);
    let fh = (drag[3] / image[3]).clamp(0.0, 1.0);
    let x = (cx + (fx * cw as f32).round() as u32).min(cx + cw - 1);
    let y = (cy + (fy * ch as f32).round() as u32).min(cy + ch - 1);
    let w = ((fw * cw as f32).round() as u32).clamp(1, cx + cw - x);
    let h = ((fh * ch as f32).round() as u32).clamp(1, cy + ch - y);
    (x, y, w, h)
}

/// Encode an egui screenshot [`ColorImage`] to a PNG file (used by the screenshot hook).
fn save_screenshot(img: &egui::ColorImage, path: &str) {
    let [w, h] = img.size;
    let mut buf = Vec::with_capacity(w * h * 4);
    for px in &img.pixels {
        buf.extend_from_slice(&px.to_srgba_unmultiplied());
    }
    match image::RgbaImage::from_raw(w as u32, h as u32, buf) {
        Some(rgba) => {
            let _ = rgba.save(path);
            eprintln!("[app] saved screenshot to {path}");
        }
        None => eprintln!("[app] screenshot buffer size mismatch"),
    }
}

impl App {
    /// A 3×3 D-pad. Each of the 8 outer buttons starts a nudge on press and stops it on release;
    /// corners fire both cardinal axes for a diagonal slew. The center cell is a Stop button.
    fn nudge_pad(&mut self, ui: &mut egui::Ui) {
        // Capture the fields the handler needs separately so it borrows disjoint parts of `self` —
        // this lets the center Stop button also touch `self.tx` between handler calls.
        let tx = &self.tx;
        let nudge_down = &mut self.nudge_down;
        // Directional buttons sense *dragging*, not clicking. A plain click-sense button drops its
        // held state once the press outlasts egui's `max_click_duration` (~0.8 s) — which reads as
        // a release and stops the slew while the button is still physically held (this was the
        // "slew stops after a few seconds" bug). Drag-sense keeps `is_pointer_button_down_on` true
        // for as long as the button is down, so the mount slews until the user actually releases.
        let dir_btn = |ui: &mut egui::Ui, label: &str| {
            ui.add_sized(
                [44.0, 32.0],
                egui::Button::new(label).sense(egui::Sense::click_and_drag()),
            )
        };
        let mut handle = |ui: &mut egui::Ui, idx: usize| {
            let (label, dirs) = NUDGE_BUTTONS[idx];
            let down = dir_btn(ui, label).is_pointer_button_down_on();
            if down != nudge_down[idx] {
                nudge_down[idx] = down;
                for &dir in dirs {
                    let _ = tx.send(Command::Nudge { dir, active: down });
                }
            }
        };

        ui.vertical_centered(|ui| {
            ui.horizontal(|ui| {
                handle(ui, 0);
                handle(ui, 1);
                handle(ui, 2);
            });
            ui.horizontal(|ui| {
                handle(ui, 3);
                if ui
                    .add_sized([44.0, 32.0], egui::Button::new("⏹"))
                    .clicked()
                {
                    let _ = tx.send(Command::Abort);
                }
                handle(ui, 4);
            });
            ui.horizontal(|ui| {
                handle(ui, 5);
                handle(ui, 6);
                handle(ui, 7);
            });
        });
    }

    /// Stream-format pickers, one per switch property the camera driver exposes (encoder, video
    /// format, bit depth, sensor mode). These govern the streamed bit depth the SER recorder
    /// captures — for true 16-bit you typically need the encoder on RAW *and* a 16-bit video
    /// format/depth. The effective depth is shown live so you can confirm before recording.
    /// Renders nothing when the driver exposes no such switches.
    fn format_controls(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        if snap.stream_switches.is_empty() {
            return;
        }
        egui::Grid::new("format_grid").num_columns(2).show(ui, |ui| {
            for sw in &snap.stream_switches {
                ui.label(&sw.label);
                egui::ComboBox::from_id_salt(&sw.prop)
                    .selected_text(&sw.selected)
                    .show_ui(ui, |ui| {
                        for name in &sw.options {
                            if ui.selectable_label(*name == sw.selected, name).clicked()
                                && *name != sw.selected
                            {
                                self.send(Command::SetCameraSwitch {
                                    prop: sw.prop.clone(),
                                    elem: name.clone(),
                                });
                            }
                        }
                    });
                ui.end_row();
            }
        });
        if let Some(depth) = snap.stream_depth {
            let color = if depth >= 16 {
                egui::Color32::GREEN
            } else {
                egui::Color32::GRAY
            };
            ui.colored_label(color, format!("stream depth: {depth}-bit"));
        }
        ui.separator();
    }

    /// Generic INDI control panel: render a device's full property tree, grouped by INDI group.
    /// Each group is a collapsing section; each property a row with a state LED, label, and a
    /// type-specific editor (numbers/switches/text editable if writable; lights read-only).
    fn indi_panel_ui(&mut self, ui: &mut egui::Ui, panel: Option<&IndiPanel>) {
        let Some(panel) = panel else {
            ui.label("(not connected)");
            return;
        };
        if panel.groups.is_empty() {
            ui.label("(no properties)");
            return;
        }
        egui::ScrollArea::vertical()
            .auto_shrink([false, true])
            .max_height(420.0)
            .show(ui, |ui| {
                for group in &panel.groups {
                    egui::CollapsingHeader::new(&group.name)
                        .id_salt(("indi", &panel.device, &group.name))
                        .show(ui, |ui| {
                            for prop in &group.props {
                                self.indi_prop_row(ui, &panel.device, prop);
                            }
                        });
                }
            });
    }

    /// One INDI property row: state LED + label, then the typed editor(s) indented beneath.
    fn indi_prop_row(&mut self, ui: &mut egui::Ui, device: &str, prop: &crate::bus::IndiProp) {
        ui.horizontal(|ui| {
            let (color, tip) = led_color(prop.state);
            ui.colored_label(color, "⏺").on_hover_text(tip);
            ui.strong(&prop.label);
            if !prop.writable {
                ui.weak("(ro)");
            }
        });
        ui.indent(("indi_prop", device, &prop.name), |ui| match &prop.value {
            IndiValue::Number(items) => {
                egui::Grid::new(("indi_num", device, &prop.name))
                    .num_columns(3)
                    .show(ui, |ui| {
                        for n in items {
                            ui.label(&n.label);
                            ui.monospace(format_num(n.value));
                            if prop.writable {
                                let key = format!("{device}/{}/{}", prop.name, n.name);
                                let buf = self.indi_num_edits.entry(key).or_insert(n.value);
                                let mut dv = egui::DragValue::new(buf);
                                if n.min < n.max {
                                    dv = dv.range(n.min..=n.max);
                                }
                                if n.step > 0.0 {
                                    dv = dv.speed(n.step);
                                }
                                ui.add(dv);
                            }
                            ui.end_row();
                        }
                    });
                if prop.writable && ui.button("Set").clicked() {
                    let elems = items
                        .iter()
                        .map(|n| {
                            let key = format!("{device}/{}/{}", prop.name, n.name);
                            let v = self.indi_num_edits.get(&key).copied().unwrap_or(n.value);
                            (n.name.clone(), v)
                        })
                        .collect();
                    self.send(Command::SetIndiNumber {
                        device: device.to_string(),
                        prop: prop.name.clone(),
                        elems,
                    });
                }
            }
            IndiValue::Switch { rule, items } => match rule {
                IndiSwitchRule::AnyOfMany => {
                    for sw in items {
                        let mut on = sw.on;
                        if ui
                            .add_enabled(prop.writable, egui::Checkbox::new(&mut on, &sw.label))
                            .clicked()
                            && prop.writable
                        {
                            self.send(Command::SetIndiSwitch {
                                device: device.to_string(),
                                prop: prop.name.clone(),
                                elems: vec![(sw.name.clone(), on)],
                            });
                        }
                    }
                }
                // OneOfMany / AtMostOne: radio-style — pick one, the driver clears siblings.
                _ => {
                    for sw in items {
                        if ui
                            .add_enabled(
                                prop.writable,
                                egui::Button::selectable(sw.on, &sw.label),
                            )
                            .clicked()
                            && prop.writable
                            && !sw.on
                        {
                            self.send(Command::SetIndiSwitch {
                                device: device.to_string(),
                                prop: prop.name.clone(),
                                elems: vec![(sw.name.clone(), true)],
                            });
                        }
                    }
                }
            },
            IndiValue::Text(items) => {
                egui::Grid::new(("indi_txt", device, &prop.name))
                    .num_columns(2)
                    .show(ui, |ui| {
                        for t in items {
                            ui.label(&t.label);
                            if prop.writable {
                                let key = format!("{device}/{}/{}", prop.name, t.name);
                                let buf = self
                                    .indi_txt_edits
                                    .entry(key)
                                    .or_insert_with(|| t.value.clone());
                                ui.text_edit_singleline(buf);
                            } else {
                                ui.monospace(&t.value);
                            }
                            ui.end_row();
                        }
                    });
                if prop.writable && ui.button("Set").clicked() {
                    let elems = items
                        .iter()
                        .map(|t| {
                            let key = format!("{device}/{}/{}", prop.name, t.name);
                            let v = self
                                .indi_txt_edits
                                .get(&key)
                                .cloned()
                                .unwrap_or_else(|| t.value.clone());
                            (t.name.clone(), v)
                        })
                        .collect();
                    self.send(Command::SetIndiText {
                        device: device.to_string(),
                        prop: prop.name.clone(),
                        elems,
                    });
                }
            }
            IndiValue::Light(items) => {
                for l in items {
                    ui.horizontal(|ui| {
                        let (color, tip) = led_color(l.state);
                        ui.colored_label(color, "⏺").on_hover_text(tip);
                        ui.label(&l.label);
                    });
                }
            }
            IndiValue::Blob(labels) => {
                ui.weak(format!("blob: {}", labels.join(", ")));
            }
        });
    }

    /// Recording / sequence controls: record the live stream to SER — a single video or a timed
    /// sequence of them (stop each by frame count or duration, with a delay between). Recording
    /// runs concurrently with guiding.
    fn recording_controls(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        let rec = &snap.recording;
        ui.collapsing("Recording / Sequence", |ui| {
            ui.horizontal(|ui| {
                ui.label("Stop by:");
                ui.selectable_value(&mut self.record_by_frames, true, "frames");
                ui.selectable_value(&mut self.record_by_frames, false, "seconds");
            });
            if self.record_by_frames {
                ui.add(
                    egui::DragValue::new(&mut self.record_target_frames)
                        .speed(10.0)
                        .range(1.0..=1_000_000.0)
                        .prefix("target: ")
                        .suffix(" frames"),
                );
            } else {
                ui.add(
                    egui::DragValue::new(&mut self.record_target_secs)
                        .speed(0.5)
                        .range(0.1..=3600.0)
                        .prefix("target: ")
                        .suffix(" s"),
                );
            }
            egui::Grid::new("record_seq_grid").num_columns(2).show(ui, |ui| {
                ui.label("Videos");
                ui.add(egui::DragValue::new(&mut self.record_count).range(1.0..=1000.0));
                ui.end_row();
                ui.label("Delay (s)");
                ui.add(
                    egui::DragValue::new(&mut self.record_delay_secs)
                        .speed(0.5)
                        .range(0.0..=3600.0),
                );
                ui.end_row();
            });

            ui.horizontal(|ui| {
                // Driver-side recording is independent of the client stream — the driver captures
                // and writes on its own — so this needs a connected camera, not an active stream.
                let can_record = snap.conn == ConnState::Connected
                    && !snap.camera_sel.is_empty()
                    && !rec.active;
                if ui
                    .add_enabled(can_record, egui::Button::new("⏺ Record"))
                    .clicked()
                {
                    let stop = if self.record_by_frames {
                        RecordStop::Frames(self.record_target_frames)
                    } else {
                        RecordStop::Seconds(self.record_target_secs)
                    };
                    self.send(Command::StartRecording(RecordConfig {
                        dir: self.capture_dir.clone(),
                        count: self.record_count.max(1),
                        stop,
                        delay_secs: self.record_delay_secs,
                    }));
                }
                if ui
                    .add_enabled(rec.active, egui::Button::new("⏹ Stop"))
                    .clicked()
                {
                    self.send(Command::StopRecording);
                }
            });

            if rec.active {
                ui.horizontal(|ui| {
                    ui.spinner();
                    if rec.phase == RecordPhase::Waiting {
                        ui.label(format!("waiting… (video {}/{})", rec.current, rec.total));
                    } else {
                        // The driver writes the file server-side and doesn't report a live frame
                        // count, so progress is video index + elapsed time.
                        ui.label(format!(
                            "video {}/{} · {:.1} s",
                            rec.current, rec.total, rec.elapsed_secs
                        ));
                    }
                });
            }
            if let Some(path) = &rec.last_file {
                ui.small(format!("saved: {path}"));
            }
            ui.small("Recorded by the INDI driver on the server host, under the capture dir.");
        });
    }

    /// Guiding section: detection mode, calibration, start/stop, per-axis loop parameters.
    fn guiding_controls(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        ui.collapsing("Guiding", |ui| {
            self.detection_controls(ui);
            ui.separator();
            self.calibration_controls(ui, snap);
            ui.separator();
            self.guide_run_controls(ui, snap);
            ui.separator();
            self.guide_param_controls(ui);
        });
    }

    /// Detection mode + on-screen overlay toggle.
    fn detection_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Mode:");
            let mut mode = self.guide_mode;
            egui::ComboBox::from_id_salt("guide_mode")
                .selected_text(match mode {
                    GuideMode::Disk => "Disk (centroid)",
                    GuideMode::Surface => "Surface (xcorr)",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut mode, GuideMode::Disk, "Disk (centroid)");
                    ui.selectable_value(&mut mode, GuideMode::Surface, "Surface (xcorr)");
                });
            if mode != self.guide_mode {
                self.guide_mode = mode;
                self.send(Command::SetGuideMode(mode));
            }
        });

        if ui
            .checkbox(&mut self.detect_overlay, "Show detection")
            .changed()
        {
            self.send(Command::SetDetectionOverlay(self.detect_overlay));
        }
    }

    /// Calibration: adjustable pulse, run/clear, status, and the resulting axis geometry.
    fn calibration_controls(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        let busy = snap.calibrating || snap.guiding;
        ui.add(
            egui::Slider::new(&mut self.calib_pulse_ms, 250.0..=4000.0)
                .text("calib pulse (ms)")
                .step_by(50.0),
        );
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Calibrate"))
                .clicked()
            {
                self.send(Command::Calibrate {
                    pulse_ms: self.calib_pulse_ms,
                });
            }
            if ui
                .add_enabled(snap.calibrated && !busy, egui::Button::new("Clear"))
                .clicked()
            {
                self.send(Command::ClearCalibration);
            }
            if snap.calibrating {
                ui.spinner();
                ui.label("calibrating…");
            } else if snap.calibrated {
                ui.colored_label(egui::Color32::GREEN, "calibrated ✓");
            } else {
                ui.colored_label(egui::Color32::GRAY, "not calibrated");
            }
        });

        // Axis geometry readout: orientation, guide rate (px/s), and squareness.
        if let Some(cal) = snap.guide_calib {
            let (ra_deg, dec_deg) = cal.axis_angles_deg();
            let (ra_scale, dec_scale) = cal.axis_scales();
            let ortho = cal.orthogonality_deg();
            ui.label(format!(
                "RA {:.1}° · {:.2} px/s    DEC {:.1}° · {:.2} px/s",
                ra_deg,
                ra_scale * 1000.0,
                dec_deg,
                dec_scale * 1000.0,
            ));
            // Flag a badly skewed calibration (well away from 90°).
            let ortho_col = if (ortho - 90.0).abs() > 20.0 {
                egui::Color32::from_rgb(230, 160, 60)
            } else {
                ui.visuals().weak_text_color()
            };
            ui.colored_label(ortho_col, format!("axis angle {ortho:.1}° (90° = square)"));
        }
    }

    /// Start / stop / re-lock, plus the DEC-mode selector.
    fn guide_run_controls(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    snap.calibrated && !snap.guiding,
                    egui::Button::new("▶ Guide"),
                )
                .clicked()
            {
                self.send(Command::StartGuiding);
            }
            if ui
                .add_enabled(snap.guiding, egui::Button::new("⏹ Stop"))
                .clicked()
            {
                self.send(Command::StopGuiding);
            }
            if ui
                .add_enabled(snap.guiding, egui::Button::new("Re-lock"))
                .clicked()
            {
                self.send(Command::Relock);
            }
        });

        ui.horizontal(|ui| {
            ui.label("DEC:");
            let mut mode = self.guide_params.dec_mode;
            egui::ComboBox::from_id_salt("dec_mode")
                .selected_text(match mode {
                    DecMode::Auto => "Auto (N+S)",
                    DecMode::NorthOnly => "North only",
                    DecMode::SouthOnly => "South only",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut mode, DecMode::Auto, "Auto (N+S)");
                    ui.selectable_value(&mut mode, DecMode::NorthOnly, "North only");
                    ui.selectable_value(&mut mode, DecMode::SouthOnly, "South only");
                });
            if mode != self.guide_params.dec_mode {
                self.guide_params.dec_mode = mode;
                self.send(Command::SetGuideParams(self.guide_params));
            }
        })
        .response
        .on_hover_text("Restrict DEC corrections to one direction to avoid backlash reversals.");
    }

    /// Per-axis loop parameters (RA row, DEC row), shared cadence, and the image scale.
    fn guide_param_controls(&mut self, ui: &mut egui::Ui) {
        let mut p = self.guide_params;
        let mut changed = false;

        egui::Grid::new("guide_params")
            .num_columns(4)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("");
                ui.label("aggr");
                ui.label("max ms");
                ui.label("min px");
                ui.end_row();

                ui.label("RA");
                changed |= ui
                    .add(egui::DragValue::new(&mut p.ra_aggr).speed(0.01).range(0.05..=1.0))
                    .changed();
                changed |= ui
                    .add(egui::DragValue::new(&mut p.ra_max_pulse_ms).speed(5.0).range(50.0..=2000.0))
                    .changed();
                changed |= ui
                    .add(egui::DragValue::new(&mut p.ra_min_move_px).speed(0.05).range(0.0..=5.0))
                    .changed();
                ui.end_row();

                ui.label("DEC");
                changed |= ui
                    .add(egui::DragValue::new(&mut p.dec_aggr).speed(0.01).range(0.05..=1.0))
                    .changed();
                changed |= ui
                    .add(egui::DragValue::new(&mut p.dec_max_pulse_ms).speed(5.0).range(50.0..=2000.0))
                    .changed();
                changed |= ui
                    .add(egui::DragValue::new(&mut p.dec_min_move_px).speed(0.05).range(0.0..=5.0))
                    .changed();
                ui.end_row();
            });

        let mut cadence = p.cadence_ms as f64;
        if ui
            .add(egui::Slider::new(&mut cadence, 100.0..=2000.0).text("cadence (ms)"))
            .changed()
        {
            p.cadence_ms = cadence as u64;
            changed = true;
        }
        if changed {
            self.guide_params = p;
            self.send(Command::SetGuideParams(p));
        }

        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(&mut self.pixel_scale)
                    .speed(0.01)
                    .range(0.0..=60.0)
                    .suffix(" ″/px"),
            )
            .on_hover_text("Image scale (arcsec/pixel). Set it to read guide errors in arcseconds.");
            ui.label("image scale");
        });
    }

    /// The guide-error graph (RA/DEC over time) plus live error / split-RMS readout.
    /// Collapsible luminance histogram of the live preview. This runs on the GUI thread, but only
    /// while the section is expanded (the `collapsing` closure isn't called when closed), and the
    /// 256-bin scan is recomputed only when a new frame arrives — so an open panel adds at most one
    /// pass per displayed frame, and a closed one costs nothing.
    fn histogram_ui(&mut self, ui: &mut egui::Ui) {
        use egui_plot::{Bar, BarChart, Plot};

        if let Some(frame) = self.bus.latest_frame.load_full() {
            if frame.seq != self.hist_seq {
                self.hist_seq = frame.seq;
                self.hist_bins = frame.luma_histogram();
                self.hist_max_adu = frame.max_adu;
            }
        }
        if self.hist_seq == 0 {
            ui.weak("No frames yet.");
            return;
        }

        // The 256 bins map onto the source's ADU range: each bin spans `(max_adu + 1) / 256` ADU,
        // so for an 8-bit stream a bin is 1 ADU wide (x: 0..255) and for a 16-bit stream it is
        // 256 ADU wide (x: 0..65535, at top-8-bit resolution — the preview only carries the high
        // byte). Placing each bar at its bin's ADU centre labels the x-axis in true ADU.
        let step = (self.hist_max_adu as f64 + 1.0) / 256.0;
        ui.weak(format!("ADU 0–{} ({}-bit)", self.hist_max_adu, if self.hist_max_adu > 255 { 16 } else { 8 }));

        // Normalize to the tallest bin so the shape is readable regardless of frame size. A
        // log-ish feel isn't needed here; a linear bar chart matches how imagers read exposure.
        let peak = self.hist_bins.iter().copied().max().unwrap_or(1).max(1) as f64;
        let bars: Vec<Bar> = self
            .hist_bins
            .iter()
            .enumerate()
            .map(|(i, &c)| Bar::new(i as f64 * step + step / 2.0, c as f64 / peak).width(step))
            .collect();
        let chart = BarChart::new("ADU", bars).color(egui::Color32::from_gray(180));
        Plot::new("hist_plot")
            .height(90.0)
            .show_axes([true, false])
            .show_y(false)
            .allow_zoom(false)
            .allow_drag(false)
            .allow_scroll(false)
            .allow_boxed_zoom(false)
            .include_x(0.0)
            .include_x(self.hist_max_adu as f64)
            .include_y(0.0)
            .include_y(1.05)
            .show(ui, |plot_ui| {
                plot_ui.bar_chart(chart);
            });
    }

    /// Focus-measurement section: a smoothed sharpness readout, peak-hold, and a rolling curve to
    /// chase best focus by hand. Sharpness is measured over the current frame — set a hardware ROI
    /// over a sunspot/limb to focus on that region.
    fn focus_ui(&mut self, ui: &mut egui::Ui, snap: &Snap) {
        use egui_plot::{HLine, Line, Plot, PlotPoints};

        ui.horizontal(|ui| {
            if ui
                .checkbox(&mut self.focus_enabled, "Measure focus")
                .on_hover_text(
                    "Compute a live sharpness metric over the current frame/ROI. Higher = sharper. \
                     Turn your focuser to maximize it.",
                )
                .changed()
            {
                self.bus.set_focus_enabled(self.focus_enabled);
            }
            if ui
                .add_enabled(self.focus_enabled, egui::Button::new("Reset peak"))
                .clicked()
            {
                self.bus.request_focus_reset();
            }
        });

        if !self.focus_enabled {
            ui.weak("Enable to measure sharpness.");
            return;
        }

        // Trend arrow: compare the mean of the recent half of the window to the earlier half, so a
        // small seeing wobble doesn't flip it every frame.
        let hist = &snap.focus_history;
        let trend = {
            let n = hist.len();
            if n >= 6 {
                let tail = n.min(20);
                let seg: Vec<f32> = hist.iter().rev().take(tail).copied().collect();
                let half = seg.len() / 2;
                let recent: f32 = seg[..half].iter().sum::<f32>() / half as f32;
                let older: f32 = seg[half..].iter().sum::<f32>() / (seg.len() - half) as f32;
                let eps = older.abs() * 0.01;
                if recent > older + eps {
                    ("▲ rising", egui::Color32::from_rgb(0x3c, 0xb3, 0x71))
                } else if recent < older - eps {
                    ("▼ falling", egui::Color32::from_rgb(0xc0, 0x39, 0x2b))
                } else {
                    ("– steady", egui::Color32::GRAY)
                }
            } else {
                ("…", egui::Color32::GRAY)
            }
        };

        ui.horizontal(|ui| {
            ui.strong(format!("Focus: {:.1}", snap.focus_metric));
            ui.colored_label(trend.1, trend.0);
            ui.separator();
            ui.label(format!("peak {:.1}", snap.focus_peak));
            if snap.focus_peak > 0.0 {
                ui.weak(format!("{:.0}% of peak", 100.0 * snap.focus_metric / snap.focus_peak));
            }
        });

        let line: PlotPoints = hist
            .iter()
            .enumerate()
            .map(|(i, v)| [i as f64, *v as f64])
            .collect();
        let n = hist.len();
        let window = 300usize;
        let x_min = n.saturating_sub(window) as f64;
        let x_max = n.max(window) as f64;
        let peak = snap.focus_peak as f64;
        Plot::new("focus_plot")
            .height(90.0)
            .show_axes([false, true])
            .allow_zoom(false)
            .allow_drag(false)
            .allow_scroll(false)
            .allow_boxed_zoom(false)
            .include_x(x_min)
            .include_x(x_max)
            .include_y(0.0)
            .show(ui, |plot_ui| {
                // Faint peak line so the user sees how close the current reading is to best focus.
                if peak > 0.0 {
                    plot_ui.hline(
                        HLine::new("", peak)
                            .color(egui::Color32::from_gray(90))
                            .width(0.5),
                    );
                }
                plot_ui.line(Line::new("focus", line));
            });
    }

    fn guide_graph(&self, ui: &mut egui::Ui, snap: &Snap) {
        use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};
        // Optional arcsec conversion for the numeric readouts.
        let scale = self.pixel_scale;
        let px = |v: f32| -> String {
            if scale > 0.0 {
                format!("{v:.2} px ({:.2}″)", v * scale)
            } else {
                format!("{v:.2} px")
            }
        };

        ui.horizontal(|ui| {
            ui.strong("Guide error");
            if let Some((ra, dec)) = snap.guide_err {
                ui.label(format!("RA {ra:+.2}  DEC {dec:+.2} px"));
            }
            ui.separator();
            ui.label(format!("RMS {}", px(snap.guide_rms)));
            ui.weak(format!(
                "(RA {:.2}  DEC {:.2})",
                snap.guide_rms_ra, snap.guide_rms_dec
            ));
            ui.separator();
            ui.label(format!("peak {}", px(snap.guide_peak)));
        });

        let ra: PlotPoints = snap
            .guide_history
            .iter()
            .enumerate()
            .map(|(i, (x, _))| [i as f64, *x as f64])
            .collect();
        let dec: PlotPoints = snap
            .guide_history
            .iter()
            .enumerate()
            .map(|(i, (_, y))| [i as f64, *y as f64])
            .collect();
        // Pin the x-axis to a rolling window so the trace scrolls rather than compressing.
        let n = snap.guide_history.len();
        let window = 300usize;
        let x_min = n.saturating_sub(window) as f64;
        let x_max = (n.max(window)) as f64;
        let rms = snap.guide_rms as f64;
        Plot::new("guide_plot")
            .height(110.0)
            .legend(Legend::default())
            .show_axes([false, true])
            .include_x(x_min)
            .include_x(x_max)
            .show(ui, |plot_ui| {
                // Faint ±total-RMS band for context.
                if rms > 0.0 {
                    let band = egui::Color32::from_gray(90);
                    plot_ui.hline(HLine::new("", rms).color(band).width(0.5));
                    plot_ui.hline(HLine::new("", -rms).color(band).width(0.5));
                }
                plot_ui.line(Line::new("RA", ra));
                plot_ui.line(Line::new("DEC", dec));
            });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drag_maps_fraction_of_full_frame() {
        // Full sensor 100×100 shown at 2× (200×200 on screen). A drag over the middle quarter
        // → x=25,y=25,w=50,h=50 in sensor pixels.
        let roi = drag_to_roi([0.0, 0.0, 200.0, 200.0], [50.0, 50.0, 100.0, 100.0], (0, 0, 100, 100));
        assert_eq!(roi, (25, 25, 50, 50));
    }

    #[test]
    fn drag_composes_with_current_roi_origin() {
        // Already cropped to a region starting at (10,20), size 80×60, shown 1:1. A drag over the
        // top-left half is offset by the current origin.
        let roi = drag_to_roi([0.0, 0.0, 80.0, 60.0], [0.0, 0.0, 40.0, 30.0], (10, 20, 80, 60));
        assert_eq!(roi, (10, 20, 40, 30));
    }

    #[test]
    fn drag_stays_inside_current_frame() {
        // A drag to the far edge can't produce a region extending past the current ROI.
        let roi = drag_to_roi([0.0, 0.0, 100.0, 100.0], [90.0, 90.0, 20.0, 20.0], (0, 0, 100, 100));
        assert_eq!(roi, (90, 90, 10, 10));
    }

    #[test]
    fn clamp_enforces_minimum_size() {
        // A 3px request grows to MIN_ROI when there's room.
        assert_eq!(clamp_roi(0, 0, 3, 3, 100, 100), (0, 0, 16, 16));
    }

    #[test]
    fn clamp_fits_within_sensor_at_edge() {
        // Near the far edge, width/height are capped to what remains (below MIN_ROI is allowed).
        assert_eq!(clamp_roi(90, 95, 50, 50, 100, 100), (90, 95, 10, 5));
    }
}
