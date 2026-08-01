//! egui front-end: live view, camera controls, mount controls, and status/log.

use eframe::egui;
use tokio::sync::mpsc::UnboundedSender;

use crate::bus::{Bus, Command, ConnState, Dir};

const DIRS: [(Dir, &str); 4] = [
    (Dir::North, "N"),
    (Dir::South, "S"),
    (Dir::East, "E"),
    (Dir::West, "W"),
];

pub struct App {
    bus: Bus,
    tx: UnboundedSender<Command>,
    /// Kept alive so the worker's tokio runtime lives as long as the app.
    _rt: tokio::runtime::Runtime,

    addr: String,
    capture_dir: String,
    texture: Option<egui::TextureHandle>,
    last_seq: u64,
    /// Per-direction "currently held" state for press-and-hold nudging (N, S, E, W).
    nudge_down: [bool; 4],
    gain_input: f64,
    exposure_input: f64,

    /// Live-view display stretch (raw stream frames are very dark).
    auto_stretch: bool,
    display_gain: f32,
    /// Force a texture re-upload when stretch settings change (even without a new frame).
    stretch_dirty: bool,

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
        App {
            bus,
            tx,
            _rt: rt,
            addr: "127.0.0.1:7624".to_owned(),
            capture_dir: "captures".to_owned(),
            texture: None,
            last_seq: 0,
            nudge_down: [false; 4],
            gain_input: 90.0,
            exposure_input: 0.05,
            auto_stretch: true,
            display_gain: 1.0,
            stretch_dirty: false,
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

    /// Upload the newest decoded frame to the GPU texture (only when the sequence changed),
    /// applying the display stretch so faint frames are visible.
    fn refresh_texture(&mut self, ctx: &egui::Context) {
        if let Some(frame) = self.bus.latest_frame.load_full() {
            if frame.seq != self.last_seq || self.stretch_dirty {
                self.last_seq = frame.seq;
                self.stretch_dirty = false;
                let pixels = stretch(&frame.rgba, self.auto_stretch, self.display_gain);
                let color =
                    egui::ColorImage::from_rgba_unmultiplied([frame.width, frame.height], &pixels);
                match &mut self.texture {
                    Some(tex) => tex.set(color, egui::TextureOptions::LINEAR),
                    None => {
                        self.texture =
                            Some(ctx.load_texture("live", color, egui::TextureOptions::LINEAR))
                    }
                }
            }
        }
    }
}

/// Snapshot of the shared state needed for one frame of UI (avoids holding the lock).
struct Snap {
    conn: ConnState,
    streaming: bool,
    fps: f32,
    frame_count: u64,
    slew_rates: Vec<String>,
    slew_rate_idx: usize,
    tracking: bool,
    last_capture: Option<String>,
    log_tail: Vec<String>,
}

impl App {
    fn snapshot(&self) -> Snap {
        let sh = self.bus.shared.lock().unwrap();
        Snap {
            conn: sh.conn,
            streaming: sh.streaming,
            fps: sh.fps,
            frame_count: sh.frame_count,
            slew_rates: sh.slew_rates.clone(),
            slew_rate_idx: sh.slew_rate_idx,
            tracking: sh.tracking,
            last_capture: sh.last_capture.clone(),
            log_tail: sh.log.iter().rev().take(8).rev().cloned().collect(),
        }
    }
}

impl eframe::App for App {
    // egui 0.35: the App trait provides a root `Ui`; panels attach to it (not the Context).
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.refresh_texture(&ctx);
        let snap = self.snapshot();
        let connected = snap.conn == ConnState::Connected;

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
                ui.label("Gain");
                if ui
                    .add(egui::DragValue::new(&mut self.gain_input).speed(1.0).range(0.0..=1000.0))
                    .drag_stopped()
                {
                    self.send(Command::SetGain(self.gain_input));
                }
                ui.label("Exposure (s)");
                if ui
                    .add(
                        egui::DragValue::new(&mut self.exposure_input)
                            .speed(0.005)
                            .range(0.0001..=30.0),
                    )
                    .drag_stopped()
                {
                    self.send(Command::SetExposure(self.exposure_input));
                }
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
            });

            ui.separator();
            ui.label("Display");
            if ui.checkbox(&mut self.auto_stretch, "Auto-stretch").changed() {
                self.stretch_dirty = true;
            }
            if ui
                .add(egui::Slider::new(&mut self.display_gain, 0.1..=20.0).text("gain"))
                .changed()
            {
                self.stretch_dirty = true;
            }
        });

        // ---- Right: mount ----
        egui::Panel::right("mount").resizable(false).min_size(210.0).show(ui, |ui| {
            ui.heading("Mount");
            ui.add_enabled_ui(connected, |ui| {
                // Slew rate
                if !snap.slew_rates.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Slew rate:");
                        let current = snap
                            .slew_rates
                            .get(snap.slew_rate_idx)
                            .cloned()
                            .unwrap_or_default();
                        egui::ComboBox::from_id_salt("slew")
                            .selected_text(current)
                            .show_ui(ui, |ui| {
                                for (i, name) in snap.slew_rates.iter().enumerate() {
                                    if ui
                                        .selectable_label(i == snap.slew_rate_idx, name)
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

                ui.separator();
                ui.label("Manual slew (press & hold)");
                self.nudge_pad(ui);

                ui.separator();
                if ui.button("⛔ Abort motion").clicked() {
                    self.send(Command::Abort);
                }
            });
        });

        // ---- Bottom: log ----
        egui::Panel::bottom("log").show(ui, |ui| {
            ui.label("Log");
            for line in &snap.log_tail {
                ui.small(line);
            }
        });

        // ---- Center: live view ----
        egui::CentralPanel::default().show(ui, |ui| {
            if let Some(tex) = &self.texture {
                let avail = ui.available_size();
                let sized = egui::load::SizedTexture::new(tex.id(), tex.size_vec2());
                ui.centered_and_justified(|ui| {
                    ui.add(
                        egui::Image::new(sized)
                            .max_size(avail)
                            .maintain_aspect_ratio(true),
                    );
                });
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("No video — connect and start the stream.");
                });
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
    /// A 3×3 D-pad; each button starts a nudge on press and stops it on release.
    fn nudge_pad(&mut self, ui: &mut egui::Ui) {
        let btn = |ui: &mut egui::Ui, label: &str| {
            ui.add_sized([44.0, 32.0], egui::Button::new(label))
        };
        let mut handle = |ui: &mut egui::Ui, idx: usize, dir: Dir, label: &str| {
            let resp = btn(ui, label);
            let down = resp.is_pointer_button_down_on();
            if down != self.nudge_down[idx] {
                self.nudge_down[idx] = down;
                self.send(Command::Nudge { dir, active: down });
            }
        };

        ui.vertical_centered(|ui| {
            ui.horizontal(|ui| {
                ui.add_space(48.0);
                handle(ui, 0, Dir::North, DIRS[0].1);
            });
            ui.horizontal(|ui| {
                handle(ui, 3, Dir::West, DIRS[3].1);
                ui.add_space(48.0);
                handle(ui, 2, Dir::East, DIRS[2].1);
            });
            ui.horizontal(|ui| {
                ui.add_space(48.0);
                handle(ui, 1, Dir::South, DIRS[1].1);
            });
        });
    }
}

/// Apply a display stretch to an RGBA frame for the live view. With `auto`, the brightest
/// channel value is mapped to 255 (a simple linear autostretch); `gain` is an extra
/// multiplier. Alpha is preserved. Returns a new buffer.
fn stretch(rgba: &[u8], auto: bool, gain: f32) -> Vec<u8> {
    let mut scale = gain.max(0.0);
    if auto {
        let max = rgba
            .chunks_exact(4)
            .flat_map(|p| [p[0], p[1], p[2]])
            .max()
            .unwrap_or(255);
        if max > 0 {
            scale *= 255.0 / max as f32;
        }
    }
    // Fast path: no change needed.
    if (scale - 1.0).abs() < f32::EPSILON {
        return rgba.to_vec();
    }
    let mut out = Vec::with_capacity(rgba.len());
    for p in rgba.chunks_exact(4) {
        for &c in &p[..3] {
            out.push((c as f32 * scale).round().clamp(0.0, 255.0) as u8);
        }
        out.push(p[3]);
    }
    out
}
