//! egui front-end: live view, camera controls, mount controls, and status/log.

use std::sync::atomic::Ordering;

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
    /// Last display image seq uploaded to the texture (the streaming fast path).
    last_display_seq: u64,
    /// Per-direction "currently held" state for press-and-hold nudging (N, S, E, W).
    nudge_down: [bool; 4],
    gain_input: f64,
    exposure_input: f64,

    /// Live-view display stretch (raw stream frames are very dark).
    auto_stretch: bool,
    display_gain: f32,
    /// Force a texture re-upload when stretch settings change (even without a new frame).
    stretch_dirty: bool,

    /// Pending ROI numeric inputs (sensor pixels): x, y, width, height. Seeded to the full
    /// sensor once its size is known, and overwritten when the user drags a rectangle.
    roi_x: i64,
    roi_y: i64,
    roi_w: i64,
    roi_h: i64,
    /// Sensor size the `roi_*` inputs were last seeded from, so a camera swap (new geometry)
    /// reseeds them to the new full frame. `(0, 0)` until first seeded.
    roi_seeded_for: (u32, u32),
    /// In-progress ROI drag on the live view: (start, current) in screen coordinates.
    roi_drag: Option<(egui::Pos2, egui::Pos2)>,

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
        App {
            bus,
            tx,
            _rt: rt,
            addr: "127.0.0.1:7624".to_owned(),
            capture_dir: "captures".to_owned(),
            texture: None,
            last_display_seq: 0,
            nudge_down: [false; 4],
            gain_input: 90.0,
            exposure_input: 0.05,
            auto_stretch: true,
            display_gain: 1.0,
            stretch_dirty: false,
            roi_x: 0,
            roi_y: 0,
            roi_w: 0,
            roi_h: 0,
            roi_seeded_for: (0, 0),
            roi_drag: None,
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
    slew_rates: Vec<String>,
    slew_rate_idx: usize,
    tracking: bool,
    last_capture: Option<String>,
    sensor_w: u32,
    sensor_h: u32,
    roi: (u32, u32, u32, u32),
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
            cameras: sh.cameras.clone(),
            mounts: sh.mounts.clone(),
            camera_sel: sh.camera_sel.clone(),
            mount_sel: sh.mount_sel.clone(),
            slew_rates: sh.slew_rates.clone(),
            slew_rate_idx: sh.slew_rate_idx,
            tracking: sh.tracking,
            last_capture: sh.last_capture.clone(),
            sensor_w: sh.sensor_w,
            sensor_h: sh.sensor_h,
            roi: sh.roi,
            log_tail: sh.log.iter().rev().take(8).rev().cloned().collect(),
        }
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
