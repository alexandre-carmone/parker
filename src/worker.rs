//! The async INDI worker: owns the connection/session, decodes the video stream into the
//! shared frame slot, and translates GUI [`Command`]s into INDI property changes.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};
use tokio::sync::mpsc::UnboundedReceiver;
use tokio::task::JoinHandle;
use tokio_stream::StreamExt;

use indi::client::active_device::ActiveDevice;
use indi::Parameter; // external crate; `crate::indi` is our local module

use crate::bus::{Bus, CameraSwitch, Command, ConnState, RecordConfig, RecordPhase, RecordStop};
use crate::frame::Frame;
use crate::guiding::{self, GuideDetector, GuideSample};
use crate::indi::camera::Camera;
use crate::indi::mount::Mount;
use crate::indi::Session;
use crate::recorder::{inflate_zlib, SerColor, SerRecorder};

/// The running guide control loop: its task plus a stop flag it polls each cycle. Dropping the
/// handle alone would leave the task running, so callers must call [`GuideLoop::shutdown`].
struct GuideLoop {
    task: JoinHandle<()>,
    stop: Arc<AtomicBool>,
}

impl GuideLoop {
    fn shutdown(self) {
        self.stop.store(true, Ordering::Relaxed);
        self.task.abort();
    }
}

/// The running recording orchestrator: its task plus a stop flag it polls between videos and
/// within each video's frame/time budget.
struct RecordTask {
    task: JoinHandle<()>,
    stop: Arc<AtomicBool>,
}

/// How often the decode thread runs detection while enabled — enough to feed the few-Hz control
/// loop and the overlay without doing centroid/NCC work on every high-FPS frame.
const DETECT_INTERVAL: Duration = Duration::from_millis(100);

/// While recording, throttle the live display to this interval. The stretch + Color32 conversion
/// and GUI upload are the costly per-frame work; skipping most of them during a recording keeps
/// CPU for writing the SER file while still showing a ~few-Hz preview. Frames are still decoded
/// and written every time — only the on-screen refresh is rate-limited.
const DISPLAY_INTERVAL_RECORDING: Duration = Duration::from_millis(200);

/// Entry point for the worker task. Runs until the command channel closes.
pub async fn run(mut rx: UnboundedReceiver<Command>, bus: Bus, ctx: egui::Context) {
    let mut session: Option<Session> = None;
    let mut frame_task: Option<FrameStream> = None;
    let mut guide_loop: Option<GuideLoop> = None;
    let mut calib_task: Option<JoinHandle<()>> = None;
    let mut record_task: Option<RecordTask> = None;

    while let Some(cmd) = rx.recv().await {
        match cmd {
            Command::Connect { addr } => {
                if let Some(fs) = frame_task.take() {
                    fs.shutdown();
                }
                session = None;
                set_conn(&bus, ConnState::Connecting);
                bus.log(format!("connecting to {addr}…"));

                match crate::indi::connect(&addr).await {
                    Ok(s) => {
                        let cam_desc = if s.camera_name.is_empty() {
                            "(none — no CCD device found)".to_string()
                        } else {
                            s.camera_name.clone()
                        };
                        let mount_desc = if s.mount_name.is_empty() {
                            "(none — no telescope device found)".to_string()
                        } else {
                            s.mount_name.clone()
                        };
                        bus.log(format!("camera: {cam_desc} · mount: {mount_desc}"));

                        let cameras = s.cameras().await;
                        let mounts = s.mounts().await;
                        if let Some(m) = &s.mount {
                            match m.slew_rates().await {
                                Ok(rates) => {
                                    if let Ok(mut sh) = bus.shared.lock() {
                                        sh.slew_rates = rates;
                                    }
                                }
                                Err(e) => bus.log(format!("reading slew rates: {e}")),
                            }
                        }
                        if let Ok(mut sh) = bus.shared.lock() {
                            sh.cameras = cameras;
                            sh.mounts = mounts;
                            sh.camera_sel = s.camera_name.clone();
                            sh.mount_sel = s.mount_name.clone();
                        }
                        init_sensor_size(&s, &bus).await;
                        init_camera_formats(&s, &bus).await;
                        if let Some(dev) = s.frame_device() {
                            frame_task = spawn_frame_task(dev, bus.clone(), ctx.clone()).await;
                        }
                        session = Some(s);
                        set_conn(&bus, ConnState::Connected);
                        bus.log("connected");
                    }
                    Err(e) => {
                        set_conn(&bus, ConnState::Failed);
                        bus.log(format!("connect failed: {e}"));
                    }
                }
                ctx.request_repaint();
            }
            Command::Disconnect => {
                stop_guiding(&mut guide_loop, &mut calib_task, &bus);
                stop_recording(&mut record_task, &bus);
                if let Some(fs) = frame_task.take() {
                    fs.shutdown();
                }
                session = None;
                if let Ok(mut sh) = bus.shared.lock() {
                    sh.streaming = false;
                    sh.fps = 0.0;
                }
                set_conn(&bus, ConnState::Disconnected);
                bus.log("disconnected");
                ctx.request_repaint();
            }
            Command::SelectCamera(name) => {
                stop_guiding(&mut guide_loop, &mut calib_task, &bus);
                stop_recording(&mut record_task, &bus);
                match session.as_mut() {
                    Some(s) => match s.select_camera(&name).await {
                        Ok(()) => {
                            if let Some(fs) = frame_task.take() {
                                fs.shutdown();
                            }
                            if let Ok(mut sh) = bus.shared.lock() {
                                sh.camera_sel = name.clone();
                                sh.streaming = false;
                                sh.fps = 0.0;
                            }
                            init_sensor_size(s, &bus).await;
                            init_camera_formats(s, &bus).await;
                            if let Some(dev) = s.frame_device() {
                                frame_task =
                                    spawn_frame_task(dev, bus.clone(), ctx.clone()).await;
                            }
                            bus.log(format!("camera: {name}"));
                        }
                        Err(e) => bus.log(format!("select camera failed: {e}")),
                    },
                    None => bus.log("not connected"),
                }
                ctx.request_repaint();
            }
            Command::SelectMount(name) => {
                stop_guiding(&mut guide_loop, &mut calib_task, &bus);
                match session.as_mut() {
                    Some(s) => match s.select_mount(&name).await {
                        Ok(()) => {
                            if let Some(m) = &s.mount {
                                match m.slew_rates().await {
                                    Ok(rates) => {
                                        if let Ok(mut sh) = bus.shared.lock() {
                                            sh.slew_rates = rates;
                                            sh.slew_rate_idx = 0;
                                        }
                                    }
                                    Err(e) => bus.log(format!("reading slew rates: {e}")),
                                }
                            }
                            if let Ok(mut sh) = bus.shared.lock() {
                                sh.mount_sel = name.clone();
                                sh.tracking = false;
                            }
                            bus.log(format!("mount: {name}"));
                        }
                        Err(e) => bus.log(format!("select mount failed: {e}")),
                    },
                    None => bus.log("not connected"),
                }
                ctx.request_repaint();
            }
            Command::Calibrate => {
                if calib_task.is_some() {
                    bus.log("calibration already running");
                } else if guide_loop.is_some() {
                    bus.log("stop guiding before calibrating");
                } else if let Some(s) = &session {
                    let streaming = bus.shared.lock().map(|sh| sh.streaming).unwrap_or(false);
                    match (streaming, s.clone_mount().await) {
                        (false, _) => bus.log("start the stream before calibrating"),
                        (true, None) => bus.log("no mount selected — cannot calibrate"),
                        (true, Some(m)) => {
                            bus.bump_ref_generation(); // fresh Surface reference at frame center
                            let (b, c) = (bus.clone(), ctx.clone());
                            calib_task = Some(tokio::spawn(async move {
                                guiding::run_calibration(m, b, c).await;
                            }));
                        }
                    }
                } else {
                    bus.log("not connected");
                }
                ctx.request_repaint();
            }
            Command::StartGuiding => {
                if guide_loop.is_some() {
                    bus.log("already guiding");
                } else if let Some(s) = &session {
                    start_guiding(s, &bus, &ctx, &mut guide_loop).await;
                } else {
                    bus.log("not connected");
                }
                ctx.request_repaint();
            }
            Command::StopGuiding => {
                stop_guiding(&mut guide_loop, &mut calib_task, &bus);
                bus.log("guiding stopped");
                ctx.request_repaint();
            }
            Command::StartRecording(cfg) => {
                // Reap a previous sequence that finished on its own (finite frame/time budget),
                // otherwise its stale handle would look like a still-running recording.
                if record_task.as_ref().is_some_and(|t| t.task.is_finished()) {
                    record_task = None;
                }
                if record_task.is_some() {
                    bus.log("already recording");
                } else if let Some(s) = &session {
                    start_recording(s, &bus, &ctx, cfg, &mut record_task).await;
                } else {
                    bus.log("not connected");
                }
                ctx.request_repaint();
            }
            Command::StopRecording => {
                stop_recording(&mut record_task, &bus);
                bus.log("recording stopped");
                ctx.request_repaint();
            }
            other => {
                // Safety: stopping the stream or changing the ROI invalidates the lock point /
                // reference patch, so stop guiding before applying such a command. The same
                // changes invalidate an in-progress recording's geometry, so stop it too.
                if matches!(
                    other,
                    Command::StopStream | Command::SetRoi { .. } | Command::ResetRoi
                ) {
                    if guide_loop.is_some() {
                        stop_guiding(&mut guide_loop, &mut calib_task, &bus);
                        bus.log("guiding stopped (frame changed)");
                    }
                    if record_task.is_some() {
                        stop_recording(&mut record_task, &bus);
                        bus.log("recording stopped (frame changed)");
                    }
                }
                match &session {
                    Some(s) => {
                        if let Err(e) = dispatch(other, s, &bus).await {
                            bus.log(format!("error: {e}"));
                        }
                    }
                    None => bus.log("not connected"),
                }
                ctx.request_repaint();
            }
        }
    }
}

/// Stop any running guide loop and calibration task, and clear the guiding telemetry flags.
fn stop_guiding(
    guide_loop: &mut Option<GuideLoop>,
    calib_task: &mut Option<JoinHandle<()>>,
    bus: &Bus,
) {
    if let Some(g) = guide_loop.take() {
        g.shutdown();
    }
    if let Some(t) = calib_task.take() {
        t.abort();
    }
    if let Ok(mut sh) = bus.shared.lock() {
        sh.guiding = false;
        sh.calibrating = false;
        sh.guide_err = None;
    }
    bus.refresh_detect();
}

/// Begin guiding: require a calibration and a running stream, lock onto the current target
/// (waiting briefly for a fresh detection), and spawn the guide loop.
async fn start_guiding(
    s: &Session,
    bus: &Bus,
    ctx: &egui::Context,
    guide_loop: &mut Option<GuideLoop>,
) {
    let (streaming, calibrated) = bus
        .shared
        .lock()
        .map(|sh| (sh.streaming, sh.calibrated))
        .unwrap_or((false, false));
    if !streaming {
        bus.log("start the stream before guiding");
        return;
    }
    if !calibrated {
        bus.log("calibrate before guiding");
        return;
    }
    let Some(mount) = s.clone_mount().await else {
        bus.log("no mount selected — cannot guide");
        return;
    };

    // Turn on detection and lock onto the current target.
    if let Ok(mut sh) = bus.shared.lock() {
        sh.guiding = true;
    }
    bus.refresh_detect();
    bus.bump_ref_generation(); // Surface: recapture the reference at the lock position
    let base_seq = bus.guide_sample.load_full().map(|s| s.seq).unwrap_or(0);
    match guiding::next_sample(bus, base_seq, Duration::from_secs(2)).await {
        Some(sample) => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.lock_point = Some((sample.x, sample.y));
                sh.guide_err = None;
                sh.guide_rms = 0.0;
                sh.guide_history.clear();
            }
            let stop = Arc::new(AtomicBool::new(false));
            let (b, c, st) = (bus.clone(), ctx.clone(), stop.clone());
            let task = tokio::spawn(async move {
                guiding::run_guide_loop(mount, b, c, st).await;
            });
            *guide_loop = Some(GuideLoop { task, stop });
            bus.log("guiding started");
        }
        None => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.guiding = false;
            }
            bus.refresh_detect();
            bus.log("no target detected — cannot lock");
        }
    }
}

fn set_conn(bus: &Bus, state: ConnState) {
    if let Ok(mut sh) = bus.shared.lock() {
        sh.conn = state;
    }
}

/// Read the bound camera's full sensor size into [`Shared`] and seed the ROI to the full frame.
/// Best-effort: a camera without `CCD_INFO` just leaves the ROI controls disabled (size 0).
async fn init_sensor_size(s: &Session, bus: &Bus) {
    let Some(cam) = s.camera.as_ref() else { return };
    match cam.sensor_size().await {
        Ok((w, h)) => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.sensor_w = w;
                sh.sensor_h = h;
                sh.roi = (0, 0, w, h);
            }
            bus.set_stream_geometry(w, h);
            bus.set_sensor_size(w, h);
            bus.log(format!("sensor: {w}×{h}"));
        }
        Err(e) => bus.log(format!("reading sensor size: {e}")),
    }
}

/// Camera switch properties that control the streamed pixel format / bit depth, in UI order.
/// Whichever the driver exposes are surfaced as dropdowns; names are driver-specific. The depth
/// that reaches the SER recorder is the product of these — e.g. on Player One you need
/// `CCD_STREAM_ENCODER=RAW`, `CCD_VIDEO_FORMAT=POA_RAW16`, and `STREAM_FULL_DEPTH=FULL_DEPTH_16BIT`
/// together for a true 16-bit stream.
const STREAM_SWITCH_PROPS: &[(&str, &str)] = &[
    ("CCD_STREAM_ENCODER", "Encoder"),
    ("CCD_VIDEO_FORMAT", "Video format"),
    ("STREAM_FULL_DEPTH", "Stream depth"),
    ("SENSOR_MODE", "Sensor mode"),
    ("CCD_CAPTURE_FORMAT", "Capture format"),
];

/// Probe the bound camera's stream-format switch properties into [`Shared`] so the UI pickers can
/// offer them. Best-effort: properties the driver lacks are simply omitted.
async fn init_camera_formats(s: &Session, bus: &Bus) {
    let Some(cam) = s.camera.as_ref() else { return };
    let mut switches = Vec::new();
    for (prop, label) in STREAM_SWITCH_PROPS {
        let (options, selected) = cam.switch_options(prop).await;
        if !options.is_empty() {
            switches.push(CameraSwitch {
                prop: prop.to_string(),
                label: label.to_string(),
                options,
                selected: selected.unwrap_or_default(),
            });
        }
    }
    if let Ok(mut sh) = bus.shared.lock() {
        sh.stream_switches = switches;
    }
}

/// A raw, still-encoded video frame handed from the reader task to the decode thread.
/// `data` is the INDI blob buffer, shared by cloning the `Arc` so the reader never copies a
/// multi-megabyte frame.
struct RawFrame {
    data: Arc<Vec<u8>>,
    format: Option<String>,
}

/// Latest-wins hand-off slot between the async reader task and the decode thread.
///
/// The reader overwrites `latest` on every arriving frame; the decoder always takes the
/// newest one. Frames the decoder can't keep up with are dropped here rather than stalling
/// the reader — so decode speed never throttles how fast we drain the camera, and the live
/// view always shows the most recent frame.
struct DecodeSlot {
    state: Mutex<SlotState>,
    cv: Condvar,
}

struct SlotState {
    latest: Option<RawFrame>,
    stop: bool,
}

impl DecodeSlot {
    fn new() -> Self {
        DecodeSlot {
            state: Mutex::new(SlotState {
                latest: None,
                stop: false,
            }),
            cv: Condvar::new(),
        }
    }

    /// Reader: publish the newest raw frame, dropping any still-undecoded one.
    fn push(&self, raw: RawFrame) {
        {
            let mut s = self.state.lock().unwrap();
            s.latest = Some(raw);
        }
        self.cv.notify_one();
    }

    /// Decoder: block until a frame is available, returning `None` once shutdown is
    /// signalled and no frame remains.
    fn wait(&self) -> Option<RawFrame> {
        let mut s = self.state.lock().unwrap();
        loop {
            if let Some(raw) = s.latest.take() {
                return Some(raw);
            }
            if s.stop {
                return None;
            }
            s = self.cv.wait(s).unwrap();
        }
    }

    /// Signal the decode thread to exit.
    fn stop(&self) {
        {
            let mut s = self.state.lock().unwrap();
            s.stop = true;
        }
        self.cv.notify_all();
    }
}

/// The running video pipeline: the async reader task, the decode thread, and the hand-off
/// slot between them. Call [`FrameStream::shutdown`] to stop and join cleanly (dropping
/// alone would detach the decode thread).
struct FrameStream {
    reader: JoinHandle<()>,
    decoder: std::thread::JoinHandle<()>,
    slot: Arc<DecodeSlot>,
}

impl FrameStream {
    fn shutdown(self) {
        self.reader.abort();
        self.slot.stop();
        let _ = self.decoder.join();
    }
}

/// Subscribe to the CCD1 BLOB property (on `dev`, the dedicated blob connection when
/// available) and start the two-stage video pipeline: a reader task that drains the camera
/// broadcast at wire speed, and a dedicated decode thread that decodes/stretches the newest
/// frame and publishes it. Decode never blocks the reader, so the camera/USB link is the
/// limit — not the capture software.
async fn spawn_frame_task(
    dev: &ActiveDevice,
    bus: Bus,
    ctx: egui::Context,
) -> Option<FrameStream> {
    let param = match dev.get_parameter("CCD1").await {
        Ok(p) => p,
        Err(e) => {
            bus.log(format!("subscribing to CCD1 failed: {e:?}"));
            return None;
        }
    };

    let slot = Arc::new(DecodeSlot::new());

    // Decode thread: runs off the async runtime because MJPEG decode + the per-pixel stretch
    // are blocking CPU work. It always decodes the newest frame the reader has published,
    // dropping stale ones. A frame that fails to decode is logged and skipped — the stream
    // keeps running.
    let decoder = {
        let slot = slot.clone();
        let bus = bus.clone();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let mut seq: u64 = 0;
            let mut last: Option<Instant> = None;
            // Guiding detector state persists across frames (holds the Surface reference patch).
            let mut detector = GuideDetector::default();
            let mut last_detect: Option<Instant> = None;
            let mut last_display: Option<Instant> = None;
            while let Some(raw) = slot.wait() {
                match decode_frame(&bus, &raw, seq + 1) {
                    Ok((frame, raw_pixels)) => {
                        seq += 1;
                        let now = Instant::now();
                        if let Ok(mut sh) = bus.shared.lock() {
                            sh.frame_count = seq;
                            if let Some(prev) = last {
                                let dt = now.duration_since(prev).as_secs_f32();
                                if dt > 0.0 && dt < 1.0 {
                                    let inst = 1.0 / dt;
                                    sh.fps = if sh.fps > 0.0 {
                                        0.9 * sh.fps + 0.1 * inst
                                    } else {
                                        inst
                                    };
                                } else if dt >= 1.0 {
                                    // A gap this large means the stream stalled or was just
                                    // (re)started — drop the stale rate instead of dragging the
                                    // EMA down with one huge interval.
                                    sh.fps = 0.0;
                                }
                            }
                        }
                        last = Some(now);

                        // Guiding detection (throttled): measure the target position and publish
                        // it for the guide loop + overlay. Runs only when enabled, and at most
                        // every DETECT_INTERVAL regardless of frame rate.
                        if bus.detect_enabled() {
                            let due = last_detect.is_none_or(|t| now.duration_since(t) >= DETECT_INTERVAL);
                            if due {
                                last_detect = Some(now);
                                let detected = detector.measure(&frame, &bus);
                                if let Some((x, y)) = detected {
                                    bus.publish_guide_sample(GuideSample { x, y, seq });
                                }
                                if let Ok(mut sh) = bus.shared.lock() {
                                    sh.detected = detected;
                                }
                            }
                        }

                        // Recording: append this frame to the open SER file — native raw bytes
                        // for a RAW stream, or RGB extracted from the MJPEG-decoded frame. A
                        // no-op (no lock taken) unless a recording is armed.
                        if bus.recording_active() {
                            match &raw_pixels {
                                Some(bytes) => bus.write_record_frame(bytes),
                                None => {
                                    let rgb: Vec<u8> = frame
                                        .rgba
                                        .chunks_exact(4)
                                        .flat_map(|p| [p[0], p[1], p[2]])
                                        .collect();
                                    bus.write_record_frame(&rgb);
                                }
                            }
                        }

                        // Do the display stretch + Color32 conversion here, off the GUI
                        // thread, and publish a ready-to-upload image. Keep the raw frame
                        // for capture. While recording, rate-limit this to spare CPU for the
                        // SER writer — the preview refreshes at a few Hz instead of full FPS.
                        let display_due = !bus.recording_active()
                            || last_display
                                .is_none_or(|t| now.duration_since(t) >= DISPLAY_INTERVAL_RECORDING);
                        if display_due {
                            last_display = Some(now);
                            let (auto, gain) = bus.display_settings();
                            let img = frame.to_display_image(auto, gain);
                            bus.publish_display(img);
                            ctx.request_repaint();
                        }
                        bus.latest_frame.store(Some(Arc::new(frame)));
                    }
                    Err(e) => tracing::warn!("frame decode failed: {e}"),
                }
            }
        })
    };

    // Reader task: drain the camera broadcast as fast as it delivers, handing the newest raw
    // frame to the decode thread. It never decodes, so `indi` rarely lags on our account.
    let reader = {
        let slot = slot.clone();
        tokio::spawn(async move {
            let mut changes = param.changes();
            while let Some(update) = changes.next().await {
                let param = match update {
                    Ok(p) => p,
                    Err(_) => continue, // lagged broadcast; skip
                };
                let Parameter::BlobVector(bv) = param.as_ref() else {
                    continue;
                };
                let Some(blob) = bv.values.get("CCD1") else {
                    continue;
                };
                let Some(data) = &blob.value else { continue };
                if data.is_empty() {
                    continue;
                }
                slot.push(RawFrame {
                    data: data.clone(),
                    format: blob.format.clone(),
                });
            }
        })
    };

    Some(FrameStream {
        reader,
        decoder,
        slot,
    })
}

/// Decode one raw video BLOB into a display [`Frame`], and return the native payload to record
/// when a recording is armed. MJPEG (`.stream_jpg`) is decoded by `image`; RAW streams
/// (`.stream` / zlib-compressed `.stream.z`) are interpreted using the current readout geometry.
/// The returned `Option<Vec<u8>>` is the sensor-native bytes to write to SER for a raw stream
/// (the MJPEG path returns `None` — the caller extracts RGB from the decoded frame instead), and
/// is only materialized when [`Bus::recording_active`] is set, to keep the non-recording hot path
/// allocation-free.
fn decode_frame(bus: &Bus, raw: &RawFrame, seq: u64) -> Result<(Frame, Option<Vec<u8>>)> {
    let fmt = raw.format.as_deref();
    // Trust the payload's own magic bytes over the driver's format label: some drivers
    // (e.g. Player One) keep delivering MJPEG frames while `CCD_STREAM_ENCODER` reports RAW, so
    // a label-only check misroutes compressed frames to the raw decoder and every frame fails the
    // pixel-count assertion. Sniffing the header decodes them correctly regardless of the label.
    let label_jpeg = fmt
        .map(|f| f.contains("jpg") || f.contains("jpeg"))
        .unwrap_or(false);
    if label_jpeg || looks_like_jpeg(&raw.data) {
        return Ok((Frame::from_stream_blob(fmt, &raw.data, seq)?, None));
    }

    let label_zlib = fmt.map(|f| f.ends_with(".z")).unwrap_or(false);
    if label_zlib || looks_like_zlib(&raw.data) {
        // Compressed raw: we must inflate for the display decode anyway, so reuse the buffer.
        let pixels = inflate_zlib(&raw.data).map_err(|e| anyhow!("inflating raw stream: {e}"))?;
        bus.set_last_raw_len(pixels.len());
        let (w, h) = resolve_raw_geometry(bus, pixels.len());
        let frame = Frame::from_raw_stream(&pixels, w, h, seq)?;
        let rec = bus.recording_active().then_some(pixels);
        Ok((frame, rec))
    } else {
        bus.set_last_raw_len(raw.data.len());
        let (w, h) = resolve_raw_geometry(bus, raw.data.len());
        let frame = Frame::from_raw_stream(&raw.data, w, h, seq)?;
        // Clone the native bytes only when recording is actually running.
        let rec = bus.recording_active().then(|| raw.data.as_ref().clone());
        Ok((frame, rec))
    }
}

/// A JPEG stream begins with the SOI marker `FF D8 FF`. Used to detect MJPEG frames a driver
/// mislabels (see [`decode_frame`]).
fn looks_like_jpeg(data: &[u8]) -> bool {
    matches!(data, [0xFF, 0xD8, 0xFF, ..])
}

/// A zlib stream begins with a header byte whose low nibble is 8 (deflate) followed by a byte that
/// makes the two-byte header a multiple of 31. `0x78` (32K window) is by far the most common first
/// byte; checking the checksum keeps this from matching arbitrary raw pixel data.
fn looks_like_zlib(data: &[u8]) -> bool {
    match data {
        [cmf, flg, ..] => (cmf & 0x0F) == 8 && (u16::from(*cmf) << 8 | u16::from(*flg)) % 31 == 0,
        _ => false,
    }
}

/// Choose the `(width, height)` to decode a dimensionless raw frame with. Normally this is the
/// requested readout geometry, but some drivers (e.g. Player One) stream the *full sensor* even
/// after a subframe is set — `CCD_STREAM_FRAME` then echoes the requested ROI while the pixels on
/// the wire are full-frame, so every frame would fail the size check. We detect that by comparing
/// the payload length against both candidates and, if only the full sensor matches, decode as full
/// sensor and correct the stored geometry (so recording sizes its SER file right too), logging the
/// discrepancy once. Falls back to the requested geometry when neither matches, preserving the
/// original error path.
fn resolve_raw_geometry(bus: &Bus, len: usize) -> (usize, usize) {
    let (rw, rh) = bus.frame_geometry();
    let matches = |w: u32, h: u32| {
        let px = (w as usize).saturating_mul(h as usize);
        px != 0 && (len == px || len == px * 2)
    };
    if matches(rw, rh) {
        return (rw as usize, rh as usize);
    }
    let (sw, sh) = bus.sensor_size();
    if matches(sw, sh) && (sw, sh) != (rw, rh) {
        bus.set_stream_geometry(sw, sh);
        bus.log(format!(
            "driver streamed full sensor {sw}×{sh}, not the requested ROI {rw}×{rh} — \
             showing full frame (this camera may not support subframing the live stream)"
        ));
        return (sw as usize, sh as usize);
    }
    // Neither matched: keep the requested geometry so `from_raw_stream` reports the mismatch.
    (rw as usize, rh as usize)
}

/// Borrow the bound camera, or error if none is selected.
fn camera(s: &Session) -> Result<&Camera> {
    s.camera
        .as_ref()
        .ok_or_else(|| anyhow!("no camera selected"))
}

/// Borrow the bound mount, or error if none is selected.
fn mount(s: &Session) -> Result<&Mount> {
    s.mount.as_ref().ok_or_else(|| anyhow!("no mount selected"))
}

/// Translate a non-lifecycle command into INDI property changes.
async fn dispatch(cmd: Command, s: &Session, bus: &Bus) -> Result<()> {
    match cmd {
        Command::StartStream => {
            let cam = camera(s)?;
            cam.start_stream().await?;
            set_streaming(bus, true);
            bus.log("video stream on");
            // Surface the driver's client preview-fps cap: it defaults low (~10) on many drivers
            // and throttles delivery regardless of ROI. `start_stream` raises it to this max.
            if let Some((val, _min, max)) = cam.number_range("LIMITS", "LIMITS_PREVIEW_FPS").await {
                bus.log(format!("preview-fps limit {val:.0} (max {max:.0})"));
            }
        }
        Command::StopStream => {
            camera(s)?.stop_stream().await?;
            set_streaming(bus, false);
            if let Ok(mut sh) = bus.shared.lock() {
                sh.fps = 0.0;
            }
            bus.log("video stream off");
        }
        Command::SetGain(v) => {
            let cam = camera(s)?;
            cam.set_gain(v).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.gain = v;
            }
            match cam.number_range("CCD_GAIN", "GAIN").await {
                Some((val, min, max)) => {
                    bus.log(format!("gain → {val:.0} (valid {min:.0}–{max:.0})"))
                }
                None => bus.log(format!("gain → {v:.0}")),
            }
        }
        Command::SetExposure(v) => {
            // Prefer the live-stream exposure (`STREAMING_EXPOSURE`) — it governs the stream frame
            // rate. Only fall back to the still `CCD_EXPOSURE` when the camera lacks a separate
            // streaming exposure (e.g. the simulator): setting `CCD_EXPOSURE` while streaming
            // triggers a one-off still capture whose frame disrupts the video stream.
            let cam = camera(s)?;
            let streaming = bus.shared.lock().map(|sh| sh.streaming).unwrap_or(false);
            let streamed = tokio::time::timeout(Duration::from_secs(2), cam.set_streaming_exposure(v))
                .await
                .map(|r| r.is_ok())
                .unwrap_or(false);
            if streamed {
                // The driver applies a new streaming exposure only on stream (re)start, so bounce
                // it if we're live — otherwise the frame rate wouldn't change until next start.
                if streaming {
                    let _ = cam.stop_stream().await;
                    if let Err(e) = cam.start_stream().await {
                        bus.log(format!("restarting stream after exposure change failed: {e}"));
                    }
                }
            } else {
                cam.set_exposure(v).await?;
            }
            if let Ok(mut sh) = bus.shared.lock() {
                sh.exposure = v;
            }
            // Read back the value the driver actually holds and its valid range, so an
            // out-of-range request (silently clamped/rejected) is visible. For the streaming
            // exposure, the value is also the frame-rate ceiling (1/exposure) — the number that
            // caps streaming fps regardless of ROI.
            let (prop, elem, label) = if streamed {
                ("STREAMING_EXPOSURE", "STREAMING_EXPOSURE_VALUE", "streaming exposure")
            } else {
                ("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", "exposure")
            };
            match cam.number_range(prop, elem).await {
                Some((val, min, max)) => {
                    let ceiling = if val > 0.0 {
                        format!(", ≈{:.1} fps ceiling", 1.0 / val)
                    } else {
                        String::new()
                    };
                    bus.log(format!(
                        "{label} → {val:.4}s (valid {min:.4}–{max:.4}s{ceiling})"
                    ));
                }
                None => bus.log(format!("{label} → {v:.4}s")),
            }
        }
        Command::Nudge { dir, active } => mount(s)?.nudge(dir, active).await?,
        Command::SetSlewRate(idx) => {
            let name = bus
                .shared
                .lock()
                .ok()
                .and_then(|sh| sh.slew_rates.get(idx).cloned());
            if let Some(name) = name {
                mount(s)?.set_slew_rate(&name).await?;
                if let Ok(mut sh) = bus.shared.lock() {
                    sh.slew_rate_idx = idx;
                }
            }
        }
        Command::SetTracking(on) => {
            mount(s)?.set_tracking(on).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.tracking = on;
            }
        }
        Command::Abort => mount(s)?.abort().await?,
        Command::CaptureFrame { dir } => capture(bus, &dir)?,
        Command::SetCameraSwitch { prop, elem } => {
            let cam = camera(s)?;
            cam.set_switch(&prop, &elem).await?;
            // Re-read the switch so the UI reflects the driver's actual state (some options are
            // interlocked). Also refresh any sibling stream switches that may have changed.
            for sw_prop in STREAM_SWITCH_PROPS.iter().map(|(p, _)| *p) {
                let (options, selected) = cam.switch_options(sw_prop).await;
                if options.is_empty() {
                    continue;
                }
                if let Ok(mut sh) = bus.shared.lock() {
                    if let Some(sw) = sh.stream_switches.iter_mut().find(|s| s.prop == sw_prop) {
                        sw.options = options;
                        sw.selected = selected.unwrap_or_default();
                    }
                }
            }
            bus.log(format!("{prop} → {elem}"));
        }
        Command::SetGuideMode(mode) => {
            bus.set_guide_mode(mode);
            bus.bump_ref_generation(); // Surface: recapture reference under the new mode
            if let Ok(mut sh) = bus.shared.lock() {
                sh.guide_mode = mode;
            }
            bus.log(format!("guide mode: {mode:?}"));
        }
        Command::SetGuideParams(params) => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.guide_params = params;
            }
        }
        Command::SetDetectionOverlay(on) => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.detect_overlay = on;
                if !on {
                    sh.detected = None;
                }
            }
            bus.refresh_detect();
        }
        Command::Relock => relock(bus).await,
        Command::SetRoi { x, y, w, h } => set_roi(s, bus, x, y, w, h).await?,
        Command::ResetRoi => {
            let (w, h) = {
                let sh = bus.shared.lock().unwrap();
                (sh.sensor_w, sh.sensor_h)
            };
            if w == 0 || h == 0 {
                return Err(anyhow!("sensor size unknown; cannot reset ROI"));
            }
            set_roi(s, bus, 0, 0, w, h).await?;
        }
        // handled in run() (need &mut session / task handles):
        Command::Connect { .. }
        | Command::Disconnect
        | Command::SelectCamera(_)
        | Command::SelectMount(_)
        | Command::Calibrate
        | Command::StartGuiding
        | Command::StopGuiding
        | Command::StartRecording(_)
        | Command::StopRecording => {}
    }
    Ok(())
}

/// Re-acquire the lock point (and Surface reference) at the current target position. Only
/// meaningful while detection is running (guiding or overlay on).
async fn relock(bus: &Bus) {
    bus.bump_ref_generation();
    let base_seq = bus.guide_sample.load_full().map(|s| s.seq).unwrap_or(0);
    match guiding::next_sample(bus, base_seq, Duration::from_secs(2)).await {
        Some(sample) => {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.lock_point = Some((sample.x, sample.y));
                sh.guide_err = None;
                sh.guide_rms = 0.0;
                sh.guide_history.clear();
            }
            bus.log("re-locked");
        }
        None => bus.log("re-lock: no target detected"),
    }
}

fn set_streaming(bus: &Bus, on: bool) {
    if let Ok(mut sh) = bus.shared.lock() {
        sh.streaming = on;
    }
}

/// Apply a readout region (ROI) to the camera. Changing `CCD_FRAME` while streaming is
/// unreliable across drivers, so if a stream is running we stop it, set the frame, and restart
/// it. The applied region is recorded in [`Shared::roi`].
async fn set_roi(s: &Session, bus: &Bus, x: u32, y: u32, w: u32, h: u32) -> Result<()> {
    let cam = camera(s)?;
    let streaming = bus
        .shared
        .lock()
        .map(|sh| sh.streaming)
        .unwrap_or(false);
    if streaming {
        cam.stop_stream().await?;
    }
    let result = cam.set_frame(x, y, w, h).await;
    // The driver may snap the requested width/height to hardware alignment (e.g. width to a
    // multiple of 8), so read back the region it actually applied and use THAT for the decode
    // geometry — otherwise every raw frame fails the size check. Do this before the restart so
    // the first streamed frames already match. Fall back to the requested values if unreadable.
    let (ax, ay, aw, ah) = cam.read_applied_roi().await.unwrap_or((x, y, w, h));
    bus.set_stream_geometry(aw, ah);
    if streaming {
        // Best-effort restart even if the frame change failed, so the stream isn't left off.
        if let Err(e) = cam.start_stream().await {
            bus.log(format!("restarting stream after ROI change failed: {e}"));
        }
    }
    result?;
    if let Ok(mut sh) = bus.shared.lock() {
        sh.roi = (ax, ay, aw, ah);
    }
    if (aw, ah) != (w, h) {
        bus.log(format!(
            "ROI requested {w}×{h}, driver applied {aw}×{ah} at ({ax},{ay})"
        ));
    } else {
        bus.log(format!("ROI set to {aw}×{ah} at ({ax},{ay})"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn raw(n: u8) -> RawFrame {
        RawFrame {
            data: Arc::new(vec![n]),
            format: None,
        }
    }

    #[test]
    fn raw_geometry_prefers_the_requested_roi() {
        let bus = Bus::new();
        bus.set_sensor_size(3856, 2180);
        bus.set_stream_geometry(1500, 1278);
        // 8-bit ROI payload → ROI geometry, unchanged.
        assert_eq!(resolve_raw_geometry(&bus, 1500 * 1278), (1500, 1278));
        assert_eq!(bus.frame_geometry(), (1500, 1278));
        // 16-bit ROI payload also matches the ROI.
        assert_eq!(resolve_raw_geometry(&bus, 1500 * 1278 * 2), (1500, 1278));
    }

    #[test]
    fn raw_geometry_falls_back_to_full_sensor() {
        // Driver ignored the ROI and streamed the full sensor: decode as full frame and correct
        // the stored geometry so later frames match without re-logging.
        let bus = Bus::new();
        bus.set_sensor_size(3856, 2180);
        bus.set_stream_geometry(1500, 1278);
        assert_eq!(resolve_raw_geometry(&bus, 3856 * 2180), (3856, 2180));
        assert_eq!(bus.frame_geometry(), (3856, 2180));
    }

    #[test]
    fn raw_geometry_keeps_roi_when_nothing_matches() {
        // Unknown payload size: keep the requested geometry so from_raw_stream reports the error.
        let bus = Bus::new();
        bus.set_sensor_size(3856, 2180);
        bus.set_stream_geometry(1500, 1278);
        assert_eq!(resolve_raw_geometry(&bus, 12345), (1500, 1278));
        assert_eq!(bus.frame_geometry(), (1500, 1278));
    }

    #[test]
    fn slot_keeps_only_the_newest_frame() {
        // Reader outruns the decoder: three frames pushed before a single take. The two
        // older, undecoded frames are dropped so the live view stays current.
        let slot = DecodeSlot::new();
        slot.push(raw(1));
        slot.push(raw(2));
        slot.push(raw(3));
        assert_eq!(*slot.wait().unwrap().data, vec![3u8]);
    }

    #[test]
    fn stop_wakes_a_blocked_decoder() {
        // A decoder blocked with no pending frame must be woken by shutdown, not hang.
        let slot = Arc::new(DecodeSlot::new());
        let waiter = {
            let slot = slot.clone();
            std::thread::spawn(move || slot.wait())
        };
        std::thread::sleep(Duration::from_millis(50));
        slot.stop();
        assert!(waiter.join().unwrap().is_none());
    }

    #[test]
    fn a_frame_pushed_before_stop_is_still_delivered() {
        let slot = DecodeSlot::new();
        slot.push(raw(7));
        slot.stop();
        assert_eq!(*slot.wait().unwrap().data, vec![7u8]);
        assert!(slot.wait().is_none());
    }
}

/// Save the current live frame to a timestamped PNG under `dir`.
fn capture(bus: &Bus, dir: &str) -> Result<()> {
    let frame = bus
        .latest_frame
        .load_full()
        .ok_or_else(|| anyhow!("no frame to capture yet"))?;
    std::fs::create_dir_all(dir).ok();
    let millis = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let path = format!("{dir}/solar_{millis}.png");
    let img = image::RgbaImage::from_raw(
        frame.width as u32,
        frame.height as u32,
        frame.rgba.clone(),
    )
    .ok_or_else(|| anyhow!("frame buffer size mismatch"))?;
    img.save(&path)?;
    if let Ok(mut sh) = bus.shared.lock() {
        sh.last_capture = Some(path.clone());
    }
    bus.log(format!("captured {path}"));
    Ok(())
}

/// Begin a recording sequence: require a running stream, capture the camera's Bayer/mono layout
/// and current encoder, then spawn the orchestrator task.
async fn start_recording(
    s: &Session,
    bus: &Bus,
    ctx: &egui::Context,
    cfg: RecordConfig,
    record_task: &mut Option<RecordTask>,
) {
    let (streaming, is_mjpeg) = bus
        .shared
        .lock()
        .map(|sh| {
            let mjpeg = sh
                .stream_switches
                .iter()
                .find(|s| s.prop == "CCD_STREAM_ENCODER")
                .map(|s| s.selected.eq_ignore_ascii_case("MJPEG"))
                .unwrap_or(false);
            (sh.streaming, mjpeg)
        })
        .unwrap_or((false, false));
    if !streaming {
        bus.log("start the stream before recording");
        return;
    }
    let Some(cam) = s.camera.as_ref() else {
        bus.log("no camera selected — cannot record");
        return;
    };
    // Bayer pattern (mono cameras report none → recorded as SER MONO).
    let cfa = cam.cfa().await;

    let stop = Arc::new(AtomicBool::new(false));
    let (b, c, st) = (bus.clone(), ctx.clone(), stop.clone());
    let task = tokio::spawn(async move {
        run_recording(cfg, is_mjpeg, cfa, b, c, st).await;
    });
    *record_task = Some(RecordTask { task, stop });
}

/// Stop the recording orchestrator and finalize any SER file it left open (so it isn't left with
/// a zero `FrameCount`). Safe to call when not recording.
fn stop_recording(record_task: &mut Option<RecordTask>, bus: &Bus) {
    if let Some(t) = record_task.take() {
        t.stop.store(true, Ordering::Relaxed);
        t.task.abort();
    }
    bus.recording_active.store(false, Ordering::Relaxed);
    // The orchestrator may have been aborted mid-video; finalize whatever remains installed.
    let leftover = bus.recorder.lock().ok().and_then(|mut g| g.take());
    if let Some(rec) = leftover {
        if let Err(e) = rec.finalize() {
            bus.log(format!("finalizing SER on stop: {e}"));
        }
    }
    if let Ok(mut sh) = bus.shared.lock() {
        sh.recording.active = false;
        sh.recording.phase = RecordPhase::Idle;
    }
}

/// Recording orchestrator: writes `cfg.count` SER videos back-to-back, each ended by `cfg.stop`
/// (a frame count or a duration), with `cfg.delay_secs` between them. The stream keeps running
/// throughout — only frame *writing* pauses between videos — so guiding stays locked the whole
/// time.
async fn run_recording(
    cfg: RecordConfig,
    is_mjpeg: bool,
    cfa: Option<String>,
    bus: Bus,
    ctx: egui::Context,
    stop: Arc<AtomicBool>,
) {
    let total = cfg.count.max(1);
    let dir = PathBuf::from(&cfg.dir);
    if let Err(e) = std::fs::create_dir_all(&dir) {
        bus.log(format!("recording: cannot create {}: {e}", cfg.dir));
        return;
    }
    if let Ok(mut sh) = bus.shared.lock() {
        sh.recording.active = true;
        sh.recording.phase = RecordPhase::Recording;
        sh.recording.current = 0;
        sh.recording.total = total;
        sh.recording.frames_written = 0;
        sh.recording.dropped = 0;
        sh.recording.elapsed_secs = 0.0;
    }
    ctx.request_repaint();

    // Shared filename stem for the whole sequence.
    let base = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);

    for i in 1..=total {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        // Wait for a frame so geometry (and, for raw, the payload size → bit depth) is known.
        let Some((w, h)) = wait_for_geometry(&bus, &stop, Duration::from_secs(5)).await else {
            bus.log("recording: no frames — is the stream running?");
            break;
        };
        // Choose the SER color layout and bit depth. MJPEG frames are recorded as 8-bit RGB;
        // raw frames keep the sensor's native depth, inferred from the payload byte length.
        let (color, depth) = if is_mjpeg {
            (SerColor::Rgb, 8u8)
        } else {
            let color = SerColor::from_cfa(cfa.as_deref());
            let px = (w as usize) * (h as usize);
            let bpp = if px > 0 { (bus.last_raw_len() / px).max(1) } else { 1 };
            (color, if bpp >= 2 { 16u8 } else { 8u8 })
        };

        let path = dir.join(format!("solar_{base}_{i:03}.ser"));
        let rec = match SerRecorder::create_file(&path, w, h, color, depth) {
            Ok(r) => r,
            Err(e) => {
                bus.log(format!("recording: {e}"));
                break;
            }
        };
        if let Ok(mut g) = bus.recorder.lock() {
            *g = Some(rec);
        }
        bus.recording_active.store(true, Ordering::Relaxed);
        if let Ok(mut sh) = bus.shared.lock() {
            sh.recording.phase = RecordPhase::Recording;
            sh.recording.current = i;
            sh.recording.frames_written = 0;
            sh.recording.elapsed_secs = 0.0;
        }
        bus.log(format!(
            "recording {i}/{total} ({w}×{h}, {depth}-bit) → {}",
            path.display()
        ));
        ctx.request_repaint();

        // Run until the frame/time budget is met (or a stop is requested), updating progress.
        let start = Instant::now();
        loop {
            tokio::time::sleep(Duration::from_millis(100)).await;
            let stopping = stop.load(Ordering::Relaxed);
            let frames = bus
                .recorder
                .lock()
                .ok()
                .and_then(|g| g.as_ref().map(|r| r.frame_count()))
                .unwrap_or(0) as u64;
            let elapsed = start.elapsed().as_secs_f64();
            if let Ok(mut sh) = bus.shared.lock() {
                sh.recording.frames_written = frames;
                sh.recording.elapsed_secs = elapsed;
            }
            ctx.request_repaint();
            let done = match cfg.stop {
                RecordStop::Frames(n) => frames >= n,
                RecordStop::Seconds(secs) => elapsed >= secs,
            };
            if stopping || done {
                break;
            }
        }

        // Disarm the decode thread and finalize this video.
        bus.recording_active.store(false, Ordering::Relaxed);
        let rec = bus.recorder.lock().ok().and_then(|mut g| g.take());
        if let Some(rec) = rec {
            match rec.finalize() {
                Ok(n) => {
                    let p = path.display().to_string();
                    if let Ok(mut sh) = bus.shared.lock() {
                        sh.recording.last_file = Some(p.clone());
                    }
                    bus.log(format!("saved {p} ({n} frames)"));
                }
                Err(e) => bus.log(format!("finalizing SER: {e}")),
            }
        }
        ctx.request_repaint();

        // Inter-video delay: the stream keeps flowing (guiding stays locked); we just don't write.
        if i < total && !stop.load(Ordering::Relaxed) && cfg.delay_secs > 0.0 {
            if let Ok(mut sh) = bus.shared.lock() {
                sh.recording.phase = RecordPhase::Waiting;
            }
            ctx.request_repaint();
            interruptible_sleep(cfg.delay_secs, &stop).await;
        }
    }

    // Sequence finished (or was stopped): clear the active flag; keep `last_file` for the UI.
    bus.recording_active.store(false, Ordering::Relaxed);
    if let Ok(mut sh) = bus.shared.lock() {
        sh.recording.active = false;
        sh.recording.phase = RecordPhase::Idle;
    }
    ctx.request_repaint();
}

/// Sleep for `secs`, returning early if `stop` is set (checked at ~100 ms granularity).
async fn interruptible_sleep(secs: f64, stop: &Arc<AtomicBool>) {
    let deadline = Instant::now() + Duration::from_secs_f64(secs.max(0.0));
    while Instant::now() < deadline {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// Wait until a decoded frame with a known size is available, returning its `(w, h)` in pixels —
/// or `None` if `stop` fires or `timeout` elapses first.
async fn wait_for_geometry(
    bus: &Bus,
    stop: &Arc<AtomicBool>,
    timeout: Duration,
) -> Option<(u32, u32)> {
    let deadline = Instant::now() + timeout;
    loop {
        if stop.load(Ordering::Relaxed) {
            return None;
        }
        if let Some(frame) = bus.latest_frame.load_full() {
            if frame.width > 0 && frame.height > 0 {
                return Some((frame.width as u32, frame.height as u32));
            }
        }
        if Instant::now() >= deadline {
            return None;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}
