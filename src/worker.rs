//! The async INDI worker: owns the connection/session, decodes the video stream into the
//! shared frame slot, and translates GUI [`Command`]s into INDI property changes.

use std::sync::{Arc, Condvar, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};
use tokio::sync::mpsc::UnboundedReceiver;
use tokio::task::JoinHandle;
use tokio_stream::StreamExt;

use indi::client::active_device::ActiveDevice;
use indi::Parameter; // external crate; `crate::indi` is our local module

use crate::bus::{Bus, Command, ConnState};
use crate::frame::Frame;
use crate::indi::camera::Camera;
use crate::indi::mount::Mount;
use crate::indi::Session;

/// Entry point for the worker task. Runs until the command channel closes.
pub async fn run(mut rx: UnboundedReceiver<Command>, bus: Bus, ctx: egui::Context) {
    let mut session: Option<Session> = None;
    let mut frame_task: Option<FrameStream> = None;

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
            other => {
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
            bus.log(format!("sensor: {w}×{h}"));
        }
        Err(e) => bus.log(format!("reading sensor size: {e}")),
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
            while let Some(raw) = slot.wait() {
                match Frame::from_stream_blob(raw.format.as_deref(), &raw.data, seq + 1) {
                    Ok(frame) => {
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
                        // Do the display stretch + Color32 conversion here, off the GUI
                        // thread, and publish a ready-to-upload image. Keep the raw frame
                        // for capture.
                        let (auto, gain) = bus.display_settings();
                        let img = frame.to_display_image(auto, gain);
                        bus.latest_frame.store(Some(Arc::new(frame)));
                        bus.publish_display(img);
                        ctx.request_repaint();
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
            camera(s)?.start_stream().await?;
            set_streaming(bus, true);
            bus.log("video stream on");
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
            camera(s)?.set_gain(v).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.gain = v;
            }
        }
        Command::SetExposure(v) => {
            camera(s)?.set_exposure(v).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.exposure = v;
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
        // handled in run() (need &mut session / frame_task):
        Command::Connect { .. }
        | Command::Disconnect
        | Command::SelectCamera(_)
        | Command::SelectMount(_) => {}
    }
    Ok(())
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
    if streaming {
        // Best-effort restart even if the frame change failed, so the stream isn't left off.
        if let Err(e) = cam.start_stream().await {
            bus.log(format!("restarting stream after ROI change failed: {e}"));
        }
    }
    result?;
    if let Ok(mut sh) = bus.shared.lock() {
        sh.roi = (x, y, w, h);
    }
    bus.log(format!("ROI set to {w}×{h} at ({x},{y})"));
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
