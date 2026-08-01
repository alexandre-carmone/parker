//! The async INDI worker: owns the connection/session, decodes the video stream into the
//! shared frame slot, and translates GUI [`Command`]s into INDI property changes.

use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};
use tokio::sync::mpsc::UnboundedReceiver;
use tokio::task::JoinHandle;
use tokio_stream::StreamExt;

use indi::Parameter; // external crate; `crate::indi` is our local module

use crate::bus::{Bus, Command, ConnState};
use crate::frame::Frame;
use crate::indi::Session;

/// Entry point for the worker task. Runs until the command channel closes.
pub async fn run(mut rx: UnboundedReceiver<Command>, bus: Bus, ctx: egui::Context) {
    let mut session: Option<Session> = None;
    let mut frame_task: Option<JoinHandle<()>> = None;

    while let Some(cmd) = rx.recv().await {
        match cmd {
            Command::Connect { addr } => {
                if let Some(t) = frame_task.take() {
                    t.abort();
                }
                session = None;
                set_conn(&bus, ConnState::Connecting);
                bus.log(format!("connecting to {addr}…"));

                match crate::indi::connect(&addr).await {
                    Ok(s) => {
                        match s.mount.slew_rates().await {
                            Ok(rates) => {
                                if let Ok(mut sh) = bus.shared.lock() {
                                    sh.slew_rates = rates;
                                }
                            }
                            Err(e) => bus.log(format!("reading slew rates: {e}")),
                        }
                        frame_task = spawn_frame_task(&s, bus.clone(), ctx.clone()).await;
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
                if let Some(t) = frame_task.take() {
                    t.abort();
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

/// Subscribe to the CCD1 BLOB property and spawn a task that decodes each frame into the
/// shared latest-frame slot, updating FPS and requesting a repaint.
async fn spawn_frame_task(s: &Session, bus: Bus, ctx: egui::Context) -> Option<JoinHandle<()>> {
    let param = match s.camera.dev.get_parameter("CCD1").await {
        Ok(p) => p,
        Err(e) => {
            bus.log(format!("subscribing to CCD1 failed: {e:?}"));
            return None;
        }
    };

    Some(tokio::spawn(async move {
        let mut changes = param.changes();
        let mut seq: u64 = 0;
        let mut last: Option<Instant> = None;

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

            match Frame::from_stream_blob(blob.format.as_deref(), data, seq + 1) {
                Ok(frame) => {
                    seq += 1;
                    let now = Instant::now();
                    if let Ok(mut sh) = bus.shared.lock() {
                        sh.frame_count = seq;
                        if let Some(prev) = last {
                            let dt = now.duration_since(prev).as_secs_f32();
                            if dt > 0.0 {
                                let inst = 1.0 / dt;
                                sh.fps = if sh.fps > 0.0 {
                                    0.9 * sh.fps + 0.1 * inst
                                } else {
                                    inst
                                };
                            }
                        }
                    }
                    last = Some(now);
                    bus.latest_frame.store(Some(Arc::new(frame)));
                    ctx.request_repaint();
                }
                Err(e) => tracing::warn!("frame decode failed: {e}"),
            }
        }
    }))
}

/// Translate a non-lifecycle command into INDI property changes.
async fn dispatch(cmd: Command, s: &Session, bus: &Bus) -> Result<()> {
    match cmd {
        Command::StartStream => {
            s.camera.start_stream().await?;
            set_streaming(bus, true);
            bus.log("video stream on");
        }
        Command::StopStream => {
            s.camera.stop_stream().await?;
            set_streaming(bus, false);
            bus.log("video stream off");
        }
        Command::SetGain(v) => {
            s.camera.set_gain(v).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.gain = v;
            }
        }
        Command::SetExposure(v) => {
            s.camera.set_exposure(v).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.exposure = v;
            }
        }
        Command::Nudge { dir, active } => s.mount.nudge(dir, active).await?,
        Command::SetSlewRate(idx) => {
            let name = bus
                .shared
                .lock()
                .ok()
                .and_then(|sh| sh.slew_rates.get(idx).cloned());
            if let Some(name) = name {
                s.mount.set_slew_rate(&name).await?;
                if let Ok(mut sh) = bus.shared.lock() {
                    sh.slew_rate_idx = idx;
                }
            }
        }
        Command::SetTracking(on) => {
            s.mount.set_tracking(on).await?;
            if let Ok(mut sh) = bus.shared.lock() {
                sh.tracking = on;
            }
        }
        Command::Abort => s.mount.abort().await?,
        Command::CaptureFrame { dir } => capture(bus, &dir)?,
        Command::Connect { .. } | Command::Disconnect => {} // handled in run()
    }
    Ok(())
}

fn set_streaming(bus: &Bus, on: bool) {
    if let Ok(mut sh) = bus.shared.lock() {
        sh.streaming = on;
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
