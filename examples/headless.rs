//! Headless M1 verification: drives the real INDI modules against the simulators (no GUI),
//! exercising connect -> stream -> decode -> capture and mount nudge with read-back.
//!
//! Run the simulators first, then:
//!   cargo run --example headless
//! It writes `captures/headless.png` (a decoded live frame) for visual inspection.

use std::time::Duration;

use anyhow::{anyhow, Result};
use tokio::time::timeout;
use tokio_stream::StreamExt;

use solar::bus::Dir;
use solar::frame::Frame;
use solar::indi;

const ADDR: &str = "127.0.0.1:7624";

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter("warn,solar=info")
        .init();

    // 1. Connect + auto-connect both devices.
    let session = indi::connect(ADDR).await?;
    println!("[headless] connected; camera + mount online");

    let rates = session.mount.slew_rates().await?;
    println!("[headless] slew rates: {rates:?}");

    // 2. Start the video stream and collect frames.
    let param = session
        .camera
        .dev
        .get_parameter("CCD1")
        .await
        .map_err(|e| anyhow!("subscribing CCD1: {e:?}"))?;
    let mut changes = param.changes();
    session.camera.start_stream().await?;
    println!("[headless] stream started; waiting for frames…");

    let mut got: Option<Frame> = None;
    let mut count = 0u64;
    while count < 6 {
        let next = timeout(Duration::from_secs(10), changes.next())
            .await
            .map_err(|_| anyhow!("timed out waiting for a frame"))?;
        let Some(update) = next else {
            return Err(anyhow!("frame stream ended"));
        };
        let Ok(param) = update else { continue };
        if let indi::Parameter::BlobVector(bv) = param.as_ref() {
            if let Some(blob) = bv.values.get("CCD1") {
                if let Some(data) = &blob.value {
                    if data.is_empty() {
                        continue;
                    }
                    count += 1;
                    got = Some(Frame::from_stream_blob(blob.format.as_deref(), data, count)?);
                }
            }
        }
    }
    let frame = got.ok_or_else(|| anyhow!("no frame decoded"))?;
    println!(
        "[headless] decoded {count} frames; last is {}x{} ({} bytes RGBA)",
        frame.width,
        frame.height,
        frame.rgba.len()
    );

    // 3. Save a PNG for visual inspection.
    std::fs::create_dir_all("captures").ok();
    let path = "captures/headless.png";
    image::RgbaImage::from_raw(frame.width as u32, frame.height as u32, frame.rgba.clone())
        .ok_or_else(|| anyhow!("frame buffer mismatch"))?
        .save(path)?;
    println!("[headless] wrote {path}");

    // Drop the frame subscription to relieve lock pressure, then stop the stream
    // (best-effort: the single control connection can lag while blobs are in transit).
    drop(changes);
    if let Err(e) = session.camera.stop_stream().await {
        println!("[headless] note: stop_stream lagged: {e}");
    } else {
        println!("[headless] stream stopped");
    }

    // 4. Mount nudge with read-back.
    session.mount.nudge(Dir::North, true).await?;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let moving_on = session.mount.is_moving(Dir::North).await?;
    session.mount.nudge(Dir::North, false).await?;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let moving_off = session.mount.is_moving(Dir::North).await?;
    println!("[headless] nudge North: moving_on={moving_on} moving_off={moving_off}");
    session.mount.abort().await?;

    if !moving_on || moving_off {
        return Err(anyhow!(
            "mount nudge read-back failed (on={moving_on}, off={moving_off})"
        ));
    }

    println!("[headless] OK — M1 pipeline verified end-to-end");
    Ok(())
}
