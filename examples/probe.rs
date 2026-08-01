//! Step 0 connectivity spike.
//!
//! Validates the `indi` crate API against the simulators:
//!   1. connect to indiserver on 127.0.0.1:7624
//!   2. connect the "CCD Simulator" device
//!   3. enable BLOB transport for CCD1
//!   4. select the MJPEG stream encoder and start CCD_VIDEO_STREAM
//!   5. receive a handful of frames and print their format + byte size
//!
//! Run the simulators first:
//!   indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
//! then:
//!   cargo run --example probe

use std::time::Duration;

use anyhow::{anyhow, Result};
use tokio::net::TcpStream;
use tokio::time::timeout;
use tokio_stream::StreamExt;

const INDI_ADDR: &str = "127.0.0.1:7624";
const CCD: &str = "CCD Simulator";

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    // 1. Client + background I/O task.
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let client = indi::client::Client::new(Some(tx));
    let connection = TcpStream::connect(INDI_ADDR)
        .await
        .map_err(|e| anyhow!("connecting to indiserver at {INDI_ADDR}: {e}"))?;
    let _io = tokio::spawn(indi::client::start(
        client.get_devices().clone(),
        rx,
        connection,
    ));

    // Ask the server to define all devices/properties (start() does not do this for us).
    client
        .send(indi::serialization::Command::GetProperties(
            indi::serialization::GetProperties {
                version: indi::INDI_PROTOCOL_VERSION.to_string(),
                device: None,
                name: None,
            },
        ))
        .map_err(|e| anyhow!("sending GetProperties: {e}"))?;

    // 2. Grab the camera device and make sure it is connected.
    let camera = timeout(Duration::from_secs(5), client.get_device(CCD))
        .await
        .map_err(|_| anyhow!("timed out waiting for device '{CCD}' to appear"))?
        .map_err(|e| anyhow!("getting device '{CCD}': {e:?}"))?;
    println!("[probe] got device: {CCD}");

    // change() returns a stream for awaiting confirmation; we only need the command sent.
    let _ = camera
        .change("CONNECTION", vec![("CONNECT", true)])
        .await
        .map_err(|e| anyhow!("connecting camera: {e:?}"))?;
    println!("[probe] camera connected");

    // 3. Enable BLOB transport for the CCD1 image property.
    camera
        .enable_blob(Some("CCD1"), indi::BlobEnable::Also)
        .await
        .map_err(|e| anyhow!("enabling BLOB transport: {e:?}"))?;
    println!("[probe] BLOB transport enabled for CCD1");

    // Subscribe to CCD1 changes *before* starting the stream so we don't miss frames.
    let ccd1 = timeout(Duration::from_secs(5), camera.get_parameter("CCD1"))
        .await
        .map_err(|_| anyhow!("timed out getting CCD1 parameter"))?
        .map_err(|e| anyhow!("getting CCD1 parameter: {e:?}"))?;
    let mut frames = ccd1.changes();

    // 4. Prefer the MJPEG encoder (easy to decode) then start the video stream.
    if let Err(e) = camera
        .change("CCD_STREAM_ENCODER", vec![("MJPEG", true)])
        .await
    {
        println!("[probe] note: could not select MJPEG encoder ({e:?}); using default");
    }
    let _ = camera
        .change("CCD_VIDEO_STREAM", vec![("STREAM_ON", true)])
        .await
        .map_err(|e| anyhow!("starting video stream: {e:?}"))?;
    println!("[probe] CCD_VIDEO_STREAM = ON; waiting for frames...");

    // 5. Receive a few frames and report format + size.
    let mut received = 0usize;
    while received < 5 {
        let next = timeout(Duration::from_secs(10), frames.next())
            .await
            .map_err(|_| anyhow!("timed out waiting for a stream frame"))?;

        let Some(update) = next else {
            return Err(anyhow!("CCD1 change stream ended unexpectedly"));
        };
        let param = update.map_err(|e| anyhow!("frame subscription error: {e:?}"))?;

        if let indi::Parameter::BlobVector(bv) = param.as_ref() {
            if let Some(blob) = bv.values.get("CCD1") {
                let fmt = blob.format.as_deref().unwrap_or("<none>");
                let len = blob.value.as_ref().map(|v| v.len()).unwrap_or(0);
                // Only count actual data-bearing frames (defs arrive with no value).
                if len > 0 {
                    received += 1;
                    println!("[probe] frame #{received}: format={fmt} bytes={len}");
                }
            }
        }
    }

    // Stop the stream so we leave the simulator in a clean state.
    let _ = camera
        .change("CCD_VIDEO_STREAM", vec![("STREAM_OFF", true)])
        .await;
    println!("[probe] OK — indi crate API validated (stream stopped)");
    Ok(())
}
