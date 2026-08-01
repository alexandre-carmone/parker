//! Thin async wrappers over the `indi` crate for the camera and mount, plus session setup.

pub mod camera;
pub mod mount;

use anyhow::{anyhow, Result};
use std::time::Duration;
use tokio::net::TcpStream;
use tokio::time::timeout;

use camera::Camera;
use mount::Mount;

/// Re-export of the external `indi` crate's `Parameter` so downstream code can name it as
/// `crate::indi::Parameter` without colliding with this local module.
pub use indi::Parameter;

/// INDI device names exposed by the simulators (and by matching real drivers).
pub const CCD_DEVICE: &str = "CCD Simulator";
pub const MOUNT_DEVICE: &str = "Telescope Simulator";

/// A live connection to an INDI server plus handles to the camera and mount devices.
pub struct Session {
    pub camera: Camera,
    pub mount: Mount,
    /// Background I/O task driving the INDI connection; aborted on drop.
    io: tokio::task::JoinHandle<()>,
}

impl Drop for Session {
    fn drop(&mut self) {
        self.io.abort();
    }
}

/// Connect to `addr` (e.g. `127.0.0.1:7624`), request all properties, and grab + connect
/// the camera and mount devices.
pub async fn connect(addr: &str) -> Result<Session> {
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let client = indi::client::Client::new(Some(tx));
    let stream = TcpStream::connect(addr)
        .await
        .map_err(|e| anyhow!("connecting to indiserver at {addr}: {e}"))?;
    let io = tokio::spawn(indi::client::start(
        client.get_devices().clone(),
        rx,
        stream,
    ));

    // start() does not request properties; without this no devices ever appear.
    client
        .send(indi::serialization::Command::GetProperties(
            indi::serialization::GetProperties {
                version: indi::INDI_PROTOCOL_VERSION.to_string(),
                device: None,
                name: None,
            },
        ))
        .map_err(|e| anyhow!("sending GetProperties: {e}"))?;

    let cam_dev = timeout(Duration::from_secs(5), client.get_device(CCD_DEVICE))
        .await
        .map_err(|_| anyhow!("timed out waiting for '{CCD_DEVICE}'"))?
        .map_err(|e| anyhow!("getting '{CCD_DEVICE}': {e:?}"))?;
    let mount_dev = timeout(Duration::from_secs(5), client.get_device(MOUNT_DEVICE))
        .await
        .map_err(|_| anyhow!("timed out waiting for '{MOUNT_DEVICE}'"))?
        .map_err(|e| anyhow!("getting '{MOUNT_DEVICE}': {e:?}"))?;

    let camera = Camera::new(cam_dev);
    let mount = Mount::new(mount_dev);
    camera.connect().await?;
    mount.connect().await?;

    Ok(Session { camera, mount, io })
}
