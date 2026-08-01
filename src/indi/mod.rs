//! Thin async wrappers over the `indi` crate for the camera and mount, plus session setup
//! and driver-agnostic device discovery.

pub mod camera;
pub mod mount;

use anyhow::{anyhow, Result};
use std::time::{Duration, Instant};
use tokio::net::TcpStream;
use tokio::time::sleep;

use indi::client::active_device::ActiveDevice;
use indi::client::Client;

use camera::Camera;
use mount::Mount;

/// Re-export of the external `indi` crate's `Parameter` so downstream code can name it as
/// `crate::indi::Parameter` without colliding with this local module.
pub use indi::Parameter;

/// INDI `DRIVER_INTERFACE` bitmask bits (see indiapi.h). Every device advertises the roles it
/// implements through the `DRIVER_INTERFACE` element of its standard `DRIVER_INFO` property.
/// We match on these bits rather than hard-coding device names, so the simulators
/// (`CCD Simulator`, `Telescope Simulator`), the PlayerOne CCD driver (which names the device
/// after the connected camera model, e.g. `PlayerOne Uranus-C`), and the LX200 OnStep mount
/// (`LX200 OnStep`) all work without configuration.
const TELESCOPE_INTERFACE: u32 = 1 << 0; // 1
const CCD_INTERFACE: u32 = 1 << 1; // 2

/// How long to wait for devices (and their `DRIVER_INFO`) to appear after requesting
/// properties. Real drivers can take a moment longer than the simulators.
const DISCOVERY_TIMEOUT: Duration = Duration::from_secs(10);

/// Once at least one device is present, how much longer to wait for the *other* kind before
/// giving up and connecting with what's available (e.g. camera up but mount driver not yet
/// running). Keeps a camera-only connect from stalling for the full [`DISCOVERY_TIMEOUT`].
const DISCOVERY_GRACE: Duration = Duration::from_secs(3);

/// A second INDI connection dedicated to the camera's video BLOBs. Isolating high-rate frame
/// data on its own socket (`BlobEnable::Only`) keeps it from starving control commands (stream
/// on/off, gain, mount nudges) on the main connection — which is set to `BlobEnable::Never`.
/// This is the standard INDI approach for live streaming; without it the shared connection's
/// device-store locks are held constantly by incoming blobs and control `change()`s time out.
struct BlobLink {
    /// Kept alive so `dev` stays valid; owns the blob connection's device store + sender.
    #[allow(dead_code)]
    client: Client,
    io: tokio::task::JoinHandle<()>,
    /// The camera device on the blob connection; the worker subscribes to CCD1 here.
    dev: ActiveDevice,
}

/// A live connection to an INDI server plus handles to the camera and mount devices.
pub struct Session {
    /// Currently-bound camera / mount. Either may be absent if that device type wasn't found
    /// at connect time; the UI device pickers can bind one later.
    pub camera: Option<Camera>,
    pub mount: Option<Mount>,
    /// Names of the currently-bound devices (empty if unbound), for display/logging.
    pub camera_name: String,
    pub mount_name: String,
    /// Server address, kept so a camera switch can rebuild the dedicated blob connection.
    addr: String,
    /// Kept alive so devices can be re-bound (see [`Session::select_camera`]); also owns the
    /// device store and the command sender.
    client: Client,
    /// Background I/O task driving the INDI connection; aborted on drop.
    io: tokio::task::JoinHandle<()>,
    /// Dedicated blob connection for the bound camera (absent if setup failed → frames fall
    /// back to the control connection).
    blob: Option<BlobLink>,
}

impl Drop for Session {
    fn drop(&mut self) {
        self.io.abort();
        if let Some(b) = &self.blob {
            b.io.abort();
        }
    }
}

/// Connect to `addr` (e.g. `127.0.0.1:7624`), request all properties, auto-discover the
/// camera (CCD) and mount (telescope) devices by their driver interface, then connect them.
pub async fn connect(addr: &str) -> Result<Session> {
    let (client, io) = open_connection(addr).await?;

    let (camera_pick, mount_pick) = discover(&client).await;
    if camera_pick.is_none() && mount_pick.is_none() {
        return Err(anyhow!(
            "no CCD or telescope device found within {DISCOVERY_TIMEOUT:?}; devices seen: {}",
            describe(&device_interfaces(&client).await),
        ));
    }
    tracing::info!(
        "discovered camera {:?}, mount {:?}",
        camera_pick,
        mount_pick
    );

    // Bind whichever devices were found; a missing one can be selected later via the UI.
    let mut camera = None;
    let mut camera_name = String::new();
    let mut blob = None;
    if let Some(name) = camera_pick {
        let dev = client
            .get_device(&name)
            .await
            .map_err(|e| anyhow!("getting camera '{name}': {e:?}"))?;
        let cam = Camera::new(dev);
        cam.connect().await?;
        blob = bind_blob(addr, &cam, &name).await;
        camera = Some(cam);
        camera_name = name;
    }

    let mut mount = None;
    let mut mount_name = String::new();
    if let Some(name) = mount_pick {
        let dev = client
            .get_device(&name)
            .await
            .map_err(|e| anyhow!("getting mount '{name}': {e:?}"))?;
        let m = Mount::new(dev);
        m.connect().await?;
        mount = Some(m);
        mount_name = name;
    }

    Ok(Session {
        camera,
        mount,
        camera_name,
        mount_name,
        addr: addr.to_string(),
        client,
        io,
        blob,
    })
}

/// Open an INDI connection to `addr`, spawn its I/O task, and request all properties (which
/// `indi::client::start` does not do on its own).
async fn open_connection(addr: &str) -> Result<(Client, tokio::task::JoinHandle<()>)> {
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let client = Client::new(Some(tx));
    let stream = TcpStream::connect(addr)
        .await
        .map_err(|e| anyhow!("connecting to indiserver at {addr}: {e}"))?;
    let io = tokio::spawn(indi::client::start(client.get_devices().clone(), rx, stream));
    client
        .send(indi::serialization::Command::GetProperties(
            indi::serialization::GetProperties {
                version: indi::INDI_PROTOCOL_VERSION.to_string(),
                device: None,
                name: None,
            },
        ))
        .map_err(|e| anyhow!("sending GetProperties: {e}"))?;
    Ok((client, io))
}

/// Set up the dedicated blob connection for `cam` (named `name`): open a second socket, bind
/// the same camera device on it with `BlobEnable::Only`, and flip the control connection to
/// `BlobEnable::Never`. On any failure, fall back to receiving blobs on the control connection
/// (`Also`) so streaming still works, just without the isolation — and return `None`.
async fn bind_blob(addr: &str, cam: &Camera, name: &str) -> Option<BlobLink> {
    match setup_blob_link(addr, name).await {
        Ok(link) => match cam.set_blob(indi::BlobEnable::Never).await {
            Ok(()) => Some(link),
            Err(e) => {
                tracing::warn!("blob isolation: control connection set_blob(Never) failed: {e:#}");
                link.io.abort();
                fallback_blob(cam).await;
                None
            }
        },
        Err(e) => {
            tracing::warn!(
                "dedicated blob connection failed ({e:#}); streaming over control connection"
            );
            fallback_blob(cam).await;
            None
        }
    }
}

/// Fallback when the dedicated blob connection can't be established: let blobs arrive on the
/// control connection (the previous single-connection behaviour).
async fn fallback_blob(cam: &Camera) {
    if let Err(e) = cam.set_blob(indi::BlobEnable::Also).await {
        tracing::warn!("blob fallback: control connection set_blob(Also) failed: {e:#}");
    }
}

/// Open a second connection and bind `name`'s camera device on it as blob-only.
async fn setup_blob_link(addr: &str, name: &str) -> Result<BlobLink> {
    let (client, io) = open_connection(addr).await?;
    let dev = match client.get_device(name).await {
        Ok(d) => d,
        Err(e) => {
            io.abort();
            return Err(anyhow!("blob connection getting camera '{name}': {e:?}"));
        }
    };
    if let Err(e) = dev.enable_blob(Some("CCD1"), indi::BlobEnable::Only).await {
        io.abort();
        return Err(anyhow!("blob connection enabling CCD1: {e:?}"));
    }
    Ok(BlobLink { client, io, dev })
}

impl Session {
    /// Names of all CCD-interface devices the server currently reports, sorted.
    pub async fn cameras(&self) -> Vec<String> {
        list_by_interface(&self.client, CCD_INTERFACE).await
    }

    /// Names of all telescope-interface devices the server currently reports, sorted.
    pub async fn mounts(&self) -> Vec<String> {
        list_by_interface(&self.client, TELESCOPE_INTERFACE).await
    }

    /// The device the worker should subscribe to for CCD1 frames: the dedicated blob connection
    /// when available, otherwise the control connection's camera device (fallback).
    pub fn frame_device(&self) -> Option<&ActiveDevice> {
        match &self.blob {
            Some(b) => Some(&b.dev),
            None => self.camera.as_ref().map(|c| &c.dev),
        }
    }

    /// Bind, connect, and switch to a different camera device by name. Stops the previously
    /// selected camera's stream first (best-effort); the caller must restart the frame task.
    pub async fn select_camera(&mut self, name: &str) -> Result<()> {
        if let Some(cam) = &self.camera {
            let _ = cam.stop_stream().await; // best-effort; may not be streaming
        }
        // Tear down the old camera's dedicated blob connection before rebinding.
        if let Some(b) = self.blob.take() {
            b.io.abort();
        }
        let dev = self
            .client
            .get_device(name)
            .await
            .map_err(|e| anyhow!("getting camera '{name}': {e:?}"))?;
        let camera = Camera::new(dev);
        camera.connect().await?;
        self.blob = bind_blob(&self.addr, &camera, name).await;
        self.camera = Some(camera);
        self.camera_name = name.to_string();
        Ok(())
    }

    /// A second, independent handle to the currently-bound mount device (for the guide loop /
    /// calibration task, which runs concurrently with the command loop). The device is already
    /// connected; this just gets another `ActiveDevice` into the same shared device store.
    pub async fn clone_mount(&self) -> Option<Mount> {
        if self.mount_name.is_empty() {
            return None;
        }
        let dev = self.client.get_device(&self.mount_name).await.ok()?;
        Some(Mount::new(dev))
    }

    /// Bind, connect, and switch to a different mount device by name.
    pub async fn select_mount(&mut self, name: &str) -> Result<()> {
        let dev = self
            .client
            .get_device(name)
            .await
            .map_err(|e| anyhow!("getting mount '{name}': {e:?}"))?;
        let mount = Mount::new(dev);
        mount.connect().await?;
        self.mount = Some(mount);
        self.mount_name = name.to_string();
        Ok(())
    }
}

/// All device names whose `DRIVER_INTERFACE` bitmask has `bit` set, sorted for stable order.
async fn list_by_interface(client: &indi::client::Client, bit: u32) -> Vec<String> {
    let mut names: Vec<String> = device_interfaces(client)
        .await
        .into_iter()
        .filter(|(_, iface)| iface & bit != 0)
        .map(|(name, _)| name)
        .collect();
    names.sort();
    names
}

/// Poll the device store for a camera (CCD interface) and a mount (telescope interface),
/// returning whichever were found. Waits up to [`DISCOVERY_TIMEOUT`] for both; but once one
/// kind has appeared, only waits [`DISCOVERY_GRACE`] longer for the other before returning
/// with what's available (so a camera-only setup connects promptly). Returns `(None, None)`
/// only if no device of either kind ever appeared.
async fn discover(client: &indi::client::Client) -> (Option<String>, Option<String>) {
    let deadline = Instant::now() + DISCOVERY_TIMEOUT;
    let mut first_seen: Option<Instant> = None;

    loop {
        let interfaces = device_interfaces(client).await;
        let camera = pick_camera(&interfaces);
        let mount = pick_mount(&interfaces);

        if camera.is_some() && mount.is_some() {
            tracing::debug!("discovered devices: {}", describe(&interfaces));
            return (camera, mount);
        }
        if camera.is_some() || mount.is_some() {
            // Give the other device type a short grace period, then proceed with what we have.
            let since = *first_seen.get_or_insert_with(Instant::now);
            if since.elapsed() >= DISCOVERY_GRACE {
                return (camera, mount);
            }
        }
        if Instant::now() >= deadline {
            return (camera, mount); // possibly (None, None)
        }
        sleep(Duration::from_millis(200)).await;
    }
}

/// Choose the mount among telescope-interface devices (first by sort order).
fn pick_mount(interfaces: &[(String, u32)]) -> Option<String> {
    let mut mounts: Vec<&String> = interfaces
        .iter()
        .filter(|(_, iface)| iface & TELESCOPE_INTERFACE != 0)
        .map(|(name, _)| name)
        .collect();
    mounts.sort();
    mounts.first().map(|n| (*n).clone())
}

/// Choose the imaging camera among CCD-interface devices. When several are present (e.g. the
/// test setup also runs a guide-camera simulator), prefer one whose name doesn't look like a
/// dedicated guider — this app guides off the main video stream, not a separate guide camera.
fn pick_camera(interfaces: &[(String, u32)]) -> Option<String> {
    let ccds: Vec<&String> = interfaces
        .iter()
        .filter(|(_, iface)| iface & CCD_INTERFACE != 0)
        .map(|(name, _)| name)
        .collect();
    ccds.iter()
        .find(|n| !n.to_lowercase().contains("guide"))
        .or_else(|| ccds.first())
        .map(|n| (*n).clone())
}

/// Snapshot each currently-known device's name and its `DRIVER_INTERFACE` bitmask (0 if the
/// device hasn't reported `DRIVER_INFO` yet).
async fn device_interfaces(client: &indi::client::Client) -> Vec<(String, u32)> {
    // Clone the per-device handles out of the store, then read each one without holding the
    // store lock across the inner awaits.
    let devices: Vec<(String, _)> = {
        let store = client.get_devices().read().await;
        store.iter().map(|(n, d)| (n.clone(), d.clone())).collect()
    };

    let mut out = Vec::with_capacity(devices.len());
    for (name, dev) in devices {
        let iface = {
            let device = dev.read().await;
            match device.get_parameters().get("DRIVER_INFO") {
                Some(param) => {
                    if let Parameter::TextVector(tv) = &*param.read().await {
                        tv.values
                            .get("DRIVER_INTERFACE")
                            .and_then(|t| parse_interface(&t.value))
                            .unwrap_or(0)
                    } else {
                        0
                    }
                }
                None => 0,
            }
        };
        out.push((name, iface));
    }
    out
}

/// Parse a `DRIVER_INTERFACE` value. Normally a plain integer string; tolerate a float form
/// just in case a driver reports it that way.
fn parse_interface(raw: &str) -> Option<u32> {
    let raw = raw.trim();
    raw.parse::<u32>()
        .ok()
        .or_else(|| raw.parse::<f64>().ok().map(|f| f as u32))
}

/// Human-readable device list for logs/errors.
fn describe(interfaces: &[(String, u32)]) -> String {
    if interfaces.is_empty() {
        return "(none)".to_string();
    }
    interfaces
        .iter()
        .map(|(n, i)| format!("{n} (iface {i:#x})"))
        .collect::<Vec<_>>()
        .join(", ")
}
