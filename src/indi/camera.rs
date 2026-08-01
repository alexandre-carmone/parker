//! Camera (CCD) control: connection, video streaming, and exposure/gain.

use anyhow::{anyhow, Result};
use indi::client::active_device::ActiveDevice;
use indi::serialization::Sexagesimal;
use indi::Parameter;

/// Wrapper around the CCD `ActiveDevice` exposing the operations M1 needs.
pub struct Camera {
    /// Underlying device handle; the worker uses it directly to subscribe to CCD1 frames.
    pub dev: ActiveDevice,
}

impl Camera {
    pub fn new(dev: ActiveDevice) -> Self {
        Camera { dev }
    }

    /// Connect the device and prefer the MJPEG encoder. BLOB transport is enabled separately
    /// via [`Camera::set_blob`] so the session can route frames onto a dedicated connection.
    pub async fn connect(&self) -> Result<()> {
        let _ = self
            .dev
            .change("CONNECTION", vec![("CONNECT", true)])
            .await
            .map_err(|e| anyhow!("connecting camera: {e:?}"))?;
        // MJPEG frames are the simplest to decode; ignore failure (some drivers lack it).
        if let Err(e) = self
            .dev
            .change("CCD_STREAM_ENCODER", vec![("MJPEG", true)])
            .await
        {
            tracing::warn!("could not select MJPEG encoder: {e:?}");
        }
        Ok(())
    }

    /// Set this connection's CCD1 BLOB transport policy (`Never`/`Also`/`Only`). BLOB enable is
    /// per-connection server state, which is what lets us isolate frame data on its own socket.
    pub async fn set_blob(&self, enabled: indi::BlobEnable) -> Result<()> {
        self.dev
            .enable_blob(Some("CCD1"), enabled)
            .await
            .map_err(|e| anyhow!("setting BLOB transport: {e:?}"))?;
        Ok(())
    }

    pub async fn start_stream(&self) -> Result<()> {
        self.toggle_stream("STREAM_ON").await
    }

    pub async fn stop_stream(&self) -> Result<()> {
        self.toggle_stream("STREAM_OFF").await
    }

    /// Fire-and-forget a `CCD_VIDEO_STREAM` switch element (`STREAM_ON`/`STREAM_OFF`).
    ///
    /// This deliberately uses `set` (send-and-return) rather than `change` (send-and-wait):
    /// a running stream holds `CCD_VIDEO_STREAM` in the `Busy` state for as long as it runs,
    /// so `change` — which waits for the property to leave `Busy` — always times out on
    /// `STREAM_ON` even though the toggle took effect. Blocking here would also stall the
    /// worker's command loop for the whole timeout. Frames arriving on CCD1 are the real
    /// confirmation that the stream is live.
    async fn toggle_stream(&self, element: &str) -> Result<()> {
        self.dev
            .parameter("CCD_VIDEO_STREAM")
            .await
            .map_err(|e| anyhow!("finding CCD_VIDEO_STREAM: {e:?}"))?
            .set(vec![(element, true)])
            .map_err(|e| anyhow!("toggling video stream ({element}): {e:?}"))?;
        Ok(())
    }

    pub async fn set_gain(&self, gain: f64) -> Result<()> {
        let _ = self
            .dev
            .change("CCD_GAIN", vec![("GAIN", Sexagesimal::from(gain))])
            .await
            .map_err(|e| anyhow!("setting gain: {e:?}"))?;
        Ok(())
    }

    /// Set the still-frame exposure in seconds (also used as the streaming exposure hint).
    pub async fn set_exposure(&self, seconds: f64) -> Result<()> {
        let _ = self
            .dev
            .change(
                "CCD_EXPOSURE",
                vec![("CCD_EXPOSURE_VALUE", Sexagesimal::from(seconds))],
            )
            .await
            .map_err(|e| anyhow!("setting exposure: {e:?}"))?;
        Ok(())
    }

    /// Set the CCD readout region (subframe / ROI) via `CCD_FRAME`, in sensor pixels. All four
    /// elements are sent together so the driver applies a consistent rectangle. Resetting to the
    /// full sensor is just `set_frame(0, 0, max_x, max_y)` — this driver has no `CCD_FRAME_RESET`.
    pub async fn set_frame(&self, x: u32, y: u32, w: u32, h: u32) -> Result<()> {
        let _ = self
            .dev
            .change(
                "CCD_FRAME",
                vec![
                    ("X", Sexagesimal::from(x as f64)),
                    ("Y", Sexagesimal::from(y as f64)),
                    ("WIDTH", Sexagesimal::from(w as f64)),
                    ("HEIGHT", Sexagesimal::from(h as f64)),
                ],
            )
            .await
            .map_err(|e| anyhow!("setting CCD frame: {e:?}"))?;
        Ok(())
    }

    /// Read the full sensor size (`CCD_INFO` → `CCD_MAX_X`/`CCD_MAX_Y`) in pixels. Used to bound
    /// the ROI controls and to reset the subframe to full.
    pub async fn sensor_size(&self) -> Result<(u32, u32)> {
        let param = self
            .dev
            .get_parameter("CCD_INFO")
            .await
            .map_err(|e| anyhow!("getting CCD_INFO: {e:?}"))?;
        let guard = param.read().await;
        if let Parameter::NumberVector(nv) = &*guard {
            let get = |name: &str| -> Option<u32> {
                nv.values.get(name).map(|n| f64::from(n.value) as u32)
            };
            let w = get("CCD_MAX_X").ok_or_else(|| anyhow!("CCD_INFO missing CCD_MAX_X"))?;
            let h = get("CCD_MAX_Y").ok_or_else(|| anyhow!("CCD_INFO missing CCD_MAX_Y"))?;
            Ok((w, h))
        } else {
            Err(anyhow!("CCD_INFO is not a number vector"))
        }
    }
}
