//! Camera (CCD) control: connection, video streaming, and exposure/gain.

use anyhow::{anyhow, Result};
use indi::client::active_device::ActiveDevice;
use indi::serialization::Sexagesimal;

/// Wrapper around the CCD `ActiveDevice` exposing the operations M1 needs.
pub struct Camera {
    /// Underlying device handle; the worker uses it directly to subscribe to CCD1 frames.
    pub dev: ActiveDevice,
}

impl Camera {
    pub fn new(dev: ActiveDevice) -> Self {
        Camera { dev }
    }

    /// Connect the device, enable BLOB transport for CCD1, and prefer the MJPEG encoder.
    pub async fn connect(&self) -> Result<()> {
        let _ = self
            .dev
            .change("CONNECTION", vec![("CONNECT", true)])
            .await
            .map_err(|e| anyhow!("connecting camera: {e:?}"))?;
        self.dev
            .enable_blob(Some("CCD1"), indi::BlobEnable::Also)
            .await
            .map_err(|e| anyhow!("enabling BLOB transport: {e:?}"))?;
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

    pub async fn start_stream(&self) -> Result<()> {
        let _ = self
            .dev
            .change("CCD_VIDEO_STREAM", vec![("STREAM_ON", true)])
            .await
            .map_err(|e| anyhow!("starting video stream: {e:?}"))?;
        Ok(())
    }

    pub async fn stop_stream(&self) -> Result<()> {
        let _ = self
            .dev
            .change("CCD_VIDEO_STREAM", vec![("STREAM_OFF", true)])
            .await
            .map_err(|e| anyhow!("stopping video stream: {e:?}"))?;
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
}
