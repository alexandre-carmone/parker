//! Camera (CCD) control: connection, video streaming, and exposure/gain.

use anyhow::{anyhow, Result};
use indi::client::active_device::ActiveDevice;
use indi::serialization::Sexagesimal;
use indi::{Parameter, SwitchState};

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

    /// Turn on `elem` in the switch vector `prop`. Used to drive the driver's stream-format
    /// switches — which govern the streamed bit depth and are driver-specific. On the Player One
    /// cameras the ones that matter for 16-bit recording are `CCD_STREAM_ENCODER` (`RAW`),
    /// `CCD_VIDEO_FORMAT` (`POA_RAW16`), and `STREAM_FULL_DEPTH` (`FULL_DEPTH_16BIT`); `_8BIT`
    /// there downsamples the client stream even when the sensor format is 16-bit.
    pub async fn set_switch(&self, prop: &str, elem: &str) -> Result<()> {
        let _ = self
            .dev
            .change(prop, vec![(elem, true)])
            .await
            .map_err(|e| anyhow!("setting {prop}={elem}: {e:?}"))?;
        Ok(())
    }

    /// Read a switch vector's element names (sorted) and the currently-selected one. Used to
    /// populate the encoder / capture-format pickers. Returns an empty list if the driver lacks
    /// the property.
    pub async fn switch_options(&self, prop: &str) -> (Vec<String>, Option<String>) {
        let param = match self.dev.get_parameter(prop).await {
            Ok(p) => p,
            Err(_) => return (Vec::new(), None),
        };
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            let mut names: Vec<String> = sv.values.keys().cloned().collect();
            names.sort();
            let selected = sv
                .values
                .iter()
                .find(|(_, s)| s.value == SwitchState::On)
                .map(|(k, _)| k.clone());
            (names, selected)
        } else {
            (Vec::new(), None)
        }
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
        self.toggle_stream("STREAM_ON").await?;
        // Lift the driver's preview-FPS throttle on every start (incl. the restart after an ROI
        // change) — the driver resets it, and without this it caps delivery to ~10fps. Best-effort.
        match self.set_preview_fps_limit_to_max().await {
            Ok(max) => tracing::info!("preview-fps limit raised to {max}"),
            Err(e) => tracing::debug!("preview-fps limit: {e}"),
        }
        Ok(())
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

    /// Set the still-frame exposure in seconds (`CCD_EXPOSURE`).
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

    /// Set the **live-stream** frame exposure (`STREAMING_EXPOSURE`), which governs the stream
    /// frame rate on cameras that separate it from the still exposure (e.g. Player One, where the
    /// still `CCD_EXPOSURE` does not affect streaming). Best-effort — not all drivers expose it.
    pub async fn set_streaming_exposure(&self, seconds: f64) -> Result<()> {
        let _ = self
            .dev
            .change(
                "STREAMING_EXPOSURE",
                vec![("STREAMING_EXPOSURE_VALUE", Sexagesimal::from(seconds))],
            )
            .await
            .map_err(|e| anyhow!("setting streaming exposure: {e:?}"))?;
        Ok(())
    }

    /// Set the driver's frame-count budget (`RECORD_OPTIONS.RECORD_FRAME_TOTAL`) for a subsequent
    /// [`record_start_frames`](Self::record_start_frames). Sends **only** this element: including
    /// the sibling `RECORD_DURATION` (e.g. as 0) makes drivers reject the whole vector as
    /// out-of-range, flipping `RECORD_OPTIONS` to `Alert` — surfaced by the crate as a
    /// `PropertyError`.
    pub async fn set_record_frame_total(&self, frames: u64) -> Result<()> {
        let _ = self
            .dev
            .change(
                "RECORD_OPTIONS",
                vec![("RECORD_FRAME_TOTAL", Sexagesimal::from(frames as f64))],
            )
            .await
            .map_err(|e| anyhow!("setting record frame total: {e:?}"))?;
        Ok(())
    }

    /// Set the driver's duration budget (`RECORD_OPTIONS.RECORD_DURATION`, seconds) for a
    /// subsequent [`record_start_duration`](Self::record_start_duration). Sends only this element,
    /// for the same reason as [`set_record_frame_total`](Self::set_record_frame_total).
    pub async fn set_record_duration(&self, seconds: f64) -> Result<()> {
        let _ = self
            .dev
            .change("RECORD_OPTIONS", vec![("RECORD_DURATION", Sexagesimal::from(seconds))])
            .await
            .map_err(|e| anyhow!("setting record duration: {e:?}"))?;
        Ok(())
    }

    /// Set where the driver writes the recording (`RECORD_FILE`). The path is on the **indiserver
    /// host**, not the client. `name` may use the driver's filename templates (`_D_` date,
    /// `_T_` time). The extension (`.ser`) is added by the driver.
    pub async fn set_record_file(&self, dir: &str, name: &str) -> Result<()> {
        let _ = self
            .dev
            .change("RECORD_FILE", vec![("RECORD_FILE_DIR", dir), ("RECORD_FILE_NAME", name)])
            .await
            .map_err(|e| anyhow!("setting record file: {e:?}"))?;
        Ok(())
    }

    /// Start a frame-count-limited driver recording (`RECORD_STREAM.RECORD_FRAME_ON`), stopping
    /// after `RECORD_FRAME_TOTAL` frames. See [`toggle_record`](Self::toggle_record) for why this
    /// is fire-and-forget.
    pub async fn record_start_frames(&self) -> Result<()> {
        self.toggle_record("RECORD_FRAME_ON").await
    }

    /// Start a duration-limited driver recording (`RECORD_STREAM.RECORD_DURATION_ON`), stopping
    /// after `RECORD_DURATION` seconds.
    pub async fn record_start_duration(&self) -> Result<()> {
        self.toggle_record("RECORD_DURATION_ON").await
    }

    /// Stop the driver recording (`RECORD_STREAM.RECORD_OFF`). Best-effort.
    pub async fn record_stop(&self) -> Result<()> {
        self.toggle_record("RECORD_OFF").await
    }

    /// Fire-and-forget a `RECORD_STREAM` switch element. Uses `set` (send-and-return) rather than
    /// `change` (send-and-wait) for the same reason as [`toggle_stream`](Self::toggle_stream): an
    /// active recording holds `RECORD_STREAM` in `Busy` for its whole duration, so `change` — which
    /// waits for the property to leave `Busy` — would time out even though the switch took effect.
    async fn toggle_record(&self, element: &str) -> Result<()> {
        self.dev
            .parameter("RECORD_STREAM")
            .await
            .map_err(|e| anyhow!("finding RECORD_STREAM: {e:?}"))?
            .set(vec![(element, true)])
            .map_err(|e| anyhow!("toggling record ({element}): {e:?}"))?;
        Ok(())
    }

    /// Whether the driver is currently recording, read from `RECORD_STREAM`: true while
    /// `RECORD_OFF` is `Off`. Used to detect the driver's auto-stop on the frame/duration budget.
    /// Returns `false` if the property is absent or unreadable.
    pub async fn record_is_running(&self) -> bool {
        let Ok(param) = self.dev.get_parameter("RECORD_STREAM").await else {
            return false;
        };
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            sv.values
                .get("RECORD_OFF")
                .map(|s| s.value == SwitchState::Off)
                .unwrap_or(false)
        } else {
            false
        }
    }

    /// Raise the driver's client preview-FPS cap (`LIMITS.LIMITS_PREVIEW_FPS`) to its maximum so
    /// every captured frame is delivered. Drivers often default this to ~10, which throttles both
    /// the live view and recording. We set the property's own advertised max (rather than an
    /// arbitrary large number, which some drivers reject as out-of-range). Returns the value set;
    /// best-effort at the call site — not all drivers expose it.
    pub async fn set_preview_fps_limit_to_max(&self) -> Result<f64> {
        let param = self
            .dev
            .get_parameter("LIMITS")
            .await
            .map_err(|e| anyhow!("getting LIMITS: {e:?}"))?;
        let max = {
            let guard = param.read().await;
            if let Parameter::NumberVector(nv) = &*guard {
                nv.values
                    .get("LIMITS_PREVIEW_FPS")
                    .map(|n| f64::from(n.max))
            } else {
                None
            }
        };
        let max = max.ok_or_else(|| anyhow!("LIMITS_PREVIEW_FPS not found"))?;
        let _ = self
            .dev
            .change("LIMITS", vec![("LIMITS_PREVIEW_FPS", Sexagesimal::from(max))])
            .await
            .map_err(|e| anyhow!("setting preview fps limit to {max}: {e:?}"))?;
        Ok(max)
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

    /// Read the readout region the driver actually applied, as `(x, y, w, h)`. Drivers snap the
    /// requested width/height to hardware alignment (e.g. the Player One rounds width down to a
    /// multiple of 8), so the streamed frame size can differ from what we asked for — we must use
    /// the applied dimensions to interpret raw stream frames.
    ///
    /// The size comes from `CCD_STREAM_FRAME` (the actual streamed frame — the snapped WIDTH lives
    /// here, while `CCD_FRAME` may still echo the requested value); the sensor offset comes from
    /// `CCD_FRAME` (`CCD_STREAM_FRAME` reports a stream-relative `0,0`). Falls back to whichever is
    /// available.
    pub async fn read_applied_roi(&self) -> Result<(u32, u32, u32, u32)> {
        let stream = self.read_frame_prop("CCD_STREAM_FRAME").await;
        let frame = self.read_frame_prop("CCD_FRAME").await;
        match (stream, frame) {
            (Some((_, _, w, h)), Some((fx, fy, _, _))) => Ok((fx, fy, w, h)),
            (Some(s), None) => Ok(s),
            (None, Some(f)) => Ok(f),
            (None, None) => Err(anyhow!("no applied ROI available (CCD_STREAM_FRAME/CCD_FRAME)")),
        }
    }

    /// Read a `CCD_FRAME`-shaped number vector (`X`/`Y`/`WIDTH`/`HEIGHT`) as `(x, y, w, h)`.
    /// `None` if the property is absent, not a number vector, or reports a zero size.
    async fn read_frame_prop(&self, prop: &str) -> Option<(u32, u32, u32, u32)> {
        let param = self.dev.get_parameter(prop).await.ok()?;
        let guard = param.read().await;
        if let Parameter::NumberVector(nv) = &*guard {
            let get = |name: &str| nv.values.get(name).map(|n| f64::from(n.value) as u32);
            let (w, h) = (get("WIDTH")?, get("HEIGHT")?);
            if w == 0 || h == 0 {
                return None;
            }
            Some((get("X").unwrap_or(0), get("Y").unwrap_or(0), w, h))
        } else {
            None
        }
    }

    /// Read a single number element as `(value, min, max)` in its native units. Used to log the
    /// value the driver actually holds after a change, plus its valid range — so a setting the
    /// driver silently clamped or rejected (out of range) is visible. `None` if the property or
    /// element is absent, or the property is not a number vector.
    pub async fn number_range(&self, prop: &str, elem: &str) -> Option<(f64, f64, f64)> {
        let param = self.dev.get_parameter(prop).await.ok()?;
        let guard = param.read().await;
        if let Parameter::NumberVector(nv) = &*guard {
            let n = nv.values.get(elem)?;
            Some((f64::from(n.value), n.min, n.max))
        } else {
            None
        }
    }

    /// Read the driver's own streaming frame-rate estimate from the standard INDI `FPS` property
    /// (published by indibase's stream manager). Prefers `AVG_FPS` — the driver's 1-second rolling
    /// average, which is steady — and falls back to the instantaneous `EST_FPS`. `None` if the
    /// driver doesn't publish `FPS` (not all do) or it isn't a number vector.
    pub async fn stream_fps(&self) -> Option<f64> {
        let param = self.dev.get_parameter("FPS").await.ok()?;
        let guard = param.read().await;
        if let Parameter::NumberVector(nv) = &*guard {
            nv.values
                .get("AVG_FPS")
                .or_else(|| nv.values.get("EST_FPS"))
                .map(|n| f64::from(n.value))
        } else {
            None
        }
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
