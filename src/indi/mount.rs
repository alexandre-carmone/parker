//! Mount (telescope) control: connection, manual nudge, slew rate, tracking, and (M2)
//! timed pulse-guiding.

use anyhow::{anyhow, Result};
use indi::client::active_device::ActiveDevice;
use indi::serialization::Sexagesimal;
use indi::{Parameter, SwitchState};

use crate::bus::Dir;

/// Wrapper around the telescope `ActiveDevice`.
pub struct Mount {
    pub dev: ActiveDevice,
}

/// Map a nudge direction to its `TELESCOPE_MOTION_*` switch property + element.
fn motion_target(dir: Dir) -> (&'static str, &'static str) {
    match dir {
        Dir::North => ("TELESCOPE_MOTION_NS", "MOTION_NORTH"),
        Dir::South => ("TELESCOPE_MOTION_NS", "MOTION_SOUTH"),
        Dir::East => ("TELESCOPE_MOTION_WE", "MOTION_EAST"),
        Dir::West => ("TELESCOPE_MOTION_WE", "MOTION_WEST"),
    }
}

/// Map a pulse-guide direction to its `TELESCOPE_TIMED_GUIDE_*` number property + element.
fn guide_target(dir: Dir) -> (&'static str, &'static str) {
    match dir {
        Dir::North => ("TELESCOPE_TIMED_GUIDE_NS", "TIMED_GUIDE_N"),
        Dir::South => ("TELESCOPE_TIMED_GUIDE_NS", "TIMED_GUIDE_S"),
        Dir::East => ("TELESCOPE_TIMED_GUIDE_WE", "TIMED_GUIDE_E"),
        Dir::West => ("TELESCOPE_TIMED_GUIDE_WE", "TIMED_GUIDE_W"),
    }
}

impl Mount {
    pub fn new(dev: ActiveDevice) -> Self {
        Mount { dev }
    }

    pub async fn connect(&self) -> Result<()> {
        let _ = self
            .dev
            .change("CONNECTION", vec![("CONNECT", true)])
            .await
            .map_err(|e| anyhow!("connecting mount: {e:?}"))?;
        Ok(())
    }

    /// Read the driver's `TELESCOPE_SLEW_RATE` switch element names, sorted for stable order.
    pub async fn slew_rates(&self) -> Result<Vec<String>> {
        let param = self
            .dev
            .get_parameter("TELESCOPE_SLEW_RATE")
            .await
            .map_err(|e| anyhow!("getting slew rates: {e:?}"))?;
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            let mut names: Vec<String> = sv.values.keys().cloned().collect();
            names.sort();
            Ok(names)
        } else {
            Err(anyhow!("TELESCOPE_SLEW_RATE is not a switch vector"))
        }
    }

    /// Read whether the mount is currently commanded to move in `dir` (reads the
    /// `TELESCOPE_MOTION_*` switch state). Used by tests to verify nudge commands took effect.
    pub async fn is_moving(&self, dir: Dir) -> Result<bool> {
        let (param_name, elem) = motion_target(dir);
        let param = self
            .dev
            .get_parameter(param_name)
            .await
            .map_err(|e| anyhow!("reading {param_name}: {e:?}"))?;
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            Ok(sv
                .values
                .get(elem)
                .map(|s| s.value == SwitchState::On)
                .unwrap_or(false))
        } else {
            Err(anyhow!("{param_name} is not a switch vector"))
        }
    }

    /// Start (`active`) or stop a manual slew in `dir`.
    pub async fn nudge(&self, dir: Dir, active: bool) -> Result<()> {
        let (param, elem) = motion_target(dir);
        let _ = self
            .dev
            .change(param, vec![(elem, active)])
            .await
            .map_err(|e| anyhow!("nudging {dir:?}: {e:?}"))?;
        Ok(())
    }

    pub async fn set_slew_rate(&self, name: &str) -> Result<()> {
        let _ = self
            .dev
            .change("TELESCOPE_SLEW_RATE", vec![(name, true)])
            .await
            .map_err(|e| anyhow!("setting slew rate {name}: {e:?}"))?;
        Ok(())
    }

    pub async fn set_tracking(&self, on: bool) -> Result<()> {
        let elem = if on { "TRACK_ON" } else { "TRACK_OFF" };
        let _ = self
            .dev
            .change("TELESCOPE_TRACK_STATE", vec![(elem, true)])
            .await
            .map_err(|e| anyhow!("setting tracking: {e:?}"))?;
        Ok(())
    }

    pub async fn abort(&self) -> Result<()> {
        let _ = self
            .dev
            .change("TELESCOPE_ABORT_MOTION", vec![("ABORT", true)])
            .await
            .map_err(|e| anyhow!("aborting motion: {e:?}"))?;
        Ok(())
    }

    /// Issue a timed guide pulse of `millis` in `dir` (used by the M2 guiding loop).
    ///
    /// Fire-and-forget via `set`, not `change`: a timed guide pulse holds the property `Busy`
    /// for its whole duration and its element value counts *down* from the requested time, so
    /// `change` — which waits for the property to settle back to the value we sent — never
    /// completes (the same reason [`crate::indi::camera::Camera::toggle_stream`] uses `set`).
    /// The caller is responsible for spacing pulses so they don't overlap.
    pub async fn pulse_guide(&self, dir: Dir, millis: f64) -> Result<()> {
        let (param, elem) = guide_target(dir);
        self.dev
            .parameter(param)
            .await
            .map_err(|e| anyhow!("finding {param}: {e:?}"))?
            .set(vec![(elem, Sexagesimal::from(millis))])
            .map_err(|e| anyhow!("pulse guide {dir:?}: {e:?}"))?;
        Ok(())
    }
}
