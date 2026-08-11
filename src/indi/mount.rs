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

/// Extract the first numeric run (digits + `.`) from a label, e.g. `"x0.25"` → `Some(0.25)`.
fn leading_number(label: &str) -> Option<f64> {
    let mut num = String::new();
    let mut started = false;
    for c in label.chars() {
        if c.is_ascii_digit() || (c == '.' && started) {
            num.push(c);
            started = true;
        } else if started {
            break;
        }
    }
    num.parse().ok()
}

/// Best-effort "slow→fast" ordering of two switch labels. Labels with an embedded number sort
/// first, by value (`x0.25 < x0.5 < x1 < x2 < x10`); labels without one (Guide, Max, VVF…)
/// sort after, alphabetically. `HashMap` loses the driver's XML order, so this reconstructs a
/// sensible one without hard-coding driver-specific element names.
fn cmp_switch_label(a: &str, b: &str) -> std::cmp::Ordering {
    match (leading_number(a), leading_number(b)) {
        (Some(x), Some(y)) => x.partial_cmp(&y).unwrap_or(std::cmp::Ordering::Equal),
        (Some(_), None) => std::cmp::Ordering::Less,
        (None, Some(_)) => std::cmp::Ordering::Greater,
        (None, None) => a.cmp(b),
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

    /// Read a `OneOfMany` switch vector as an ordered list of `(element name, display label)`
    /// pairs plus the index of the currently-selected (`On`) element. Ordered slow→fast via
    /// [`cmp_switch_label`] since the driver's XML order isn't preserved by the `HashMap`.
    async fn read_switch_options(&self, prop: &str) -> Result<(Vec<(String, String)>, usize)> {
        let param = self
            .dev
            .get_parameter(prop)
            .await
            .map_err(|e| anyhow!("getting {prop}: {e:?}"))?;
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            let mut opts: Vec<(String, String, bool)> = sv
                .values
                .iter()
                .map(|(name, sw)| {
                    (
                        name.clone(),
                        sw.label.clone().unwrap_or_else(|| name.clone()),
                        sw.value == SwitchState::On,
                    )
                })
                .collect();
            opts.sort_by(|a, b| cmp_switch_label(&a.1, &b.1));
            let selected = opts.iter().position(|(_, _, on)| *on).unwrap_or(0);
            let pairs = opts.into_iter().map(|(n, l, _)| (n, l)).collect();
            Ok((pairs, selected))
        } else {
            Err(anyhow!("{prop} is not a switch vector"))
        }
    }

    /// Read `TELESCOPE_SLEW_RATE` as `(element name, label)` pairs plus the selected index.
    pub async fn slew_rates(&self) -> Result<(Vec<(String, String)>, usize)> {
        self.read_switch_options("TELESCOPE_SLEW_RATE").await
    }

    /// Read `TELESCOPE_TRACK_MODE` (Sidereal/Solar/Lunar/Custom) as `(element name, label)` pairs
    /// plus the selected index. Not all mounts expose it.
    pub async fn track_modes(&self) -> Result<(Vec<(String, String)>, usize)> {
        self.read_switch_options("TELESCOPE_TRACK_MODE").await
    }

    /// Read whether tracking is currently on (`TELESCOPE_TRACK_STATE` `TRACK_ON`).
    pub async fn tracking_on(&self) -> Result<bool> {
        let param = self
            .dev
            .get_parameter("TELESCOPE_TRACK_STATE")
            .await
            .map_err(|e| anyhow!("reading track state: {e:?}"))?;
        let guard = param.read().await;
        if let Parameter::SwitchVector(sv) = &*guard {
            Ok(sv
                .values
                .get("TRACK_ON")
                .map(|s| s.value == SwitchState::On)
                .unwrap_or(false))
        } else {
            Err(anyhow!("TELESCOPE_TRACK_STATE is not a switch vector"))
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

    pub async fn set_track_mode(&self, name: &str) -> Result<()> {
        let _ = self
            .dev
            .change("TELESCOPE_TRACK_MODE", vec![(name, true)])
            .await
            .map_err(|e| anyhow!("setting track mode {name}: {e:?}"))?;
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
