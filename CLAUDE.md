# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`solar` is a Rust desktop app for **solar and planetary imaging** driven by INDI. It gives full
camera control, mount control, SER video recording, and — its distinguishing feature — **guiding
derived from the main high-FPS video stream itself** (no separate guide camera). It talks to
`indiserver` over the native INDI TCP protocol via the pure-Rust [`indi`](https://crates.io/crates/indi)
crate, so there is **no libindi/C++ linkage**.

## Build & run

Developed on NixOS; `eframe`/`winit`/`wgpu` need runtime libs (GL, Wayland/X11, `libxkbcommon`,
Vulkan, fontconfig) that the dev shell provides:

```sh
nix develop                    # enter dev shell (rustc, cargo, clippy, rust-analyzer)
cargo run --bin solar          # the GUI
cargo run --release            # use for real imaging — see profile note below
cargo test                     # run tests
cargo test <name>              # run a single test by substring
cargo clippy
```

`cargo run` (debug) is viable for GUI work because `Cargo.toml` optimizes dependencies fully
(`opt-level = 3`) and this crate lightly (`opt-level = 1`) — without that, decoding one 8 MP JPEG
frame takes ~3.4 s. Still prefer `--release` for actual imaging sessions.

### Running against simulators (no hardware)

```sh
indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
```

Exposes `CCD Simulator`, `Telescope Simulator`, `Guide Simulator`. The CCD sim starts nearly
black — use the GUI **Auto-stretch** toggle, or brighten the sim itself:
`indi_setprop "CCD Simulator.SIMULATOR_SETTINGS.SIM_SKYGLOW=13"` (lower = brighter).

### Headless / test env hooks

- `SOLAR_AUTOCONNECT=1` — auto-connect + start streaming on launch.
- `SOLAR_SCREENSHOT=path.png` — auto-connect, stream, save a GUI screenshot, then exit.
- `cargo run --example headless` — no-GUI pipeline check (connect, stream, decode, save PNG, nudge).
- Other examples in `examples/`: `probe` (connectivity spike), `guide_check`, `record_check`.

## Architecture

Two halves that communicate **only** through `bus::Bus`:

- **GUI thread** (`app.rs`) — eframe/egui, runs on the main thread. Renders the live view and all
  controls; sends `Command`s; reads shared state each repaint.
- **Async worker** (`worker.rs`) — a tokio multi-thread runtime spawned in `main.rs`. Owns the
  INDI connection/session, decodes the video stream, and translates `Command`s into INDI property
  changes. Its top-level `run()` is a `match` over incoming `Command`s.

### The Bus (`bus.rs`) — read this first

`Bus` is the single source of truth for cross-thread communication and is the best map of the
whole system. Its design is deliberate:

- **GUI → worker**: `Command` enum over an unbounded mpsc channel.
- **worker → GUI (frames)**: `latest_frame` and `display` are `ArcSwapOption` (latest-wins) — a
  high-FPS stream **drops stale frames** rather than queuing. The worker does the stretch +
  `Color32` conversion off the GUI thread; the GUI only uploads the finished `ColorImage` when
  `display_seq` bumps.
- **worker → GUI (state)**: `Shared` behind a `std::sync::Mutex`, held only briefly.
- **Lock-free flags** (`AtomicBool`/`AtomicU*`, some storing `f32` via bit pattern) let the decode
  thread and guide loop read settings without locking. Note the rule repeated throughout: **never
  hold the `recorder`/`shared` locks simultaneously**, and don't call `refresh_detect()` while
  holding `shared`.

### INDI layer (`indi/`)

Thin async wrappers over the `indi` crate: `indi/mod.rs` (`Session`, connection, discovery),
`indi/camera.rs`, `indi/mount.rs`. Key facts:

- Devices are discovered by the `DRIVER_INTERFACE` bitmask in `DRIVER_INFO`, **not** by hard-coded
  names — so simulators, PlayerOne cameras (device named after the model), and the LX200 OnStep
  mount all work unconfigured.
- Video BLOBs run on a **second dedicated INDI connection** (`BlobLink`, `BlobEnable::Only`) so
  high-rate frame data doesn't starve control commands on the main connection (`BlobEnable::Never`).
- **You must send `GetProperties` after `indi::client::start`** or no devices ever appear — the
  crate does not request them for you.

### Guiding (`guiding/`)

Stream-based: the decode thread runs target detection (throttled to ~10 Hz via `DETECT_INTERVAL`),
publishes a `GuideSample`, and the worker's async control loop issues pulse-guide corrections.
`detector.rs` has two `GuideMode`s (Disk centroid vs. Surface cross-correlation); `controller.rs`
has the calibration matrix + pulse math. Calibration must succeed before guiding.

### Recording (`recorder.rs`)

Writes the stream to **SER** files, orchestrated as a sequence of videos (`RecordConfig`). The
decode thread appends each frame's native payload; a size mismatch is counted as a dropped frame
rather than corrupting the file. Handles `.stream` (raw), `.stream.z` (zlib, inflated locally —
the `indi` crate only base64-decodes BLOBs), and `.stream_jpg` (MJPEG) frame formats.

**16-bit recording gotcha:** a 16-bit stream needs the driver switches `STREAM_FULL_DEPTH` +
`CCD_VIDEO_FORMAT` + `RAW` — **not** `CCD_CAPTURE_FORMAT`. Stream-format switches are surfaced to
the UI generically as `CameraSwitch` (driver-specific names shown as-is).

## Dependency pins (do not casually bump)

- `wcs = "=0.4.1"` — `indi` pulls `fitsrs 0.3.x`, which only builds against `wcs 0.4.1`; 0.4.2
  added fields that break it.
- `egui_plot = "0.36"` with `egui = "0.35"` — `egui_plot` is one minor ahead of `egui`
  (`egui_plot 0.35` would pull `egui 0.34`, causing a duplicate-`egui` type mismatch).

## Logging

`main.rs` silences two noisy dependencies by default (`indi::client=off`, `twinkle_client=off`)
because they emit ~1/s recoverable ERRORs during streaming. Set `RUST_LOG` to see them when
debugging INDI wire issues.
