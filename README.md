# solar

An INDI-driven desktop suite for **solar and planetary imaging**: full camera control,
mount control, SER video recording, and — its distinguishing feature — **guiding derived
from the main high-FPS video stream itself** (no separate guide camera). Written in Rust
(`tokio` + `eframe`/`egui`), talking to `indiserver` over the native INDI TCP protocol via
the pure-Rust [`indi`](https://crates.io/crates/indi) crate — no libindi/C++ linkage required.

## Features

- **Live view** with off-thread auto-stretch; a high-FPS stream drops stale frames rather than
  queuing, so the display stays current.
- **Camera control** — exposure/gain/ROI, plus stream-format switches surfaced generically
  (including the 16-bit RAW path).
- **Mount control** — nudge/slew via pulse-guide and motion commands.
- **Stream guiding** — target detection runs on the decode thread (disk centroid or surface
  cross-correlation); the async control loop issues pulse-guide corrections. Calibration must
  succeed before guiding.
- **SER recording** — writes the native stream to SER files, orchestrated as sequences of
  videos. Handles `.stream` (raw), `.stream.z` (zlib), and `.stream_jpg` (MJPEG) payloads.

Devices are discovered by the `DRIVER_INTERFACE` bitmask, not by hard-coded names, so
simulators, PlayerOne cameras, and the LX200/OnStep mount all work unconfigured.

## Running the app

```sh
cargo run --bin solar          # GUI
cargo run --release            # use for real imaging sessions
```

`cargo run` (debug) is viable for GUI work because `Cargo.toml` optimizes dependencies fully
and this crate lightly; still prefer `--release` for actual imaging.

Env hooks for headless testing:
- `SOLAR_AUTOCONNECT=1` — auto-connect + start streaming on launch.
- `SOLAR_SCREENSHOT=path.png` — auto-connect, stream, save a GUI screenshot, then exit.

Headless pipeline check (no GUI): `cargo run --example headless` connects, streams, decodes,
saves a PNG, and nudges the mount with read-back. Other examples: `probe` (connectivity spike),
`guide_check`, `record_check`.

The CCD simulator starts nearly black; the live view has an **Auto-stretch** toggle so faint
frames are visible. To make the simulator itself brighter for testing:
`indi_setprop "CCD Simulator.SIMULATOR_SETTINGS.SIM_SKYGLOW=13"` (lower = brighter).

## Development

This repo is developed on NixOS. The dev shell provides the Rust toolchain and the runtime
libraries eframe/winit/wgpu load at runtime (GL, Wayland/X11, `libxkbcommon`, Vulkan,
fontconfig):

```sh
nix develop            # enter the dev shell (rustc, cargo, clippy, rust-analyzer)
cargo test             # run tests
cargo clippy
```

### Run the INDI simulators (no hardware needed)

```sh
indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
```

Devices exposed: `CCD Simulator`, `Telescope Simulator`, `Guide Simulator`.

## Architecture

Two halves that communicate **only** through `bus::Bus`:

- **GUI thread** (`app.rs`) — eframe/egui on the main thread. Renders the live view and
  controls, sends `Command`s, reads shared state each repaint.
- **Async worker** (`worker.rs`) — a tokio runtime that owns the INDI connection/session,
  decodes the video stream, and translates `Command`s into INDI property changes.

`Bus` (`bus.rs`) is the single source of truth: `Command`s flow GUI → worker over an mpsc
channel; frames flow worker → GUI via latest-wins `ArcSwapOption` (stale frames dropped);
shared state sits behind a briefly-held mutex, with lock-free atomic flags for the hot paths.

Video BLOBs run on a **second dedicated INDI connection** so high-rate frame data doesn't
starve control commands on the main connection.

## Notes / gotchas

- `indi::client::start` does **not** request properties — after starting the client you must
  send a `GetProperties` command or no devices will ever appear.
- The `indi` crate transitively pulls `fitsrs 0.3.x`, which only builds against `wcs 0.4.1`
  (`wcs 0.4.2` added fields that break it). `Cargo.toml` pins `wcs = "=0.4.1"`.
- 16-bit recording needs the driver switches `STREAM_FULL_DEPTH` + `CCD_VIDEO_FORMAT` +
  `RAW` — **not** `CCD_CAPTURE_FORMAT`.
- `main.rs` silences `indi::client` and `twinkle_client` logs by default (they emit ~1/s
  recoverable ERRORs while streaming); set `RUST_LOG` to see them when debugging wire issues.
