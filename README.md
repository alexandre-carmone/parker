# solar

An INDI-driven desktop suite for **solar and planetary imaging**: full camera control,
mount control, and **guiding derived from the main high-FPS video stream** (no separate
guide camera). Written in Rust (`tokio` + `eframe`/`egui`), talking to `indiserver` over
the native INDI TCP protocol via the pure-Rust [`indi`](https://crates.io/crates/indi)
crate — no libindi/C++ linkage required.

## Status

- [x] **Step 0** — dev shell + connectivity spike (`examples/probe.rs`)
- [x] **M1** — connect + live view + mount nudge + single-frame capture
- [ ] **M2** — stream guiding (centroid/limb tracking → pulse-guide corrections)
- [ ] **M3** — SER recording, snapshots, config, polish

## Running the app

```sh
cargo run --bin solar          # GUI: Connect, Start stream, nudge the mount, Capture
```

Env hooks for headless testing:
- `SOLAR_AUTOCONNECT=1` — auto-connect + start streaming on launch.
- `SOLAR_SCREENSHOT=path.png` — auto-connect, stream, save a GUI screenshot, then exit.

Headless pipeline check (no GUI): `cargo run --example headless` connects, streams, decodes,
saves `captures/headless.png`, and nudges the mount with read-back.

The CCD simulator starts nearly black; the live view has an **Auto-stretch** toggle so faint
frames are visible. To make the simulator itself brighter for testing:
`indi_setprop "CCD Simulator.SIMULATOR_SETTINGS.SIM_SKYGLOW=13"` (lower = brighter).

## Development

This repo is developed on NixOS. The dev shell provides the Rust toolchain and the runtime
libraries eframe/winit/wgpu load at runtime (GL, Wayland/X11, `libxkbcommon`, Vulkan,
fontconfig):

```sh
nix develop            # enter the dev shell (rustc, cargo, clippy, rust-analyzer)
```

### Run the INDI simulators (no hardware needed)

```sh
indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
```

Devices exposed: `CCD Simulator`, `Telescope Simulator`, `Guide Simulator`.

### Connectivity spike

```sh
cargo run --example probe
```

Connects to `127.0.0.1:7624`, connects the CCD, enables BLOB transport, selects the MJPEG
stream encoder, starts `CCD_VIDEO_STREAM`, and prints the first few frames' format/size.

## Notes / gotchas

- `indi::client::start` does **not** request properties — after starting the client you must
  send a `GetProperties` command (`client.send(...)`) or no devices will ever appear.
- The `indi` crate transitively pulls `fitsrs 0.3.x`, which only builds against `wcs 0.4.1`
  (`wcs 0.4.2` added fields that break it). `Cargo.toml` pins `wcs = "=0.4.1"`.
- Stream frames arrive on the `CCD1` BLOB property with format `.stream_jpg` (MJPEG),
  `.stream` (raw), or `.stream.z` (zlib raw). MJPEG is the simplest to decode.
