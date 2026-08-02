//! Headless end-to-end check for M3 SER recording against the INDI simulators.
//!
//! Drives the real worker command path (no GUI): connect → (pick a raw/16-bit capture format if
//! the driver offers one) → stream → record a two-video sequence stopped by frame count, then
//! parse the produced `.ser` files and validate their headers and byte sizes. Finally, a
//! best-effort check that guiding and recording run concurrently without stopping each other.
//!
//! Run alongside:
//!   indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
//! then:  cargo run --example record_check
//!
//! Exits non-zero if the recording sequence didn't produce valid SER files.

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use solar::bus::{Bus, Command, ConnState, RecordConfig, RecordStop};
use solar::worker;
use tokio::sync::mpsc;

/// Poll `f` until it returns true or `timeout` elapses; returns whether it succeeded.
async fn wait_until(label: &str, timeout: Duration, mut f: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if f() {
            println!("  ✓ {label}");
            return true;
        }
        if Instant::now() >= deadline {
            println!("  ✗ {label} (timed out after {timeout:?})");
            return false;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

fn read_i32(buf: &[u8], off: usize) -> i32 {
    i32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

/// Parse and sanity-check one SER file. Returns Ok(frame_count) or Err(reason).
fn validate_ser(path: &PathBuf) -> Result<i32, String> {
    let bytes = fs::read(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    if bytes.len() < 178 {
        return Err(format!("{}: shorter than a SER header", path.display()));
    }
    if &bytes[0..14] != b"LUCAM-RECORDER" {
        return Err(format!("{}: bad FileID", path.display()));
    }
    let color_id = read_i32(&bytes, 18);
    let width = read_i32(&bytes, 26);
    let height = read_i32(&bytes, 30);
    let depth = read_i32(&bytes, 34);
    let count = read_i32(&bytes, 38);
    if width <= 0 || height <= 0 {
        return Err(format!("{}: bad geometry {width}×{height}", path.display()));
    }
    if count <= 0 {
        return Err(format!("{}: FrameCount not patched ({count})", path.display()));
    }
    let planes = if color_id == 100 { 3 } else { 1 }; // 100 = RGB
    let bpp = if depth > 8 { 2 } else { 1 };
    let expected =
        178 + (count as usize) * (width as usize) * (height as usize) * planes * bpp
            + (count as usize) * 8; // per-frame timestamp trailer
    if bytes.len() != expected {
        return Err(format!(
            "{}: size {} != expected {} ({width}×{height}, {depth}-bit, colorid {color_id}, {count} frames)",
            path.display(),
            bytes.len(),
            expected
        ));
    }
    println!(
        "    {} — {width}×{height}, {depth}-bit, colorid {color_id}, {count} frames, {} bytes ✓",
        path.file_name().unwrap().to_string_lossy(),
        bytes.len()
    );
    Ok(count)
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "solar=info,warn".into()),
        )
        .init();
    let addr = std::env::var("SOLAR_ADDR").unwrap_or_else(|_| "127.0.0.1:7624".to_string());

    // Unique output dir under the system temp folder.
    let out_dir = std::env::temp_dir().join(format!(
        "solar_record_check_{}",
        std::process::id()
    ));
    let _ = fs::remove_dir_all(&out_dir);
    let dir_str = out_dir.to_string_lossy().to_string();

    let bus = Bus::new();
    let (tx, rx) = mpsc::unbounded_channel();
    let ctx = egui::Context::default();
    let worker = tokio::spawn(worker::run(rx, bus.clone(), ctx));
    let send = |c: Command| {
        let _ = tx.send(c);
    };

    println!("connecting to {addr} …");
    send(Command::Connect { addr });
    if !wait_until("connected", Duration::from_secs(15), || {
        bus.shared.lock().unwrap().conn == ConnState::Connected
    })
    .await
    {
        std::process::exit(1);
    }

    // Push every stream-format switch toward native raw + 16-bit where the driver offers it, so
    // the recorder captures true depth. Each switch's options are driver-specific; we pick the
    // element that looks like RAW / 16-bit / full-depth-16.
    let switches = bus.shared.lock().unwrap().stream_switches.clone();
    println!("  stream switches offered:");
    for sw in &switches {
        println!("    {} ({}) = {:?} [{}]", sw.label, sw.prop, sw.options, sw.selected);
        let want = sw.options.iter().find(|o| {
            let u = o.to_uppercase();
            (u.contains("RAW") && u.contains("16"))
                || u.contains("16BIT")
                || (sw.prop == "CCD_STREAM_ENCODER" && u == "RAW")
        });
        if let Some(elem) = want {
            if *elem != sw.selected {
                println!("      → set {} = {elem}", sw.prop);
                send(Command::SetCameraSwitch {
                    prop: sw.prop.clone(),
                    elem: elem.clone(),
                });
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
    }

    send(Command::StartStream);
    if !wait_until("frames flowing", Duration::from_secs(10), || {
        bus.shared.lock().unwrap().frame_count > 5
    })
    .await
    {
        std::process::exit(1);
    }

    // Record a 2-video sequence: 20 frames each, 1s between.
    const TARGET: u64 = 20;
    println!("recording 2 videos × {TARGET} frames (1s delay) → {dir_str}");
    send(Command::StartRecording(RecordConfig {
        dir: dir_str.clone(),
        count: 2,
        stop: RecordStop::Frames(TARGET),
        delay_secs: 1.0,
    }));

    wait_until("recording started", Duration::from_secs(5), || {
        bus.shared.lock().unwrap().recording.active
    })
    .await;
    let finished = wait_until("recording sequence finished", Duration::from_secs(40), || {
        let r = &bus.shared.lock().unwrap().recording;
        !r.active && r.last_file.is_some()
    })
    .await;
    if !finished {
        eprintln!("recording did not finish cleanly");
        std::process::exit(1);
    }

    // Validate the produced files.
    let mut sers: Vec<PathBuf> = fs::read_dir(&out_dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().map(|x| x == "ser").unwrap_or(false))
                .collect()
        })
        .unwrap_or_default();
    sers.sort();
    println!("\n--- SER validation ({} files) ---", sers.len());
    if sers.len() != 2 {
        eprintln!("expected 2 .ser files, found {}", sers.len());
        std::process::exit(1);
    }
    let mut ok = true;
    for p in &sers {
        match validate_ser(p) {
            Ok(n) if n as u64 >= TARGET => {}
            Ok(n) => {
                eprintln!("  {}: only {n} frames (< {TARGET})", p.display());
                ok = false;
            }
            Err(e) => {
                eprintln!("  {e}");
                ok = false;
            }
        }
    }
    if !ok {
        std::process::exit(1);
    }

    // Best-effort: guiding + recording concurrently, confirming neither tears the other down.
    println!("\n--- guiding-during-recording (best-effort) ---");
    send(Command::SetDetectionOverlay(true));
    let detected = wait_until("target detected", Duration::from_secs(6), || {
        bus.shared.lock().unwrap().detected.is_some()
    })
    .await;
    if detected {
        send(Command::Calibrate);
        let calibrated = wait_until("calibration finished", Duration::from_secs(30), || {
            let sh = bus.shared.lock().unwrap();
            !sh.calibrating && sh.calibrated
        })
        .await;
        if calibrated {
            send(Command::StartGuiding);
            wait_until("guiding started", Duration::from_secs(4), || {
                bus.shared.lock().unwrap().guiding
            })
            .await;
            send(Command::StartRecording(RecordConfig {
                dir: dir_str.clone(),
                count: 1,
                stop: RecordStop::Seconds(3.0),
                delay_secs: 0.0,
            }));
            let concurrent = wait_until(
                "guiding AND recording active together",
                Duration::from_secs(5),
                || {
                    let sh = bus.shared.lock().unwrap();
                    sh.guiding && sh.recording.active
                },
            )
            .await;
            // Let it run, then confirm guiding survived the whole recording.
            tokio::time::sleep(Duration::from_secs(4)).await;
            let still_guiding = bus.shared.lock().unwrap().guiding;
            println!("    concurrent={concurrent}, guiding still active after record={still_guiding}");
            send(Command::StopGuiding);
        } else {
            println!("    calibration didn't converge on the simulator — skipping concurrency check");
        }
    } else {
        println!("    no target detected — skipping concurrency check");
    }

    println!("\n--- worker log (tail) ---");
    for line in bus.shared.lock().unwrap().log.iter().rev().take(14).rev() {
        println!("  {line}");
    }

    send(Command::Disconnect);
    tokio::time::sleep(Duration::from_millis(300)).await;
    worker.abort();
    let _ = fs::remove_dir_all(&out_dir);
    println!("\ndone — SER recording verified.");
}
