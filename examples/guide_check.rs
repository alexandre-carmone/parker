//! Headless end-to-end check for M2 guiding against the INDI simulators.
//!
//! Drives the real worker command path (no GUI): connect → stream → enable detection →
//! calibrate → guide, polling the shared state and printing what happened. Run alongside:
//!   indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide
//! then:  cargo run --example guide_check
//!
//! It reports observed facts (frames flowing, target detected, calibration result, guide
//! corrections issued) rather than asserting a clean lock — the simulated field may not
//! translate under guide pulses the way a real sky does.

use std::time::{Duration, Instant};

use solar::bus::{Bus, Command, ConnState};
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

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "solar=info,warn".into()),
        )
        .init();
    let addr = std::env::var("SOLAR_ADDR").unwrap_or_else(|_| "127.0.0.1:7624".to_string());

    let bus = Bus::new();
    let (tx, rx) = mpsc::unbounded_channel();
    let ctx = egui::Context::default();
    let worker = tokio::spawn(worker::run(rx, bus.clone(), ctx));

    let send = |c: Command| {
        let _ = tx.send(c);
    };

    println!("connecting to {addr} …");
    send(Command::Connect { addr });
    let connected = wait_until("connected", Duration::from_secs(15), || {
        bus.shared.lock().unwrap().conn == ConnState::Connected
    })
    .await;
    if !connected {
        std::process::exit(1);
    }

    send(Command::StartStream);
    wait_until("frames flowing", Duration::from_secs(10), || {
        bus.shared.lock().unwrap().frame_count > 5
    })
    .await;

    send(Command::SetDetectionOverlay(true));
    let detected = wait_until("target detected", Duration::from_secs(6), || {
        bus.shared.lock().unwrap().detected.is_some()
    })
    .await;
    if let Some((x, y)) = bus.shared.lock().unwrap().detected {
        println!("    detected target at ({x:.1}, {y:.1}) px");
    }

    if detected {
        println!("calibrating (pulses the mount N/S/E/W) …");
        send(Command::Calibrate {
            pulse_ms: solar::guiding::DEFAULT_CALIB_MS,
        });
        // Calibration issues four ~1.5s pulses with settle time.
        wait_until("calibration finished", Duration::from_secs(30), || {
            let sh = bus.shared.lock().unwrap();
            !sh.calibrating && (sh.calibrated || sh.log.iter().any(|l| l.contains("calibration failed")))
        })
        .await;

        let calibrated = bus.shared.lock().unwrap().calibrated;
        if calibrated {
            println!("guiding for 6s …");
            send(Command::StartGuiding);
            wait_until("guiding started", Duration::from_secs(4), || {
                bus.shared.lock().unwrap().guiding
            })
            .await;
            tokio::time::sleep(Duration::from_secs(6)).await;
            let (n, rms) = {
                let sh = bus.shared.lock().unwrap();
                (sh.guide_history.len(), sh.guide_rms)
            };
            println!("    guide-loop cycles recorded: {n}, RMS {rms:.2} px");
            send(Command::StopGuiding);
        }
    }

    // Dump the recent log so the calibration measurements / outcome are visible.
    println!("\n--- worker log (tail) ---");
    for line in bus.shared.lock().unwrap().log.iter().rev().take(12).rev() {
        println!("  {line}");
    }

    send(Command::Disconnect);
    tokio::time::sleep(Duration::from_millis(300)).await;
    worker.abort();
    println!("\ndone.");
}
