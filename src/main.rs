//! solar — INDI-driven solar/planetary imaging suite.
//!
//! The GUI (eframe/egui) runs on the main thread; a tokio runtime on background threads
//! owns the INDI connection and the video stream. They communicate over [`bus`].

use solar::app;
use solar::bus::Bus;
use solar::worker;
use tracing_subscriber::prelude::*;

fn main() -> eframe::Result<()> {
    // Two independent log sinks:
    //
    //  * stdout — quiet by default so the terminal stays readable during streaming. Two
    //    dependencies flood the log with recoverable, non-actionable ERRORs, so we silence them
    //    here (set RUST_LOG to override):
    //      - `indi::client`   — its reader loop logs an ERROR for every wire message it can't
    //        model, then continues; drivers re-send un-parseable properties (`FPS` with an empty
    //        `EST_FPS`, telescope `TELESCOPE_MOUNT_TYPE`) on every update.
    //      - `twinkle_client` — the Notify primitive `indi` builds on logs a lock-acquire timeout
    //        (~1/s) whenever a value stays referenced too long, which heavy blob traffic causes.
    //
    //  * a deep, daily-rolling FILE in ./logs — always on, captures those two deps too, so if the
    //    stream freezes on its own (see the reader-exit / stall-watchdog logs in worker.rs) the
    //    evidence is already on disk. Non-blocking so file I/O never stalls the decode path.
    //    Override its verbosity with SOLAR_FILE_LOG (an EnvFilter directive string).
    let stdout_filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "info,solar=info,indi::client=off,twinkle_client=off".into());
    let file_filter = tracing_subscriber::EnvFilter::new(
        std::env::var("SOLAR_FILE_LOG")
            .unwrap_or_else(|_| "info,solar=debug,indi::client=info,twinkle_client=info".into()),
    );

    let log_dir = std::env::var("SOLAR_LOG_DIR").unwrap_or_else(|_| "logs".into());
    let file_appender = tracing_appender::rolling::daily(&log_dir, "solar.log");
    let (file_writer, _guard) = tracing_appender::non_blocking(file_appender);

    tracing_subscriber::registry()
        .with(
            tracing_subscriber::fmt::layer()
                .with_writer(std::io::stdout)
                .with_filter(stdout_filter),
        )
        .with(
            tracing_subscriber::fmt::layer()
                .with_writer(file_writer)
                .with_ansi(false)
                .with_target(true)
                .with_filter(file_filter),
        )
        .init();

    // Session marker so a specific run (e.g. one that ended in a freeze) is easy to find in a
    // daily-rolling file that may hold several runs.
    tracing::info!(
        pid = std::process::id(),
        "=== solar session start (deep log → {log_dir}/solar.log.<date>) ==="
    );

    let bus = Bus::new();
    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("building tokio runtime");

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("solar — INDI planetary imaging"),
        ..Default::default()
    };

    let bus_worker = bus.clone();
    eframe::run_native(
        "solar",
        options,
        Box::new(move |cc| {
            // The worker needs the egui Context (created by eframe) to request repaints.
            let ctx = cc.egui_ctx.clone();
            // Use the light (white) theme for the UI.
            cc.egui_ctx.set_theme(egui::Theme::Light);
            rt.spawn(worker::run(rx, bus_worker, ctx));
            Ok(Box::new(app::App::new(bus, tx, rt)))
        }),
    )
}
