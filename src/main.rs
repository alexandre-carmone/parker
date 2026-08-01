//! solar — INDI-driven solar/planetary imaging suite.
//!
//! The GUI (eframe/egui) runs on the main thread; a tokio runtime on background threads
//! owns the INDI connection and the video stream. They communicate over [`bus`].

use solar::app;
use solar::bus::Bus;
use solar::worker;

fn main() -> eframe::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                // Two dependencies flood the log during streaming with recoverable, non-actionable
                // ERRORs, so we silence them by default (set RUST_LOG to see them when debugging):
                //   * `indi::client` — its reader loop logs an ERROR for every wire message it
                //     can't model, then continues; drivers re-send un-parseable properties (`FPS`
                //     with an empty `EST_FPS`, telescope `TELESCOPE_MOUNT_TYPE`) on every update.
                //   * `twinkle_client` — the Notify primitive `indi` builds on logs a lock-acquire
                //     timeout (~1/s) whenever a value stays referenced too long, which the heavy
                //     blob traffic of a live video stream causes constantly.
                "info,solar=info,indi::client=off,twinkle_client=off".into()
            }),
        )
        .init();

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
            rt.spawn(worker::run(rx, bus_worker, ctx));
            Ok(Box::new(app::App::new(bus, tx, rt)))
        }),
    )
}
