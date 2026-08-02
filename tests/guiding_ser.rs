//! End-to-end validation of the stream-guiding pipeline against **real recorded SER captures**.
//!
//! The unit tests in `guiding/detector.rs` and `guiding/controller.rs` exercise the pure math on
//! synthetic buffers. This test closes the loop on genuine data: it decodes frames from a real
//! `.ser` file, runs the actual [`GuideDetector`] over them exactly as the worker's decode thread
//! does, and checks that the detector locks onto the target and reports a coherent, slowly
//! drifting position — then feeds that measured drift through the real [`Calibration`]/[`pulses_for`]
//! control math and checks the correction it would command opposes the drift.
//!
//! ## Skipping
//! Captures are large and git-ignored, so this test is data-gated: it runs only when a suitable
//! `.ser` is found (via `SOLAR_TEST_SER`, else the first mono capture under `captures/`) and is a
//! no-op pass otherwise, so CI without the data still goes green.
//!
//! ## Why it scales 16-bit → 8-bit before detecting
//! These solar captures are 16-bit but very faint — the raw sample peak is only a few hundred out
//! of 65535. The live detector consumes `Frame::rgba`, which for a 16-bit stream is built from the
//! **high byte alone** (`frame.rs::from_raw_stream`); on faint data that high byte is nearly flat
//! (max gray 1–2), so raw detection is unreliable (Surface/NCC can't lock at all, and the Disk
//! centroid latches onto scattered noise on ~10% of frames). Detection is meant to run on real
//! signal — an 8-bit mono stream, or the auto-stretched view. We reproduce that here by scaling
//! each frame by its own peak (per-frame auto-stretch) before handing it to the detector. Because
//! [`centroid_threshold`] normalizes internally, this scaling only recovers quantization precision;
//! it does not manufacture the drift being measured.

use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom};
use std::path::PathBuf;
use std::sync::atomic::Ordering;

use solar::bus::{Bus, Dir};
use solar::frame::Frame;
use solar::guiding::{pulses_for, Calibration, GuideDetector, GuideMode, GuideParams};

/// SER v3 header length; frame data starts here (mirrors `recorder::HEADER_LEN`).
const HEADER_LEN: u64 = 178;
/// Cap on how many frames we decode — enough to characterize the drift without reading GBs.
const MAX_FRAMES: usize = 300;
/// Don't bother asserting on a capture shorter than this — too little to be meaningful.
const MIN_FRAMES: usize = 60;

/// The little of the SER header we need to interpret the frames.
struct SerHeader {
    width: usize,
    height: usize,
    /// Bits per sample (8 or 16).
    depth: usize,
    frames: usize,
    /// SER `ColorID` — we only handle mono (0) here.
    color_id: i32,
}

fn read_i32(f: &mut File, off: u64) -> io::Result<i32> {
    let mut b = [0u8; 4];
    f.seek(SeekFrom::Start(off))?;
    f.read_exact(&mut b)?;
    Ok(i32::from_le_bytes(b))
}

fn read_header(f: &mut File) -> io::Result<SerHeader> {
    Ok(SerHeader {
        color_id: read_i32(f, 18)?,
        width: read_i32(f, 26)? as usize,
        height: read_i32(f, 30)? as usize,
        depth: read_i32(f, 34)? as usize,
        frames: read_i32(f, 38)? as usize,
    })
}

/// Find a mono SER to test: `SOLAR_TEST_SER` if set, else the first readable mono capture under
/// `captures/`. Returns `None` (→ skip) when no suitable file exists.
fn locate_ser() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("SOLAR_TEST_SER") {
        let path = PathBuf::from(p);
        return path.exists().then_some(path);
    }
    let mut candidates: Vec<PathBuf> = std::fs::read_dir("captures")
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "ser"))
        .collect();
    candidates.sort();
    candidates.into_iter().find(|p| {
        File::open(p)
            .and_then(|mut f| read_header(&mut f))
            .map(|h| h.color_id == 0 && (h.depth == 8 || h.depth == 16) && h.frames >= MIN_FRAMES)
            .unwrap_or(false)
    })
}

/// Read one frame's payload and scale it to an 8-bit mono buffer by its own peak (per-frame
/// auto-stretch — see the module doc). `depth` is 8 or 16.
fn read_scaled_frame(f: &mut File, idx: usize, hdr: &SerHeader, buf: &mut [u8]) -> io::Result<Vec<u8>> {
    let bytes_per_sample = hdr.depth / 8;
    let frame_len = hdr.width * hdr.height * bytes_per_sample;
    f.seek(SeekFrom::Start(HEADER_LEN + (idx * frame_len) as u64))?;
    f.read_exact(&mut buf[..frame_len])?;
    if bytes_per_sample == 1 {
        return Ok(buf[..frame_len].to_vec());
    }
    let mut peak = 1u16;
    for s in buf[..frame_len].chunks_exact(2) {
        peak = peak.max(u16::from_le_bytes([s[0], s[1]]));
    }
    let gray = buf[..frame_len]
        .chunks_exact(2)
        .map(|s| {
            let v = u16::from_le_bytes([s[0], s[1]]) as u32;
            ((v * 255) / peak as u32).min(255) as u8
        })
        .collect();
    Ok(gray)
}

fn median(mut v: Vec<f32>) -> f32 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

/// The `p`-th percentile (0–100) of `v`.
fn percentile(mut v: Vec<f32>, p: usize) -> f32 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[(v.len() * p / 100).min(v.len() - 1)]
}

fn mag((x, y): (f32, f32)) -> f32 {
    (x * x + y * y).sqrt()
}

/// Run the real [`GuideDetector`] (Disk mode) over up to [`MAX_FRAMES`] frames of `path` and
/// return `(header, track)` where `track[i]` is the measured target position or `None` (no lock).
fn detect_track(path: &PathBuf) -> (SerHeader, Vec<Option<(f32, f32)>>) {
    let mut f = File::open(path).expect("open SER");
    let hdr = read_header(&mut f).expect("read SER header");
    let n = hdr.frames.min(MAX_FRAMES);
    let frame_len = hdr.width * hdr.height * (hdr.depth / 8);
    let mut buf = vec![0u8; frame_len];

    // Drive the detector exactly as the decode thread does: read mode + generation from the Bus.
    let bus = Bus::new();
    bus.guide_mode.store(GuideMode::Disk.as_u8(), Ordering::Relaxed);
    let mut det = GuideDetector::default();

    let track = (0..n)
        .map(|i| {
            let gray = read_scaled_frame(&mut f, i, &hdr, &mut buf).expect("read frame");
            let frame = Frame::from_raw_stream(&gray, hdr.width, hdr.height, i as u64).expect("decode");
            det.measure(&frame, &bus)
        })
        .collect();
    (hdr, track)
}

#[test]
fn guiding_locks_and_tracks_recorded_drift() {
    let Some(path) = locate_ser() else {
        eprintln!("guiding_ser: no mono .ser capture found (set SOLAR_TEST_SER) — skipping");
        return;
    };
    eprintln!("guiding_ser: validating against {}", path.display());

    let (hdr, track) = detect_track(&path);
    let n = track.len();
    assert!(n >= MIN_FRAMES, "capture too short: {n} frames");

    // 1) The detector must keep a lock on essentially every frame — a coherent target, not noise.
    let locked = track.iter().filter(|p| p.is_some()).count();
    let lock_ratio = locked as f32 / n as f32;
    assert!(
        lock_ratio >= 0.95,
        "detector lost lock on too many frames: {locked}/{n} locked ({:.0}%)",
        lock_ratio * 100.0
    );

    // Thresholds are scale-relative (a fraction of the shorter frame dimension) so the test holds
    // for any well-framed disk capture regardless of sensor size. They are deliberately generous:
    // the claim is a *coherent slow drift*, not a specific pixel count.
    let min_dim = hdr.width.min(hdr.height) as f32;
    let xs: Vec<f32> = track.iter().flatten().map(|p| p.0).collect();
    let ys: Vec<f32> = track.iter().flatten().map(|p| p.1).collect();

    // 2) The track stays clustered — the centroid never wanders off across the whole run. Measured
    //    as the 90th-percentile distance from the median center (robust to a few noisy frames).
    let (cx, cy) = (median(xs.clone()), median(ys.clone()));
    let devs: Vec<f32> = track
        .iter()
        .flatten()
        .map(|p| mag((p.0 - cx, p.1 - cy)))
        .collect();
    let p90_dev = percentile(devs.clone(), 90);
    assert!(
        p90_dev <= 0.08 * min_dim,
        "target wanders too much: p90 deviation {p90_dev:.1}px > {:.1}px (8% of {min_dim})",
        0.08 * min_dim
    );

    // 3) Frame-to-frame motion is small — the detector tracks smoothly rather than jumping.
    let steps: Vec<f32> = xs
        .windows(2)
        .zip(ys.windows(2))
        .map(|(x, y)| mag((x[1] - x[0], y[1] - y[0])))
        .collect();
    let median_step = median(steps.clone());
    assert!(
        median_step <= 0.05 * min_dim,
        "detector jumps between frames: median step {median_step:.1}px > {:.1}px",
        0.05 * min_dim
    );

    // 4) There is a bounded net drift — "slightly low", i.e. a slow wander, not a runaway. Measured
    //    robustly as the median position of the first vs. last 10% of the run.
    let k = (locked / 10).max(1);
    let drift = (
        median(xs[xs.len() - k..].to_vec()) - median(xs[..k].to_vec()),
        median(ys[ys.len() - k..].to_vec()) - median(ys[..k].to_vec()),
    );
    assert!(
        mag(drift) <= 0.10 * min_dim,
        "net drift {:.1}px is not a slow drift (> {:.1}px)",
        mag(drift),
        0.10 * min_dim
    );

    eprintln!(
        "guiding_ser: {n} frames, {:.0}% locked, center ({cx:.0},{cy:.0}), \
         net drift {:.1},{:.1}px (|{:.1}|), p90 dev {p90_dev:.1}px, median step {median_step:.1}px",
        lock_ratio * 100.0,
        drift.0,
        drift.1,
        mag(drift),
    );

    // 5) Feed the measured drift through the real control math: the correction must cancel it.
    //    A simple axis-aligned calibration (West→+x, North→+y at 0.01 px/ms) stands in for a real
    //    calibration run — enough to check the closed-form inverse and the pulse directions.
    let cal = Calibration::from_pulses((10.0, 0.0), (0.0, 10.0), 1000.0).unwrap();

    // The correction the controller solves for, applied via the calibration's forward map, must
    // move the target by −drift (back onto the lock point).
    let (ra_ms, dec_ms) = cal.correct(drift);
    let applied = cal.displacement(ra_ms, dec_ms);
    assert!(
        mag((applied.0 + drift.0, applied.1 + drift.1)) < 1e-2,
        "correction does not cancel measured drift: applied {applied:?} vs drift {drift:?}"
    );

    // And the concrete pulses issued this cycle oppose the drift on each axis that clears the
    // deadband: +x drift → pulse East, −x → West; +y → South, −y → North.
    let params = GuideParams {
        ra_aggr: 1.0,
        dec_aggr: 1.0,
        ..Default::default()
    };
    let pulses = pulses_for(&cal, &params, drift);
    for (dir, ms) in &pulses {
        let max = params.ra_max_pulse_ms.max(params.dec_max_pulse_ms) as f64;
        assert!(*ms <= max, "pulse {ms} exceeds max");
        match dir {
            Dir::East => assert!(drift.0 > 0.0, "East pulse but drift.x = {}", drift.0),
            Dir::West => assert!(drift.0 < 0.0, "West pulse but drift.x = {}", drift.0),
            Dir::South => assert!(drift.1 > 0.0, "South pulse but drift.y = {}", drift.1),
            Dir::North => assert!(drift.1 < 0.0, "North pulse but drift.y = {}", drift.1),
        }
    }
}
