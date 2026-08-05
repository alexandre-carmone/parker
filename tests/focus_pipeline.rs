//! End-to-end check of the focus-measurement path exactly as the decode thread runs it:
//! raw stream bytes -> `Frame::from_raw_stream` -> `to_gray` -> `focus::sharpness`. Proves the
//! metric drops as the (same) scene is defocused, through the real decode code — deterministic,
//! no hardware. (The INDI CCD simulator's *video stream* renders only noise, not its focus-
//! simulated starfield, so a live focus sweep isn't possible against the simulators.)

use solar::focus;
use solar::frame::Frame;
use solar::guiding::detector::to_gray;

const W: usize = 64;
const H: usize = 64;

/// A high-contrast, detail-rich 16-bit scene: period-4 vertical bars (0 vs 60000 ADU). Period 4
/// (not 2) so the Brenner stride-2 gradient actually sees the edges — see `focus.rs`.
fn sharp_scene() -> Vec<u16> {
    let mut px = vec![0u16; W * H];
    for row in px.chunks_exact_mut(W) {
        for (x, p) in row.iter_mut().enumerate() {
            *p = if (x / 2) % 2 == 0 { 0 } else { 60000 };
        }
    }
    px
}

/// Defocus = optical blur. A horizontal box blur of `radius` columns spreads the bars, lowering
/// local contrast just like a defocused image.
fn defocus(scene: &[u16], radius: usize) -> Vec<u16> {
    let mut out = vec![0u16; W * H];
    for (r_in, r_out) in scene.chunks_exact(W).zip(out.chunks_exact_mut(W)) {
        for (x, p) in r_out.iter_mut().enumerate() {
            let lo = x.saturating_sub(radius);
            let hi = (x + radius).min(W - 1);
            let sum: u64 = (lo..=hi).map(|i| r_in[i] as u64).sum();
            *p = (sum / (hi - lo + 1) as u64) as u16;
        }
    }
    out
}

/// Serialize a 16-bit scene to the little-endian raw payload the decode path expects.
fn raw_le(scene: &[u16]) -> Vec<u8> {
    scene.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// Run the exact decode-thread pipeline on a raw payload and return the sharpness metric.
fn measure(raw: &[u8]) -> f64 {
    let frame = Frame::from_raw_stream(raw, W, H, 1).expect("decode raw frame");
    let gray = to_gray(&frame.rgba, frame.width, frame.height);
    focus::sharpness(&gray, frame.width, frame.height)
}

#[test]
fn sharpness_decreases_monotonically_with_defocus() {
    let scene = sharp_scene();
    let in_focus = measure(&raw_le(&scene));
    let slight = measure(&raw_le(&defocus(&scene, 1)));
    let heavy = measure(&raw_le(&defocus(&scene, 3)));

    // In focus is the sharpest; each defocus step lowers the metric.
    assert!(in_focus > slight, "in_focus {in_focus} should beat slight {slight}");
    assert!(slight > heavy, "slight {slight} should beat heavy {heavy}");
    // And a heavily defocused frame still scores well above a flat (featureless) frame.
    let flat = measure(&raw_le(&vec![30000u16; W * H]));
    assert!(heavy > flat, "heavy {heavy} should beat flat {flat}");
    assert_eq!(flat, 0.0);
}
