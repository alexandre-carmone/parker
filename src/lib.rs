//! solar — INDI-driven solar/planetary imaging suite (library crate).
//!
//! The binary (`src/main.rs`) is a thin shell over these modules; examples and tests use
//! them directly to exercise the INDI pipeline without a GUI.

pub mod app;
pub mod bus;
pub mod ephemeris;
pub mod focus;
pub mod frame;
pub mod guiding;
pub mod indi;
pub mod worker;
