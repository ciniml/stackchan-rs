//! Board support layer. Exactly one of the `core2` / `cores3` submodules is compiled in
//! (enforced by `compile_error!` in `main.rs`); each exposes the same surface:
//!
//! - `Board` — everything the application needs, fully initialized: the display, a
//!   [`crate::head::HeadDriver`] for the pan/tilt servos, the mechanically safe
//!   [`crate::head::HeadLimits`], and an RNG.
//! - `init()` — brings up the chip (clocks, heap, logger) and all board peripherals,
//!   returning a `Board`.

#[cfg(feature = "core2")]
mod core2;
#[cfg(feature = "core2")]
#[allow(unused_imports)]
pub use core2::{Board, DisplayTy, HeadParts, HeadTy, SpeakerTy, TouchTy, init, make_head};

#[cfg(feature = "cores3")]
mod cores3;
#[cfg(feature = "cores3")]
#[allow(unused_imports)]
pub use cores3::{Board, DisplayTy, HeadParts, HeadTy, SpeakerTy, TouchTy, init, make_head};
