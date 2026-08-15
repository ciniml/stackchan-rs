//! Embassy tasks, one per concern (mirroring the C++ firmware's FreeRTOS task split):
//!
//! - [`render`] — 30 fps avatar rendering onto the board display. Core 0.
//! - [`servo`] — 50 ms head motion tick: smooths pose targets from
//!   [`crate::shared_state::STATE`] through `PathGenerator` and commands the servos.
//!   Runs alone on core 1 so head motion timing is unaffected by rendering or comms.
//! - [`idle`] — idle behaviour: posts a random pose target every 1.5–4 s. Core 0.
//! - [`input`] — touch input: tap cycles the expression, touch-and-hold makes the head
//!   follow the finger and suppresses idle behaviour. Core 0.
//! - [`audio`] — synthesized sounds (boot arpeggio, touch blip) via async I2S DMA, with
//!   envelope-driven lip sync. Core 0.
//! - [`net`] — Wi-Fi station + minimal HTTP control API (expression / head / sound),
//!   or a provisioning access point with a captive setup page when no credentials are
//!   stored. Core 0.
//! - [`portal`] — captive-portal DHCP / DNS servers used in AP mode. Core 0.
//!
//! Tasks communicate only through [`crate::shared_state::STATE`] (atomics — safe across
//! cores).

pub mod audio;
pub mod idle;
pub mod input;
pub mod net;
pub mod portal;
pub mod render;
pub mod servo;
