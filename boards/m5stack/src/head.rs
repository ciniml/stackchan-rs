//! Board-independent head (pan/tilt) pose interface.
//!
//! The application layer works in a single pose space — degrees as 1/1024-degree fixed
//! point (`u32`), pan/tilt both centered around 90° — which is also the unit fed to
//! `stackchan_rs::path_generator::PathGenerator`. Each board provides a [`HeadDriver`]
//! that converts poses to its actuator (MCPWM hobby servos on Core2, SCS0009 serial bus
//! servos on CoreS3) and a [`HeadLimits`] describing the mechanically safe range.

/// One degree in the fixed-point pose space.
pub const DEG: u32 = 1024;

/// Mechanically safe pose range for a head, in 1/1024-degree units.
#[derive(Clone, Copy, Debug)]
pub struct HeadLimits {
    pub pan_min: u32,
    pub pan_max: u32,
    pub tilt_min: u32,
    pub tilt_max: u32,
}

impl HeadLimits {
    pub const fn pan_center(&self) -> u32 {
        (self.pan_min + self.pan_max) / 2
    }
    pub const fn tilt_center(&self) -> u32 {
        (self.tilt_min + self.tilt_max) / 2
    }
    pub fn clamp_pan(&self, pan: u32) -> u32 {
        pan.clamp(self.pan_min, self.pan_max)
    }
    pub fn clamp_tilt(&self, tilt: u32) -> u32 {
        tilt.clamp(self.tilt_min, self.tilt_max)
    }
}

/// Actuator backend for the head. Implementations must tolerate a partially wired or
/// absent servo bus: `set_pose` on an absent servo is a no-op. (Whole-head absence is
/// reported by `Board::head_present` at bring-up; the servo task is simply not started.)
pub trait HeadDriver {
    /// Command the head towards the pose. `period_ms` is the expected interval until the
    /// next command; drivers that support timed moves (SCS0009) use it as the move time.
    /// Pose values are pre-clamped by the caller via [`HeadLimits`].
    fn set_pose(&mut self, pan: u32, tilt: u32, period_ms: u16);
}
