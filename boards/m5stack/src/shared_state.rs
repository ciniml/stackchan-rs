//! Cross-task shared state, mirroring the C++ firmware's `SharedState`: a single static
//! of lock-free atomics that producer tasks (behaviour, future network/API tasks) write
//! and consumer tasks (render, servo) read. There is no central queue — commands are
//! posted by storing into fields, consumed by the owning task on its own tick.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use m5stack_avatar_rs::Expression;

pub static STATE: SharedState = SharedState::new();

pub struct SharedState {
    /// Head pose target, 1/1024-degree units (see [`crate::head`]).
    pan_target: AtomicU32,
    tilt_target: AtomicU32,
    /// Bumped on every new pose target so the servo task can detect a re-post of the
    /// same coordinates.
    pose_seq: AtomicU32,
    /// Current avatar expression, encoded via [`expression_to_u8`].
    expression: AtomicU8,
    /// End of the current user interaction, as wrapping milliseconds (compared with a
    /// wrapping-signed difference, so the 49-day u32 rollover is harmless). While in the
    /// future, autonomous behaviour (the idle task) must not post pose targets.
    interact_until_ms: AtomicU32,
    /// Mouth openness for lip sync, 0..=100 (percent). Written by the audio task from
    /// the playback envelope, read by the render task every frame.
    mouth_open_pct: AtomicU8,
    /// Pending sound command (a [`Sound`] discriminant); 0 = none. Single-slot mailbox:
    /// a later post overwrites an unplayed one.
    sound_cmd: AtomicU8,
    /// Speaker volume, 0..=100. Seeded from the persisted config at boot.
    volume: AtomicU8,
}

/// Sounds the audio task can play.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sound {
    /// Boot arpeggio (C5 E5 G5 C6).
    Arpeggio = 1,
    /// Short blip acknowledging a touch.
    Blip = 2,
}

#[allow(dead_code)] // consumed once input/network tasks post expressions
fn expression_to_u8(e: Expression) -> u8 {
    match e {
        Expression::Neutral => 0,
        Expression::Happy => 1,
        Expression::Angry => 2,
        Expression::Sad => 3,
        Expression::Doubt => 4,
        Expression::Sleepy => 5,
    }
}

fn expression_from_u8(v: u8) -> Expression {
    match v {
        1 => Expression::Happy,
        2 => Expression::Angry,
        3 => Expression::Sad,
        4 => Expression::Doubt,
        5 => Expression::Sleepy,
        _ => Expression::Neutral,
    }
}

impl SharedState {
    const fn new() -> Self {
        Self {
            pan_target: AtomicU32::new(90 * crate::head::DEG),
            tilt_target: AtomicU32::new(90 * crate::head::DEG),
            pose_seq: AtomicU32::new(0),
            expression: AtomicU8::new(1), // Happy
            interact_until_ms: AtomicU32::new(0),
            mouth_open_pct: AtomicU8::new(0),
            sound_cmd: AtomicU8::new(0),
            volume: AtomicU8::new(80),
        }
    }

    /// Post a new head pose target. Values are clamped by the servo task against the
    /// board's `HeadLimits`.
    pub fn set_head_target(&self, pan: u32, tilt: u32) {
        self.pan_target.store(pan, Ordering::Relaxed);
        self.tilt_target.store(tilt, Ordering::Relaxed);
        self.pose_seq.fetch_add(1, Ordering::Release);
    }

    /// Returns `(pan, tilt, seq)`. A change in `seq` means a new target was posted.
    pub fn head_target(&self) -> (u32, u32, u32) {
        let seq = self.pose_seq.load(Ordering::Acquire);
        (
            self.pan_target.load(Ordering::Relaxed),
            self.tilt_target.load(Ordering::Relaxed),
            seq,
        )
    }

    #[allow(dead_code)] // consumed once input/network tasks post expressions
    pub fn set_expression(&self, e: Expression) {
        self.expression.store(expression_to_u8(e), Ordering::Relaxed);
    }

    pub fn expression(&self) -> Expression {
        expression_from_u8(self.expression.load(Ordering::Relaxed))
    }

    /// Advance to the next expression in a fixed cycle (used by tap input).
    pub fn cycle_expression(&self) {
        let next = (self.expression.load(Ordering::Relaxed) + 1) % 6;
        self.expression.store(next, Ordering::Relaxed);
    }

    /// Mark the user as interacting until `now_ms + hold_ms`; suppresses idle behaviour.
    pub fn note_interaction(&self, now_ms: u32, hold_ms: u32) {
        self.interact_until_ms
            .store(now_ms.wrapping_add(hold_ms), Ordering::Relaxed);
    }

    pub fn interaction_active(&self, now_ms: u32) -> bool {
        let until = self.interact_until_ms.load(Ordering::Relaxed);
        (until.wrapping_sub(now_ms) as i32) > 0
    }

    pub fn set_mouth_open_pct(&self, pct: u8) {
        self.mouth_open_pct.store(pct.min(100), Ordering::Relaxed);
    }

    pub fn mouth_open_pct(&self) -> u8 {
        self.mouth_open_pct.load(Ordering::Relaxed)
    }

    /// Request a sound. Overwrites any not-yet-played request.
    pub fn post_sound(&self, sound: Sound) {
        self.sound_cmd.store(sound as u8, Ordering::Relaxed);
    }

    /// Take the pending sound request, if any (clears the mailbox).
    pub fn take_sound(&self) -> Option<Sound> {
        match self.sound_cmd.swap(0, Ordering::Relaxed) {
            1 => Some(Sound::Arpeggio),
            2 => Some(Sound::Blip),
            _ => None,
        }
    }

    pub fn set_volume(&self, volume: u8) {
        self.volume.store(volume.min(100), Ordering::Relaxed);
    }

    pub fn volume(&self) -> u8 {
        self.volume.load(Ordering::Relaxed)
    }
}
