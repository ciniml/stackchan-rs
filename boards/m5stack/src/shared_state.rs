//! Cross-task shared state, mirroring the C++ firmware's `SharedState`: a single static
//! of lock-free atomics that producer tasks (behaviour, future network/API tasks) write
//! and consumer tasks (render, servo) read. There is no central queue — commands are
//! posted by storing into fields, consumed by the owning task on its own tick.

use core::cell::RefCell;
use core::sync::atomic::{AtomicI8, AtomicU8, AtomicU32, Ordering};

use critical_section::Mutex as CsMutex;

use m5stack_avatar_rs::Expression;

pub static STATE: SharedState = SharedState::new();

/// Balloon text capacity in bytes (UTF-8: roughly 80 Japanese characters).
pub const BALLOON_CAP: usize = 256;

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
    /// External gaze override active (touch tracking). While set, the render task feeds
    /// `gaze_*` into the avatar and the saccade animator is suppressed.
    gaze_active: AtomicU8,
    /// Gaze override, -100..=100 mapped to the avatar's -1.0..=1.0 range.
    gaze_h: AtomicI8,
    gaze_v: AtomicI8,
    /// Balloon text mailbox (empty string = no balloon). `balloon_version` bumps on
    /// every post so the render task can detect changes without holding the lock.
    balloon: CsMutex<RefCell<heapless::String<BALLOON_CAP>>>,
    balloon_version: AtomicU32,
    /// Face bytecode mailbox (`AVDS` v1; empty = reset to the embedded default face).
    face: CsMutex<RefCell<alloc::vec::Vec<u8>>>,
    face_version: AtomicU32,
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
            gaze_active: AtomicU8::new(0),
            gaze_h: AtomicI8::new(0),
            gaze_v: AtomicI8::new(0),
            balloon: CsMutex::new(RefCell::new(heapless::String::new())),
            balloon_version: AtomicU32::new(0),
            face: CsMutex::new(RefCell::new(alloc::vec::Vec::new())),
            face_version: AtomicU32::new(0),
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

    /// Set or clear the external gaze override. `h`/`v` are -1.0..=1.0.
    pub fn set_gaze(&self, gaze: Option<(f32, f32)>) {
        match gaze {
            Some((h, v)) => {
                self.gaze_h
                    .store((h.clamp(-1.0, 1.0) * 100.0) as i8, Ordering::Relaxed);
                self.gaze_v
                    .store((v.clamp(-1.0, 1.0) * 100.0) as i8, Ordering::Relaxed);
                self.gaze_active.store(1, Ordering::Release);
            }
            None => self.gaze_active.store(0, Ordering::Release),
        }
    }

    /// Post balloon text (empty string clears the balloon).
    pub fn post_balloon(&self, text: &str) {
        critical_section::with(|cs| {
            let mut b = self.balloon.borrow_ref_mut(cs);
            b.clear();
            let _ = b.push_str(text);
        });
        self.balloon_version.fetch_add(1, Ordering::Release);
    }

    /// Returns the balloon text if it changed since `*seen` (empty = clear balloon).
    pub fn take_balloon(&self, seen: &mut u32) -> Option<heapless::String<BALLOON_CAP>> {
        let version = self.balloon_version.load(Ordering::Acquire);
        if version == *seen {
            return None;
        }
        *seen = version;
        Some(critical_section::with(|cs| self.balloon.borrow_ref(cs).clone()))
    }

    /// Post face bytecode (empty slice = reset to the default face).
    pub fn post_face(&self, bytes: &[u8]) {
        critical_section::with(|cs| {
            let mut f = self.face.borrow_ref_mut(cs);
            f.clear();
            f.extend_from_slice(bytes);
        });
        self.face_version.fetch_add(1, Ordering::Release);
    }

    /// Returns the face bytecode if it changed since `*seen` (empty = reset).
    pub fn take_face(&self, seen: &mut u32) -> Option<alloc::vec::Vec<u8>> {
        let version = self.face_version.load(Ordering::Acquire);
        if version == *seen {
            return None;
        }
        *seen = version;
        Some(critical_section::with(|cs| self.face.borrow_ref(cs).clone()))
    }

    pub fn gaze(&self) -> Option<(f32, f32)> {
        if self.gaze_active.load(Ordering::Acquire) == 0 {
            return None;
        }
        Some((
            self.gaze_h.load(Ordering::Relaxed) as f32 / 100.0,
            self.gaze_v.load(Ordering::Relaxed) as f32 / 100.0,
        ))
    }
}
