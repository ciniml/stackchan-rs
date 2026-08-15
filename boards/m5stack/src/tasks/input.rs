//! Touch input task (core 0): polls the FT6336 touch controller and turns touches into
//! avatar/head commands, mirroring the C++ firmware's demo-loop input handling:
//!
//! - Touch **down** (edge): cycle the avatar expression.
//! - While **touching**: the eyes and head follow the touch point — the gaze tracks the
//!   finger directly, screen X maps to pan and screen Y to tilt — and idle behaviour is
//!   suppressed for a hold-off period. On release the gaze returns to the saccade
//!   animator.

use embassy_time::{Duration, Instant, Ticker};
use log::{debug, warn};
use m5drivers_rs::TouchPoint;

use crate::board::TouchTy;
use crate::head::{DEG, HeadLimits};
use crate::shared_state::{STATE, Sound};

/// Touch poll period. 33 ms ≈ 30 Hz, plenty for finger tracking.
const POLL_MS: u64 = 33;
/// How long idle behaviour stays suppressed after the last touch.
const INTERACT_HOLD_MS: u32 = 3000;
/// Don't re-post a follow target unless it moved at least this much (1°).
const FOLLOW_DEADBAND: u32 = DEG;

/// LCD dimensions used for the touch→pose mapping. On Core2 the touch surface extends
/// to y=279 (virtual button strip); clamping to the LCD area folds that strip onto the
/// bottom edge, which is fine for head-follow purposes.
const SCREEN_W: u32 = 320;
const SCREEN_H: u32 = 240;

/// Map a touch point to a head pose. Touching the left edge turns the head fully toward
/// `pan_max`, the right edge toward `pan_min`; the top edge tilts to `tilt_max`, the
/// bottom to `tilt_min`. (Flip here if the servo orientation on the actual hardware
/// disagrees.)
fn touch_to_pose(p: TouchPoint, limits: &HeadLimits) -> (u32, u32) {
    let x = (p.x as u32).min(SCREEN_W - 1);
    let y = (p.y as u32).min(SCREEN_H - 1);
    let pan_range = limits.pan_max - limits.pan_min;
    let tilt_range = limits.tilt_max - limits.tilt_min;
    let pan = limits.pan_min + (SCREEN_W - 1 - x) * pan_range / (SCREEN_W - 1);
    let tilt = limits.tilt_min + (SCREEN_H - 1 - y) * tilt_range / (SCREEN_H - 1);
    (pan, tilt)
}

#[embassy_executor::task]
pub async fn input(mut touch: TouchTy, limits: HeadLimits) {
    let mut was_touching = false;
    let mut last_posted: Option<(u32, u32)> = None;
    let mut ticker = Ticker::every(Duration::from_millis(POLL_MS));
    loop {
        ticker.next().await;

        let point = match touch.touch_point() {
            Ok(p) => p,
            Err(e) => {
                warn!("touch read failed: {:?}", e);
                continue;
            }
        };

        match point {
            Some(p) => {
                let now_ms = Instant::now().as_millis() as u32;
                STATE.note_interaction(now_ms, INTERACT_HOLD_MS);

                // Eyes track the finger while touching.
                let gx = (p.x as f32 / (SCREEN_W - 1) as f32) * 2.0 - 1.0;
                let gy = ((p.y as f32).min((SCREEN_H - 1) as f32) / (SCREEN_H - 1) as f32)
                    * 2.0
                    - 1.0;
                STATE.set_gaze(Some((gx, gy)));

                if !was_touching {
                    STATE.cycle_expression();
                    STATE.post_sound(Sound::Blip);
                    debug!("touch down at ({}, {}): expression -> {:?}", p.x, p.y, STATE.expression());
                }

                let pose = touch_to_pose(p, &limits);
                let moved = last_posted.is_none_or(|(pan, tilt)| {
                    pan.abs_diff(pose.0) >= FOLLOW_DEADBAND
                        || tilt.abs_diff(pose.1) >= FOLLOW_DEADBAND
                });
                if moved {
                    STATE.set_head_target(pose.0, pose.1);
                    last_posted = Some(pose);
                }
                was_touching = true;
            }
            None => {
                if was_touching {
                    STATE.set_gaze(None);
                }
                was_touching = false;
                last_posted = None;
            }
        }
    }
}
