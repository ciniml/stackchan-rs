//! Head motion task: consumes pose targets from [`STATE`] on a fixed 20 ms tick,
//! smooths them with trapezoid velocity profiles, and commands the board's servos.
//!
//! Each SCS command asks the servo to reach the stepped position in **2×** the tick
//! period. Because a fresh command arrives every tick, the servo never enters the
//! deceleration tail of a segment — velocity stays continuous across ticks (the same
//! overlap trick the C++ firmware's 20 ms servo loop uses). Commanding the reached
//! target once and then going quiet avoids needless bus traffic and hold jitter.

use embassy_time::{Duration, Ticker};
use log::debug;
use stackchan_rs::path_generator::PathGenerator;

use crate::board::HeadTy;
use crate::head::{DEG, HeadDriver, HeadLimits};
use crate::shared_state::STATE;

/// Servo command update period (matches the C++ firmware's 20 ms servo loop).
const SERVO_TICK_MS: u64 = 20;
/// Move time sent with each command; 2× the tick so consecutive commands overlap.
const SERVO_MOVE_TIME_MS: u16 = (SERVO_TICK_MS * 2) as u16;
/// Path generator tuning in 1/1024-degree units per 20 ms tick:
/// ≈0.24°/tick² acceleration (600°/s²), ≈2.4°/tick top speed (120°/s).
const PATH_MAX_ACCEL: f32 = 0.24 * DEG as f32;
const PATH_MAX_VEL: f32 = 2.4 * DEG as f32;

#[embassy_executor::task]
pub async fn servo(mut head: HeadTy, limits: HeadLimits) {
    let mut pan_path =
        PathGenerator::<256>::new(limits.pan_center(), PATH_MAX_ACCEL, PATH_MAX_VEL);
    let mut tilt_path =
        PathGenerator::<256>::new(limits.tilt_center(), PATH_MAX_ACCEL, PATH_MAX_VEL);
    let (_, _, mut last_seq) = STATE.head_target();
    // Send one final command after motion ends, then stay quiet until the next target.
    let mut settled = false;

    let mut ticker = Ticker::every(Duration::from_millis(SERVO_TICK_MS));
    loop {
        ticker.next().await;

        let (pan_target, tilt_target, seq) = STATE.head_target();
        if seq != last_seq {
            last_seq = seq;
            let pan_target = limits.clamp_pan(pan_target);
            let tilt_target = limits.clamp_tilt(tilt_target);
            pan_path.begin_move_to(pan_target);
            tilt_path.begin_move_to(tilt_target);
            settled = false;
            debug!("new head target pan={} tilt={}", pan_target, tilt_target);
        }

        if settled {
            continue;
        }

        let moving = pan_path.is_moving() || tilt_path.is_moving();
        let pan_pos = if pan_path.is_moving() {
            pan_path.step_next()
        } else {
            pan_path.get_target_position()
        };
        let tilt_pos = if tilt_path.is_moving() {
            tilt_path.step_next()
        } else {
            tilt_path.get_target_position()
        };
        head.set_pose(
            limits.clamp_pan(pan_pos),
            limits.clamp_tilt(tilt_pos),
            SERVO_MOVE_TIME_MS,
        );
        if !moving {
            settled = true;
        }
    }
}
