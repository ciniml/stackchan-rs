//! Idle behaviour task: posts a random head pose target every 1.5–4 s, mirroring the
//! C++ firmware's demo loop. Later phases (touch input, network API) will post targets
//! into the same [`STATE`], overriding this behaviour.

use embassy_time::{Instant, Timer};
use esp_hal::rng::Rng;
use log::debug;

use crate::head::HeadLimits;
use crate::shared_state::STATE;

/// Random target update interval bounds.
const RANDOM_INTERVAL_MIN_MS: u64 = 1500;
const RANDOM_INTERVAL_MAX_MS: u64 = 4000;

fn pick_random(rng: &Rng, min: u32, max: u32) -> u32 {
    let span = max.saturating_sub(min);
    if span == 0 {
        return min;
    }
    min + rng.random() % (span + 1)
}

#[embassy_executor::task]
pub async fn idle(rng: Rng, limits: HeadLimits) {
    loop {
        let span = RANDOM_INTERVAL_MAX_MS - RANDOM_INTERVAL_MIN_MS;
        let wait_ms = RANDOM_INTERVAL_MIN_MS + (rng.random() as u64) % span;
        Timer::after_millis(wait_ms).await;

        // Stay quiet while the user is interacting (touch input owns the head).
        if STATE.interaction_active(Instant::now().as_millis() as u32) {
            continue;
        }

        let pan = pick_random(&rng, limits.pan_min, limits.pan_max);
        let tilt = pick_random(&rng, limits.tilt_min, limits.tilt_max);
        STATE.set_head_target(pan, tilt);
        debug!("idle: new target pan={} tilt={} (next in {}ms)", pan, tilt, wait_ms);
    }
}
