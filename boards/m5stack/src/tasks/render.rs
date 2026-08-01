//! Avatar render task: owns the display and the stackchan-idf–style avatar, draws at
//! 30 fps, and applies face state (expression, mouth, gaze, balloon) from [`STATE`]
//! each frame.

use embassy_time::{Duration, Instant, Ticker};
use log::{info, warn};
use embedded_graphics::pixelcolor::Rgb565;
use m5stack_avatar_rs::stackchan::StackchanAvatar;

use crate::board::DisplayTy;
use crate::shared_state::STATE;

const FRAME_MS: u64 = 33; // ~30 fps

#[embassy_executor::task]
pub async fn render(display: &'static mut DisplayTy) {
    let mut avatar: StackchanAvatar<Rgb565> = StackchanAvatar::new();
    let mut balloon_seen = 0u32;
    let mut face_seen = 0u32;

    let mut ticker = Ticker::every(Duration::from_millis(FRAME_MS));
    loop {
        avatar.set_expression(STATE.expression());
        avatar.set_mouth_open(STATE.mouth_open_pct() as f32 / 100.0);
        // External gaze target; the avatar's saccade animator wanders around it.
        match STATE.gaze() {
            Some((h, v)) => avatar.set_gaze(h, v),
            None => avatar.set_gaze(0.0, 0.0),
        }
        if let Some(bytes) = STATE.take_face(&mut face_seen) {
            if bytes.is_empty() {
                avatar.reset_face_bytecode();
                info!("face bytecode reset to default");
            } else {
                match avatar.load_face_bytecode(&bytes) {
                    Ok(()) => info!("face bytecode loaded ({} bytes)", bytes.len()),
                    Err(e) => warn!("face bytecode rejected: {:?}", e),
                }
            }
        }
        if let Some(text) = STATE.take_balloon(&mut balloon_seen) {
            if text.is_empty() {
                avatar.clear_balloon();
            } else {
                avatar.set_balloon_text(&text, 0);
            }
        }
        avatar
            .tick(Instant::now().as_millis() as u32, display)
            .unwrap();
        ticker.next().await;
    }
}
