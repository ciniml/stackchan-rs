//! Avatar render task: owns the display and the avatar, draws at 30 fps, and applies
//! face state (expression) from [`STATE`] each frame.

use embassy_time::{Duration, Ticker};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use m5stack_avatar_rs::components::balloon::BalloonContext;
use m5stack_avatar_rs::components::face::DrawContext;
use m5stack_avatar_rs::{Avatar, BasicPaletteKey, Palette};

use crate::board::DisplayTy;
use crate::shared_state::STATE;

const FRAME_MS: u64 = 33; // ~30 fps

type AvatarString = heapless::String<64>;

struct EmbassyTimer;
impl m5stack_avatar_rs::Timer for EmbassyTimer {
    fn timestamp_milliseconds(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
}

#[embassy_executor::task]
pub async fn render(display: &'static mut DisplayTy) {
    let mut context: DrawContext<Rgb565, AvatarString> = DrawContext::default();
    context.palette.set_color(&BasicPaletteKey::Primary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Secondary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Background, Rgb565::BLACK);
    context.palette.set_color(&BasicPaletteKey::BalloonForeground, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::BalloonBackground, Rgb565::BLACK);
    context.set_text(Some("Rusty Stack-chan!"));
    context.expression = STATE.expression();
    let mut avatar: Avatar<'static, Rgb565, AvatarString> = Avatar::new(context, 30);
    let timer = EmbassyTimer;

    let mut ticker = Ticker::every(Duration::from_millis(FRAME_MS));
    loop {
        let context = avatar.context();
        context.expression = STATE.expression();
        context.mouth_open_ratio = STATE.mouth_open_pct() as f32 / 100.0;
        avatar.run(display, &timer).unwrap();
        ticker.next().await;
    }
}
