//! M5Stack Core2 firmware for the Stackchan project.
//!
//! Migrated to esp-hal 1.x. Chip drivers (AXP192 PMIC, ILI9341 LCD) live in `m5drivers-rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::{Instant, Rate};
use esp_println::println;
use m5drivers_rs::axp192::{self, Axp192, Axp192Gpio};
use m5drivers_rs::ili9341::Ili9341;
use m5stack_avatar_rs::components::balloon::BalloonContext;
use m5stack_avatar_rs::components::eye::GazeContext;
use m5stack_avatar_rs::components::face::DrawContext;
use m5stack_avatar_rs::{Avatar, BasicPaletteKey, Expression, Palette};
use stackchan_rs::path_generator::PathGenerator;

const SERVO_PERIOD_HZ: u32 = 50; // 20 ms
const SERVO_RESOLUTION: u32 = 4096;

fn degree_1024_to_pwm_count(degree_1024: u32) -> u16 {
    // 0.5..=2.5 ms pulse over a 20 ms period mapped onto 0..=4095 timer ticks.
    let period_us = 1_000_000 / SERVO_PERIOD_HZ; // 20_000
    (((SERVO_RESOLUTION * 2 * degree_1024) / (180 * 1024) + SERVO_RESOLUTION / 2) / (period_us / 1000)) as u16
}

struct InstantTimer;
impl m5stack_avatar_rs::Timer for InstantTimer {
    fn timestamp_milliseconds(&self) -> u64 {
        Instant::now().duration_since_epoch().as_millis()
    }
}

type AvatarString = heapless::String<64>;

#[main]
fn main() -> ! {
    let peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 100 * 1024);
    let mut delay = Delay::new();

    println!("[m5stack-core2] booting");

    // ---- I2C0 for the AXP192 PMIC ------------------------------------------
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO21)
    .with_scl(peripherals.GPIO22);

    let axp = RefCell::new(Axp192::new(i2c, 0x34));
    {
        let mut a = axp.borrow_mut();
        a.reset_core2(|ms| delay.delay_millis(ms)).unwrap();
    }
    println!("[m5stack-core2] AXP192 ready");

    // ---- SPI2 for the ILI9341 LCD ------------------------------------------
    let spi_bus = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(SpiMode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO18)
    .with_mosi(peripherals.GPIO23)
    .with_miso(peripherals.GPIO38);
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let spi_dev = ExclusiveDevice::new(spi_bus, cs, Delay::new()).unwrap();

    let dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let lcd_rst = Axp192Gpio::new(&axp, axp192::GpioNum::Gpio4);

    let mut display = Ili9341::new(spi_dev, dc, lcd_rst);
    display.init(&mut delay).unwrap();
    display.fill(0, 0, 320, 240, Rgb565::BLACK).unwrap();
    println!("[m5stack-core2] ILI9341 ready");

    // ---- MCPWM0 for the pan/tilt servos ------------------------------------
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(2)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    let mut pwm_tilt: esp_hal::mcpwm::operator::PwmPin<'_, MCPWM0<'_>, 0, true> = mcpwm
        .operator0
        .with_pin_a(peripherals.GPIO14, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut pwm_pan: esp_hal::mcpwm::operator::PwmPin<'_, MCPWM0<'_>, 1, true> = mcpwm
        .operator1
        .with_pin_a(peripherals.GPIO13, PwmPinConfig::UP_ACTIVE_HIGH);
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            SERVO_RESOLUTION as u16,
            PwmWorkingMode::Increase,
            Rate::from_hz(SERVO_PERIOD_HZ),
        )
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);
    pwm_tilt.set_timestamp(degree_1024_to_pwm_count(70 * 1024));
    pwm_pan.set_timestamp(degree_1024_to_pwm_count(90 * 1024));

    let mut path_gen_pan = PathGenerator::<256>::new(90 * 1024, 1.0 * 1024.0, 30.0 * 1024.0);
    let mut path_gen_tilt = PathGenerator::<256>::new(70 * 1024, 1.0 * 1024.0, 30.0 * 1024.0);

    // ---- Avatar setup ------------------------------------------------------
    let mut context: DrawContext<Rgb565, AvatarString> = DrawContext::default();
    context.palette.set_color(&BasicPaletteKey::Primary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Secondary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Background, Rgb565::BLACK);
    context.palette.set_color(&BasicPaletteKey::BalloonForeground, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::BalloonBackground, Rgb565::BLACK);
    context.set_text(Some("Rusty Stack-chan!"));
    context.expression = Expression::Happy;
    let mut avatar = Avatar::new(context, 30);
    let tick_timer = InstantTimer;

    // ---- Main loop: render avatar + step servo path generators -------------
    loop {
        avatar.run(&mut display, &tick_timer).unwrap();

        let h = avatar.context().horizontal();
        let v = avatar.context().vertical();
        if !path_gen_pan.is_moving() {
            path_gen_pan.begin_move_to((h * 22.5 * 1024.0) as u32 + 90 * 1024);
        }
        if !path_gen_tilt.is_moving() {
            path_gen_tilt.begin_move_to(65 * 1024 - (v * 5.0 * 1024.0) as u32);
        }
        let pan = path_gen_pan.step_next().clamp(45 * 1024, 135 * 1024);
        let tilt = path_gen_tilt.step_next().clamp(60 * 1024, 70 * 1024);
        pwm_pan.set_timestamp(degree_1024_to_pwm_count(pan));
        pwm_tilt.set_timestamp(degree_1024_to_pwm_count(tilt));
    }
}
