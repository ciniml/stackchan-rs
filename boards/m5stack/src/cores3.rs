//! M5Stack CoreS3 / CoreS3 SE firmware (selected by `--features cores3`).
//!
//! Chip drivers (AXP2101 PMIC, AW9523B GPIO expander, ILI9342 LCD) live in `m5drivers-rs`.
//! Servos are not driven yet — Stackchan's servo wiring on CoreS3 depends on which Grove
//! port / hat is in use, so add that once the hardware target is fixed.

use core::cell::RefCell;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::{Instant, Rate};
use esp_println::println;
use m5drivers_rs::aw9523::Aw9523Reg;
use m5drivers_rs::{
    AW9523_DEFAULT_ADDR, AXP2101_DEFAULT_ADDR, Aw9523, Aw9523Pin, Axp2101, Ili9342, Pin,
};
use m5stack_avatar_rs::components::balloon::BalloonContext;
use m5stack_avatar_rs::components::face::DrawContext;
use m5stack_avatar_rs::{Avatar, BasicPaletteKey, Expression, Palette};

struct InstantTimer;
impl m5stack_avatar_rs::Timer for InstantTimer {
    fn timestamp_milliseconds(&self) -> u64 {
        Instant::now().duration_since_epoch().as_millis()
    }
}

type AvatarString = heapless::String<64>;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 100 * 1024);
    let mut delay = Delay::new();

    println!("[m5stack-cores3] booting");

    // ---- Internal I2C bus (AXP2101 + AW9523B): SDA=GPIO12, SCL=GPIO11 -------
    let i2c_bus = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO12)
    .with_scl(peripherals.GPIO11);
    let i2c_bus = RefCell::new(i2c_bus);

    // ---- AW9523B init (matches M5GFX `M5GFX.cpp` CoreS3 setup) -------------
    let aw = RefCell::new(Aw9523::new(
        I2cRefCellDevice::new(&i2c_bus),
        AW9523_DEFAULT_ADDR,
    ));
    {
        let mut a = aw.borrow_mut();
        a.write_reg(Aw9523Reg::P0Direction, 0b0001_1000).unwrap();
        a.write_reg(Aw9523Reg::P1Direction, 0b0000_1100).unwrap();
        a.write_reg(Aw9523Reg::GlobalCtl, 0b0001_0000).unwrap();
        a.write_reg(Aw9523Reg::P0Output, 0b0000_0111).unwrap();
        a.write_reg(Aw9523Reg::P1Output, 0b0010_0011).unwrap();
    }
    println!("[m5stack-cores3] AW9523B configured");

    // ---- AXP2101: chip ID + CoreS3 power-on + backlight --------------------
    let mut axp = Axp2101::new(I2cRefCellDevice::new(&i2c_bus), AXP2101_DEFAULT_ADDR);
    match axp.chip_id() {
        Ok(id) => println!("[m5stack-cores3] AXP2101 chip id = 0x{:02X} (expect 0x4A)", id),
        Err(e) => panic!("AXP2101 chip id read failed: {:?}", e),
    }
    axp.power_on_cores3().unwrap();
    axp.set_backlight_brightness(160).unwrap();

    // ---- LCD SPI (mode 0, 40 MHz; 3-wire — MISO unused) --------------------
    let spi_bus = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(SpiMode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO36)
    .with_mosi(peripherals.GPIO37);
    let cs = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
    let spi_dev = ExclusiveDevice::new(spi_bus, cs, Delay::new()).unwrap();

    let dc = Output::new(peripherals.GPIO35, Level::Low, OutputConfig::default());
    let rst = Aw9523Pin::new(&aw, Pin::p1(5));

    let mut display = Ili9342::new(spi_dev, dc, rst);
    display.init(&mut delay).unwrap();
    display.fill(0, 0, 320, 240, Rgb565::BLACK).unwrap();
    println!("[m5stack-cores3] ILI9342 ready");

    // ---- Avatar ------------------------------------------------------------
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

    loop {
        avatar.run(&mut display, &tick_timer).unwrap();
    }
}
