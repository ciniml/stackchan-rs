//! M5Stack Core2 board support (selected by `--features core2`).
//!
//! Chip drivers (AXP192 PMIC, ILI9341 LCD) live in `m5drivers-rs`.
//! Pan/tilt: two hobby PWM servos on MCPWM0 (pan=GPIO13, tilt=GPIO14).

use core::cell::RefCell;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{Async, Blocking};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::i2s::master::{Channels, Config as I2sConfig, DataFormat, I2s, I2sTx};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::mcpwm::operator::{PwmPin, PwmPinConfig};
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;
use esp_hal::rng::Rng;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use m5drivers_rs::axp192::{self, Axp192, Axp192Gpio};
use m5drivers_rs::ili9341::Ili9341;
use m5drivers_rs::{FT6336_DEFAULT_ADDR, Ft6336};
use static_cell::StaticCell;

use crate::head::{DEG, HeadDriver, HeadLimits};

const SERVO_PERIOD_HZ: u32 = 50; // 20 ms
const SERVO_RESOLUTION: u32 = 4096;

const HEAD_LIMITS: HeadLimits = HeadLimits {
    pan_min: 45 * DEG,
    pan_max: 135 * DEG,
    tilt_min: 60 * DEG,
    tilt_max: 70 * DEG,
};

fn degree_1024_to_pwm_count(degree_1024: u32) -> u16 {
    // 0.5..=2.5 ms pulse over a 20 ms period mapped onto 0..=4095 timer ticks.
    let period_us = 1_000_000 / SERVO_PERIOD_HZ; // 20_000
    (((SERVO_RESOLUTION * 2 * degree_1024) / (180 * 1024) + SERVO_RESOLUTION / 2)
        / (period_us / 1000)) as u16
}

/// Pan/tilt head on MCPWM hobby servos. Always present — there is no feedback channel
/// on a PWM servo, so presence cannot be probed.
pub struct PwmHead {
    pan: PwmPin<'static, MCPWM0<'static>, 1, true>,
    tilt: PwmPin<'static, MCPWM0<'static>, 0, true>,
}

impl HeadDriver for PwmHead {
    fn set_pose(&mut self, pan: u32, tilt: u32, _period_ms: u16) {
        self.pan.set_timestamp(degree_1024_to_pwm_count(pan));
        self.tilt.set_timestamp(degree_1024_to_pwm_count(tilt));
    }
}

// ---- Static placement for the large stateful objects ----------------------
// Kept in BSS via `static_cell::StaticCell` so they don't sit in any stack frame
// (see the note in cores3.rs; the same stack-overflow hazard applies here).
// The internal I2C bus (AXP192 + FT6336 touch) is shared through a RefCell; all users
// stay on core 0 and never hold a borrow across an await point.
type I2cBusTy = I2c<'static, Blocking>;
type I2cBusCell = RefCell<I2cBusTy>;
type I2cDeviceTy = I2cRefCellDevice<'static, I2cBusTy>;
type Axp192Cell = RefCell<Axp192<I2cDeviceTy>>;
type SpiDeviceTy = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;
type LcdRstTy = Axp192Gpio<'static, I2cDeviceTy>;
pub type DisplayTy = Ili9341<SpiDeviceTy, Output<'static>, LcdRstTy>;
pub type HeadTy = PwmHead;
pub type TouchTy = Ft6336<I2cDeviceTy>;
pub type SpeakerTy = I2sTx<'static, Async>;

static I2C_BUS: StaticCell<I2cBusCell> = StaticCell::new();
static AXP: StaticCell<Axp192Cell> = StaticCell::new();
static DISPLAY: StaticCell<DisplayTy> = StaticCell::new();

/// On Core2 the head driver owns its MCPWM pins outright and is `Send`, so it can move
/// to the servo core as-is. `HeadParts`/`make_head` exist to match the CoreS3 surface.
pub type HeadParts = PwmHead;

pub fn make_head(parts: HeadParts) -> PwmHead {
    parts
}

pub struct Board {
    pub display: &'static mut DisplayTy,
    pub head_parts: HeadParts,
    pub head_present: bool,
    pub touch: Option<TouchTy>,
    pub speaker: Option<SpeakerTy>,
    pub limits: HeadLimits,
    pub rng: Rng,
    pub cpu_ctrl: esp_hal::peripherals::CPU_CTRL<'static>,
    pub sw_int1: esp_hal::interrupt::software::SoftwareInterrupt<'static, 1>,
    pub wifi: esp_hal::peripherals::WIFI<'static>,
    pub flash: esp_hal::peripherals::FLASH<'static>,
}

pub fn init() -> Board {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 112 * 1024);
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let mut delay = Delay::new();

    info!("booting (Core2)");

    // ---- esp-rtos scheduler / embassy time driver (TIMG0) ------------------
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ---- Internal I2C bus (AXP192 + FT6336): SDA=GPIO21, SCL=GPIO22 --------
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO21)
    .with_scl(peripherals.GPIO22);
    let i2c_bus: &'static I2cBusCell = I2C_BUS.init(RefCell::new(i2c));

    let axp: &'static Axp192Cell = AXP.init(RefCell::new(Axp192::new(
        I2cRefCellDevice::new(i2c_bus),
        0x34,
    )));
    {
        let mut a = axp.borrow_mut();
        a.reset_core2(|ms| delay.delay_millis(ms)).unwrap();
    }
    info!("AXP192 ready");

    // ---- FT6336 touch controller (poll-driven; INT=GPIO39 unused) ----------
    let mut touch = Ft6336::new(I2cRefCellDevice::new(i2c_bus), FT6336_DEFAULT_ADDR);
    let touch = match touch.vendor_id() {
        Ok(id) => {
            info!("FT6336 touch ready (vendor 0x{:02X})", id);
            Some(touch)
        }
        Err(e) => {
            log::warn!("FT6336 not responding ({:?}); touch input disabled", e);
            None
        }
    };

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
    let lcd_rst = Axp192Gpio::new(axp, axp192::GpioNum::Gpio4);

    let display: &'static mut DisplayTy = DISPLAY.init(Ili9341::new(spi_dev, dc, lcd_rst));
    display.init(&mut delay).unwrap();
    display.fill(0, 0, 320, 240, Rgb565::BLACK).unwrap();
    info!("ILI9341 ready");

    // ---- MCPWM0 for the pan/tilt servos ------------------------------------
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(2)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    let pwm_tilt = mcpwm
        .operator0
        .with_pin_a(peripherals.GPIO14, PwmPinConfig::UP_ACTIVE_HIGH);
    let pwm_pan = mcpwm
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

    let mut head = PwmHead {
        pan: pwm_pan,
        tilt: pwm_tilt,
    };
    head.set_pose(HEAD_LIMITS.pan_center(), HEAD_LIMITS.tilt_center(), 0);
    info!("MCPWM servos ready");

    // ---- Speaker: NS4168 on I2S0, BCK=GPIO12, WS=GPIO0, DOUT=GPIO2 ---------
    // The NS4168 enable line hangs off AXP192 GPIO2 (PMIC GPIO, unrelated to ESP GPIO2).
    let speaker = {
        if let Err(e) = axp
            .borrow_mut()
            .set_gpio_output(axp192::GpioNum::Gpio2, true)
        {
            log::warn!("speaker enable (AXP192 GPIO2) failed: {:?}", e);
        }
        let (_rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(0, 32768);
        match I2s::new(
            peripherals.I2S0,
            peripherals.DMA_I2S0,
            I2sConfig::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(crate::tasks::audio::SAMPLE_RATE_HZ))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        ) {
            Ok(i2s) => {
                let tx = i2s
                    .into_async()
                    .i2s_tx
                    .with_bclk(peripherals.GPIO12)
                    .with_ws(peripherals.GPIO0)
                    .with_dout(peripherals.GPIO2)
                    .build(tx_descriptors);
                info!("NS4168 speaker ready");
                Some(tx)
            }
            Err(e) => {
                log::warn!("I2S init failed ({:?}); speaker disabled", e);
                None
            }
        }
    };

    Board {
        display,
        head_parts: head,
        head_present: true,
        touch,
        speaker,
        limits: HEAD_LIMITS,
        rng: Rng::new(),
        cpu_ctrl: peripherals.CPU_CTRL,
        sw_int1: sw_int.software_interrupt1,
        wifi: peripherals.WIFI,
        flash: peripherals.FLASH,
    }
}
