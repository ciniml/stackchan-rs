//! M5Stack CoreS3 / CoreS3 SE firmware (selected by `--features cores3`).
//!
//! Chip drivers (AXP2101 PMIC, AW9523B GPIO expander, ILI9342 LCD) live in `m5drivers-rs`.
//!
//! Pan/tilt: two SCS0009 servos on UART2 / Grove Port C, driven through `scs-servo`. The
//! main loop picks a random target every 1.5–4 s and feeds the smoothed path back to the
//! servos at a 50 ms cadence (catching up if the avatar render holds the loop longer).

use core::cell::RefCell;
use core::time::Duration as CoreDuration;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::rng::Rng;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::uart::{Config as UartConfig, Uart, UartRx, UartTx};
use esp_println::println;
use m5drivers_rs::aw9523::Aw9523Reg;
use m5drivers_rs::{
    AW9523_DEFAULT_ADDR, AXP2101_DEFAULT_ADDR, Aw9523, Aw9523Pin, Axp2101, Ili9342, Pin,
};
use m5stack_avatar_rs::components::balloon::BalloonContext;
use m5stack_avatar_rs::components::face::DrawContext;
use m5stack_avatar_rs::{Avatar, BasicPaletteKey, Expression, Palette};
use scs_servo::device::ServoControl;
use scs_servo::device::scs0009::Scs0009ServoControl;
use scs_servo::protocol::{ProtocolMasterConfig, StreamReader, StreamWriter};
use stackchan_rs::path_generator::PathGenerator;
use static_cell::StaticCell;

// ---- SCS bus / servo configuration -----------------------------------------
const SCS_BAUD: u32 = 1_000_000;
const PAN_ID: u8 = 1;
const TILT_ID: u8 = 2;
/// Center of the SCS0009 1024-step range (≈ 180°).
const SCS_CENTER: u32 = 0x200;
/// Approximate ±45° in SCS units (1024 steps / 360° ≈ 2.84 unit/deg).
const PAN_HALF_RANGE: u32 = 128;
/// Approximate ±15°.
const TILT_HALF_RANGE: u32 = 43;
/// Servo command update period.
const SERVO_TICK_MS: u64 = 50;
/// Random target update interval bounds.
const RANDOM_INTERVAL_MIN_MS: u64 = 1500;
const RANDOM_INTERVAL_MAX_MS: u64 = 4000;

struct InstantTimer;
impl m5stack_avatar_rs::Timer for InstantTimer {
    fn timestamp_milliseconds(&self) -> u64 {
        Instant::now().duration_since_epoch().as_millis()
    }
}

// ---- scs-servo Timer / Instant adapters using esp-hal time ----------------
struct ScsClock;
struct ScsInstant(Instant);
impl scs_servo::device::Instant for ScsInstant {
    fn elapsed(&self) -> CoreDuration {
        CoreDuration::from_micros(self.0.elapsed().as_micros())
    }
}
impl scs_servo::device::Timer for ScsClock {
    type Instant = ScsInstant;
    fn now() -> Self::Instant {
        ScsInstant(Instant::now())
    }
}

// ---- StreamReader / StreamWriter that share one UART via RefCell ----------
struct UartTxRef<'a, 'd>(&'a RefCell<UartTx<'d, Blocking>>);
struct UartRxRef<'a, 'd>(&'a RefCell<UartRx<'d, Blocking>>);

impl<'a, 'd> StreamWriter for UartTxRef<'a, 'd> {
    type Error = esp_hal::uart::TxError;
    fn write(&mut self, data: &[u8]) -> nb::Result<usize, Self::Error> {
        let mut tx = self.0.borrow_mut();
        match tx.write(data) {
            Ok(0) => Err(nb::Error::WouldBlock),
            Ok(n) => Ok(n),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl<'a, 'd> StreamReader for UartRxRef<'a, 'd> {
    type Error = esp_hal::uart::RxError;
    fn read(&mut self, data: &mut [u8]) -> nb::Result<usize, Self::Error> {
        let mut rx = self.0.borrow_mut();
        match rx.read_buffered(data) {
            Ok(0) => Err(nb::Error::WouldBlock),
            Ok(n) => Ok(n),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

type AvatarString = heapless::String<64>;

// ---- Static placement for the large stateful objects ----------------------
// These live in BSS via `static_cell::StaticCell` so they don't sit in `main`'s stack
// frame. opt-level=3 used to inline `main` to a single frame large enough to overflow
// the stack inside esp-hal's `Uart::new`; moving them to BSS makes the layout stable.
type I2cBusTy = I2c<'static, Blocking>;
type I2cBusCell = RefCell<I2cBusTy>;
type I2cDeviceTy = I2cRefCellDevice<'static, I2cBusTy>;
type Aw9523Cell = RefCell<Aw9523<I2cDeviceTy>>;
type Axp2101Ty = Axp2101<I2cDeviceTy>;
type SpiBusTy = Spi<'static, Blocking>;
type SpiDeviceTy = ExclusiveDevice<SpiBusTy, Output<'static>, Delay>;
type LcdRstTy = Aw9523Pin<'static, I2cDeviceTy>;
type DisplayTy = Ili9342<SpiDeviceTy, Output<'static>, LcdRstTy>;
type AvatarTy = Avatar<'static, Rgb565, AvatarString>;
type UartTxCell = RefCell<UartTx<'static, Blocking>>;
type UartRxCell = RefCell<UartRx<'static, Blocking>>;
type ScsServoTy = Scs0009ServoControl<
    UartRxRef<'static, 'static>,
    UartTxRef<'static, 'static>,
    ScsClock,
>;

static I2C_BUS: StaticCell<I2cBusCell> = StaticCell::new();
static AW_CELL: StaticCell<Aw9523Cell> = StaticCell::new();
static AXP: StaticCell<Axp2101Ty> = StaticCell::new();
static DISPLAY: StaticCell<DisplayTy> = StaticCell::new();
static AVATAR: StaticCell<AvatarTy> = StaticCell::new();
static UART_TX: StaticCell<UartTxCell> = StaticCell::new();
static UART_RX: StaticCell<UartRxCell> = StaticCell::new();
static PAN: StaticCell<ScsServoTy> = StaticCell::new();
static TILT: StaticCell<ScsServoTy> = StaticCell::new();

fn pick_random_target(rng: &Rng, center: u32, half_range: u32) -> u32 {
    let span = half_range.saturating_mul(2);
    if span == 0 {
        return center;
    }
    let offset = rng.random() % span;
    center.saturating_sub(half_range).saturating_add(offset)
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 64 * 1024);
    let mut delay = Delay::new();

    println!("[m5stack-cores3] booting");

    // ---- Internal I2C bus (AXP2101 + AW9523B): SDA=GPIO12, SCL=GPIO11 -------
    let i2c_bus_local = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO12)
    .with_scl(peripherals.GPIO11);
    let i2c_bus: &'static I2cBusCell = I2C_BUS.init(RefCell::new(i2c_bus_local));

    // ---- AW9523B init (matches M5GFX `M5GFX.cpp` CoreS3 setup) -------------
    let aw: &'static Aw9523Cell = AW_CELL.init(RefCell::new(Aw9523::new(
        I2cRefCellDevice::new(i2c_bus),
        AW9523_DEFAULT_ADDR,
    )));
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
    let axp: &'static mut Axp2101Ty = AXP.init(Axp2101::new(
        I2cRefCellDevice::new(i2c_bus),
        AXP2101_DEFAULT_ADDR,
    ));
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
    let rst = Aw9523Pin::new(aw, Pin::p1(5));

    let display: &'static mut DisplayTy = DISPLAY.init(Ili9342::new(spi_dev, dc, rst));
    display.init(&mut delay).unwrap();
    display.fill(0, 0, 320, 240, Rgb565::BLACK).unwrap();
    println!("[m5stack-cores3] ILI9342 ready");

    // ---- UART2 for the SCS0009 bus (Grove Port C: TX=GPIO17, RX=GPIO18) ----
    let uart_local = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(SCS_BAUD),
    )
    .unwrap()
    .with_tx(peripherals.GPIO17)
    .with_rx(peripherals.GPIO18);
    let (uart_rx_local, uart_tx_local) = uart_local.split();
    let uart_tx: &'static UartTxCell = UART_TX.init(RefCell::new(uart_tx_local));
    let uart_rx: &'static UartRxCell = UART_RX.init(RefCell::new(uart_rx_local));

    let scs_timeout = CoreDuration::from_millis(20);
    let pan: &'static mut ScsServoTy = PAN.init(Scs0009ServoControl::<_, _, ScsClock>::new(
        PAN_ID,
        UartRxRef(uart_rx),
        UartTxRef(uart_tx),
        ProtocolMasterConfig { echo_back: true },
        scs_timeout,
    ));
    let tilt: &'static mut ScsServoTy = TILT.init(Scs0009ServoControl::<_, _, ScsClock>::new(
        TILT_ID,
        UartRxRef(uart_rx),
        UartTxRef(uart_tx),
        ProtocolMasterConfig { echo_back: true },
        scs_timeout,
    ));
    // Probe each servo via `output_enable`. If the call times out (e.g. the SCS bus is not
    // wired up, or the servo IDs differ), keep running the avatar without driving servos.
    let pan_present = match pan.output_enable() {
        Ok(()) => true,
        Err(e) => {
            println!(
                "[m5stack-cores3] pan (id={}) not responding ({:?}); disabling pan servo",
                PAN_ID, e
            );
            false
        }
    };
    let tilt_present = match tilt.output_enable() {
        Ok(()) => true,
        Err(e) => {
            println!(
                "[m5stack-cores3] tilt (id={}) not responding ({:?}); disabling tilt servo",
                TILT_ID, e
            );
            false
        }
    };
    let servos_present = pan_present || tilt_present;
    if !servos_present {
        println!("[m5stack-cores3] no SCS0009 detected; avatar only");
    }

    let rng = Rng::new();
    let mut pan_path = PathGenerator::<256>::new(SCS_CENTER, 2.0, 8.0);
    let mut tilt_path = PathGenerator::<256>::new(SCS_CENTER, 2.0, 8.0);
    let mut last_tick = Instant::now();
    let mut next_random = Instant::now();

    // ---- Avatar ------------------------------------------------------------
    let mut context: DrawContext<Rgb565, AvatarString> = DrawContext::default();
    context.palette.set_color(&BasicPaletteKey::Primary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Secondary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Background, Rgb565::BLACK);
    context.palette.set_color(&BasicPaletteKey::BalloonForeground, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::BalloonBackground, Rgb565::BLACK);
    context.set_text(Some("Rusty Stack-chan!"));
    context.expression = Expression::Happy;
    let avatar: &'static mut AvatarTy = AVATAR.init(Avatar::new(context, 30));
    let tick_timer = InstantTimer;

    loop {
        avatar.run(display, &tick_timer).unwrap();

        if !servos_present {
            continue;
        }

        // Pick a new random target when the random timer fires.
        if Instant::now() >= next_random {
            let pan_target = pick_random_target(&rng, SCS_CENTER, PAN_HALF_RANGE);
            let tilt_target = pick_random_target(&rng, SCS_CENTER, TILT_HALF_RANGE);
            pan_path.begin_move_to(pan_target);
            tilt_path.begin_move_to(tilt_target);
            let span = RANDOM_INTERVAL_MAX_MS - RANDOM_INTERVAL_MIN_MS;
            let extra = (rng.random() as u64) % span;
            next_random =
                Instant::now() + Duration::from_millis(RANDOM_INTERVAL_MIN_MS + extra);
            println!(
                "[m5stack-cores3] new target pan={} tilt={} next={}ms",
                pan_target,
                tilt_target,
                RANDOM_INTERVAL_MIN_MS + extra
            );
        }

        // Catch up on missed servo ticks; render frames may take longer than SERVO_TICK_MS.
        let elapsed_ms = last_tick.elapsed().as_millis();
        let ticks_due = (elapsed_ms / SERVO_TICK_MS) as usize;
        if ticks_due > 0 {
            last_tick += Duration::from_millis(ticks_due as u64 * SERVO_TICK_MS);
            let mut pan_pos = pan_path.get_target_position() as u16;
            let mut tilt_pos = tilt_path.get_target_position() as u16;
            for _ in 0..ticks_due {
                if pan_path.is_moving() {
                    pan_pos = pan_path.step_next() as u16;
                }
                if tilt_path.is_moving() {
                    tilt_pos = tilt_path.step_next() as u16;
                }
            }
            if pan_present {
                let _ = pan.set_target_period(SERVO_TICK_MS as u16);
                let _ = pan.set_target_position(pan_pos);
            }
            if tilt_present {
                let _ = tilt.set_target_period(SERVO_TICK_MS as u16);
                let _ = tilt.set_target_position(tilt_pos);
            }
        }
    }
}
