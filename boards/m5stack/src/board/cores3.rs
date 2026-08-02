//! M5Stack CoreS3 / CoreS3 SE board support (selected by `--features cores3`).
//!
//! Chip drivers (AXP2101 PMIC, AW9523B GPIO expander, ILI9342 LCD, PY32 IO expander)
//! live in `m5drivers-rs`.
//!
//! Pan/tilt: two SCS0009 serial bus servos on UART2 (TX=GPIO6, RX=GPIO7), driven through
//! `scs-servo`. Servo bus power (VM_EN) is switched via the PY32 IO expander on the
//! Stack-chan mount board.

use core::cell::RefCell;
use core::time::Duration as CoreDuration;

use embedded_hal::delay::DelayNs;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use esp_hal::{Async, Blocking};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::i2s::master::{Channels, Config as I2sConfig, DataFormat, I2s, I2sTx};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rng::Rng;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::time::{Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use log::{info, warn};
use m5drivers_rs::aw9523::Aw9523Reg;
use m5drivers_rs::{
    AW9523_DEFAULT_ADDR, AW88298_DEFAULT_ADDR, AXP2101_DEFAULT_ADDR, Aw88298, Aw9523, Aw9523Pin,
    Axp2101, FT6336_DEFAULT_ADDR, Ft6336, Ili9342Dma, PY32_DEFAULT_ADDR, Pin, Py32IoExpander,
};
use scs_servo::device::ServoControl;
use scs_servo::device::scs0009::Scs0009ServoControl;
use scs_servo::protocol::{ProtocolMasterConfig, StreamReader, StreamWriter};
use static_cell::StaticCell;

use crate::head::{DEG, HeadDriver, HeadLimits};

// ---- SCS bus / servo configuration -----------------------------------------
const SCS_BAUD: u32 = 1_000_000;
const PAN_ID: u8 = 1;
const TILT_ID: u8 = 2;
/// Center of the SCS0009 1024-step range.
const SCS_CENTER: i32 = 0x200;
/// SCS units per 45° (1024 steps / 360° ≈ 2.84 unit/deg → 128 units).
const SCS_UNITS_PER_45_DEG: i32 = 128;

const HEAD_LIMITS: HeadLimits = HeadLimits {
    pan_min: 45 * DEG,
    pan_max: 135 * DEG,
    // ±15° — matches the previous ±43 SCS-unit tilt range.
    tilt_min: 75 * DEG,
    tilt_max: 105 * DEG,
};

/// Convert a pose in 1/1024-degree units (90° = straight ahead) to SCS0009 raw units.
fn deg_1024_to_scs(pose: u32) -> u16 {
    let delta = pose as i32 - (90 * DEG) as i32;
    let raw = SCS_CENTER + delta * SCS_UNITS_PER_45_DEG / (45 * DEG) as i32;
    raw.clamp(0, 1023) as u16
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
// Both adapters borrow the same `RefCell<Uart<...>>`. Each call grabs a brief
// `borrow_mut`, performs one write or read_buffered, and releases — so the SCS
// protocol's "send packet, then read response" sequence works without contention.
struct UartTxRef<'a, 'd>(&'a RefCell<Uart<'d, Blocking>>);
struct UartRxRef<'a, 'd>(&'a RefCell<Uart<'d, Blocking>>);

impl<'a, 'd> StreamWriter for UartTxRef<'a, 'd> {
    type Error = esp_hal::uart::TxError;
    fn write(&mut self, data: &[u8]) -> nb::Result<usize, Self::Error> {
        let mut uart = self.0.borrow_mut();
        match uart.write(data) {
            Ok(0) => Err(nb::Error::WouldBlock),
            Ok(n) => Ok(n),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

impl<'a, 'd> StreamReader for UartRxRef<'a, 'd> {
    type Error = esp_hal::uart::RxError;
    fn read(&mut self, data: &mut [u8]) -> nb::Result<usize, Self::Error> {
        let mut uart = self.0.borrow_mut();
        match uart.read_buffered(data) {
            Ok(0) => Err(nb::Error::WouldBlock),
            Ok(n) => Ok(n),
            Err(e) => Err(nb::Error::Other(e)),
        }
    }
}

// ---- Static placement for the large stateful objects ----------------------
// These live in BSS via `static_cell::StaticCell` so they don't sit in `main`'s stack
// frame. opt-level=3 used to inline `main` to a single frame large enough to overflow
// the stack inside esp-hal's `Uart::new`; moving them to BSS makes the layout stable.
type I2cBusTy = I2c<'static, Blocking>;
type I2cBusCell = RefCell<I2cBusTy>;
type I2cDeviceTy = I2cRefCellDevice<'static, I2cBusTy>;
type Aw9523Cell = RefCell<Aw9523<I2cDeviceTy>>;
type Axp2101Ty = Axp2101<I2cDeviceTy>;
type LcdRstTy = Aw9523Pin<'static, I2cDeviceTy>;
type LcdDriverTy = Ili9342Dma<SpiDmaBus<'static, Async>, Output<'static>, Output<'static>, LcdRstTy>;

/// Newtype implementing the avatar's [`m5stack_avatar_rs::stackchan::AsyncDisplay`] for
/// the DMA LCD driver (orphan-rule workaround).
pub struct DisplayTy(LcdDriverTy);

impl m5stack_avatar_rs::stackchan::AsyncDisplay for DisplayTy {
    type Error = m5drivers_rs::Ili9342DmaError;

    fn dimensions(&self) -> (i32, i32) {
        (320, 240)
    }

    async fn fill_rect(&mut self, x: i32, y: i32, w: u32, h: u32, color: u16)
    -> Result<(), Self::Error> {
        self.0.fill_rect(x as u16, y as u16, w as u16, h as u16, color).await
    }

    async fn blit(&mut self, x: i32, y: i32, w: u32, h: u32, pixels: &[u8])
    -> Result<(), Self::Error> {
        self.0.blit(x as u16, y as u16, w as u16, h as u16, pixels).await
    }
}
pub type HeadTy = ScsHead;
pub type TouchTy = Ft6336<I2cDeviceTy>;
pub type SpeakerTy = I2sTx<'static, Async>;
type UartTy = Uart<'static, Blocking>;
type UartCell = RefCell<UartTy>;
type ScsServoTy =
    Scs0009ServoControl<UartRxRef<'static, 'static>, UartTxRef<'static, 'static>, ScsClock>;

static I2C_BUS: StaticCell<I2cBusCell> = StaticCell::new();
static AW_CELL: StaticCell<Aw9523Cell> = StaticCell::new();
static AXP: StaticCell<Axp2101Ty> = StaticCell::new();
static DISPLAY: StaticCell<DisplayTy> = StaticCell::new();
// Initialized by `make_head` on the second core: the SCS bus (UART + servo controllers)
// lives entirely on core 1, so the RefCell-based sharing never crosses cores.
static UART: StaticCell<UartCell> = StaticCell::new();
static PAN: StaticCell<ScsServoTy> = StaticCell::new();
static TILT: StaticCell<ScsServoTy> = StaticCell::new();

/// Pan/tilt head on SCS0009 serial bus servos. Presence is probed per-servo during
/// bring-up; a missing servo is skipped so the firmware can run avatar-only.
pub struct ScsHead {
    pan: &'static mut ScsServoTy,
    tilt: &'static mut ScsServoTy,
    pan_present: bool,
    tilt_present: bool,
}

impl HeadDriver for ScsHead {
    fn set_pose(&mut self, pan: u32, tilt: u32, period_ms: u16) {
        if self.pan_present {
            let _ = self.pan.set_target_period(period_ms);
            let _ = self.pan.set_target_position(deg_1024_to_scs(pan));
        }
        if self.tilt_present {
            let _ = self.tilt.set_target_period(period_ms);
            let _ = self.tilt.set_target_position(deg_1024_to_scs(tilt));
        }
    }
}

/// Drain whatever the SCS bus has left in the RX FIFO. Run between probes so the previous
/// servo's tail bytes can't be interpreted as the next servo's reply.
#[allow(dead_code)]
fn drain_uart(uart: &RefCell<Uart<'_, Blocking>>) {
    let mut scratch = [0u8; 32];
    loop {
        let n = uart.borrow_mut().read_buffered(&mut scratch).unwrap_or(0);
        if n == 0 {
            break;
        }
    }
}

/// Probe IDs 1..=`max_id` on the SCS bus by attempting a read of the servo's limit
/// register. The throwaway `Scs0009ServoControl` per iteration is `Box`-allocated so it
/// sits on the heap rather than this function's stack frame. Debug helper.
#[allow(dead_code)]
fn scan_scs_servos(uart: &'static UartCell, max_id: u8) {
    use alloc::boxed::Box;
    info!("scanning SCS bus (ids 1..={})", max_id);
    let mut found = 0u32;
    for id in 1..=max_id {
        drain_uart(uart);
        let mut probe = Box::new(Scs0009ServoControl::<_, _, ScsClock>::new(
            id,
            UartRxRef(uart),
            UartTxRef(uart),
            ProtocolMasterConfig { echo_back: false },
            CoreDuration::from_millis(50),
        ));
        if probe.position_lower_limit().is_ok() {
            info!("  found SCS servo id={}", id);
            found += 1;
        }
    }
    info!("scan done: {} servo(s) responded", found);
}

/// Everything needed to build the [`ScsHead`] on the servo core. All fields are `Send`
/// (the `Uart` is owned), so the whole struct can move into the second core's entry
/// closure; the non-`Send` `RefCell` sharing is only created there, in [`make_head`].
pub struct HeadParts {
    uart: UartTy,
    pan_present: bool,
    tilt_present: bool,
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

/// Build the head driver. Must be called on the core that will run the servo task —
/// the servo controllers share the UART through a `RefCell`, which must not cross cores.
pub fn make_head(parts: HeadParts) -> ScsHead {
    let uart: &'static UartCell = UART.init(RefCell::new(parts.uart));
    // These servos run with SCS response level 0 (reply only to READ/PING), so runtime
    // write commands must not wait for an acknowledgement — with a waiting write, every
    // set_target_* blocked for the full response timeout (ratchety motion).
    let scs_timeout = CoreDuration::from_millis(200);
    let pan: &'static mut ScsServoTy = PAN.init(
        Scs0009ServoControl::<_, _, ScsClock>::new(
            PAN_ID,
            UartRxRef(uart),
            UartTxRef(uart),
            ProtocolMasterConfig { echo_back: false },
            scs_timeout,
        )
        .with_wait_write_response(false),
    );
    let tilt: &'static mut ScsServoTy = TILT.init(
        Scs0009ServoControl::<_, _, ScsClock>::new(
            TILT_ID,
            UartRxRef(uart),
            UartTxRef(uart),
            ProtocolMasterConfig { echo_back: false },
            scs_timeout,
        )
        .with_wait_write_response(false),
    );
    ScsHead {
        pan,
        tilt,
        pan_present: parts.pan_present,
        tilt_present: parts.tilt_present,
    }
}

// Each bring-up stage below is `#[inline(never)]` so its locals live only for the
// duration of that stage. With everything inlined into one giant `init` frame the main
// stack overflowed (symptom: corrupted I2S config, LoadProhibited inside `Uart::new`).

#[inline(never)]
fn init_i2c_bus(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO12<'static>,
    scl: esp_hal::peripherals::GPIO11<'static>,
) -> &'static I2cBusCell {
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl);
    I2C_BUS.init(RefCell::new(i2c))
}

/// AW9523B init (matches M5GFX `M5GFX.cpp` CoreS3 setup) + AXP2101 power-on.
#[inline(never)]
fn init_power(i2c_bus: &'static I2cBusCell) -> &'static Aw9523Cell {
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
    info!("AW9523B configured");

    let axp: &'static mut Axp2101Ty = AXP.init(Axp2101::new(
        I2cRefCellDevice::new(i2c_bus),
        AXP2101_DEFAULT_ADDR,
    ));
    match axp.chip_id() {
        Ok(id) => info!("AXP2101 chip id = 0x{:02X} (expect 0x4A)", id),
        Err(e) => panic!("AXP2101 chip id read failed: {:?}", e),
    }
    axp.power_on_cores3().unwrap();
    axp.set_backlight_brightness(160).unwrap();
    aw
}

/// LCD on SPI2, mode 0, 40 MHz; 3-wire — MISO unused.
#[inline(never)]
fn init_display(
    spi2: esp_hal::peripherals::SPI2<'static>,
    dma: esp_hal::peripherals::DMA_CH1<'static>,
    sck: esp_hal::peripherals::GPIO36<'static>,
    mosi: esp_hal::peripherals::GPIO37<'static>,
    cs: esp_hal::peripherals::GPIO3<'static>,
    dc: esp_hal::peripherals::GPIO35<'static>,
    aw: &'static Aw9523Cell,
    delay: &mut Delay,
) -> &'static mut DisplayTy {
    // SPI with DMA: init the panel in blocking mode, then upgrade the bus to async so
    // frame transfers run on DMA while the executor keeps scheduling other tasks.
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = esp_hal::dma_buffers!(64, 16384);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let spi_bus = Spi::new(
        spi2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(SpiMode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_dma(dma)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    let cs = Output::new(cs, Level::High, OutputConfig::default());
    let dc = Output::new(dc, Level::Low, OutputConfig::default());
    let rst = Aw9523Pin::new(aw, Pin::p1(5));

    let mut lcd = Ili9342Dma::new(spi_bus, dc, cs, rst);
    lcd.init(delay).unwrap();
    lcd.fill_blocking(0, 0, 320, 240, 0x0000).unwrap();
    info!("ILI9342 ready (DMA)");
    DISPLAY.init(DisplayTy(lcd.map_bus(|b| b.into_async())))
}

#[inline(never)]
fn init_touch(i2c_bus: &'static I2cBusCell) -> Option<TouchTy> {
    let mut touch = Ft6336::new(I2cRefCellDevice::new(i2c_bus), FT6336_DEFAULT_ADDR);
    match touch.vendor_id() {
        Ok(id) => {
            info!("FT6336 touch ready (vendor 0x{:02X})", id);
            Some(touch)
        }
        Err(e) => {
            warn!("FT6336 not responding ({:?}); touch input disabled", e);
            None
        }
    }
}

/// AW88298 amp + I2S0. The amp is additionally gated by AW9523 P0.2, already driven
/// high by the P0Output init (0b0000_0111 includes bit 2).
#[inline(never)]
fn init_speaker(
    i2c_bus: &'static I2cBusCell,
    i2s0: esp_hal::peripherals::I2S0<'static>,
    dma: esp_hal::peripherals::DMA_CH0<'static>,
    bclk: esp_hal::peripherals::GPIO34<'static>,
    ws: esp_hal::peripherals::GPIO33<'static>,
    dout: esp_hal::peripherals::GPIO13<'static>,
) -> Option<SpeakerTy> {
    let mut amp = Aw88298::new(I2cRefCellDevice::new(i2c_bus), AW88298_DEFAULT_ADDR);
    if let Err(e) = amp.configure_speaker(crate::tasks::audio::SAMPLE_RATE_HZ) {
        warn!("AW88298 not responding ({:?}); speaker disabled", e);
        return None;
    }
    let (_rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(0, 32768);
    match I2s::new(
        i2s0,
        dma,
        I2sConfig::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(crate::tasks::audio::SAMPLE_RATE_HZ))
            .with_data_format(DataFormat::Data16Channel16)
            .with_channels(Channels::STEREO),
    ) {
        Ok(i2s) => {
            let tx = i2s
                .into_async()
                .i2s_tx
                .with_bclk(bclk)
                .with_ws(ws)
                .with_dout(dout)
                .build(tx_descriptors);
            info!("AW88298 speaker ready");
            Some(tx)
        }
        Err(e) => {
            warn!("I2S init failed ({:?}); speaker disabled", e);
            None
        }
    }
}

/// PY32 IO expander: enable servo bus power (VM_EN). The PY32 boots slowly, so probe its
/// version register for up to ~1.2 s. Non-fatal: without it the servo bus just stays
/// unpowered and the servo presence probe fails.
#[inline(never)]
fn enable_servo_power(i2c_bus: &'static I2cBusCell, delay: &mut Delay) {
    let mut py32 = Py32IoExpander::new(I2cRefCellDevice::new(i2c_bus), PY32_DEFAULT_ADDR);
    match py32.probe_version(delay, 6) {
        Ok(version) => {
            info!("PY32 IO expander: version 0x{:02X}", version);
            match py32.set_servo_power(true) {
                Ok(()) => {
                    delay.delay_ms(200);
                    info!("PY32 IO expander: servo VM_EN asserted (servo bus powered)");
                }
                Err(e) => warn!("PY32 IOE: VM_EN setup failed: {:?}", e),
            }
        }
        Err(_) => {
            warn!("PY32 IO expander: not responding (servo bus will stay unpowered)");
        }
    }
}

/// UART2 for the SCS0009 bus (Stackchan board: TX=GPIO6, RX=GPIO7) + presence probe.
/// Probes borrow the UART through a stack-local `RefCell`, `Box`ed so the controller
/// state sits on the heap; the UART is recovered afterwards so it can move to the servo
/// core as an owned, `Send` value.
#[inline(never)]
fn init_servo_bus(
    uart2: esp_hal::peripherals::UART2<'static>,
    tx: esp_hal::peripherals::GPIO6<'static>,
    rx: esp_hal::peripherals::GPIO7<'static>,
) -> HeadParts {
    let uart = Uart::new(uart2, UartConfig::default().with_baudrate(SCS_BAUD))
        .unwrap()
        .with_tx(tx)
        .with_rx(rx);

    let uart_cell = RefCell::new(uart);
    let probe = |id: u8| -> bool {
        use alloc::boxed::Box;
        // Presence is probed with a READ (answered at any SCS response level; a write
        // acknowledgement never comes at response level 0). Torque-on is then sent as a
        // fire-and-forget write.
        let mut servo = Box::new(
            Scs0009ServoControl::<_, _, ScsClock>::new(
                id,
                UartRxRef(&uart_cell),
                UartTxRef(&uart_cell),
                ProtocolMasterConfig { echo_back: false },
                CoreDuration::from_millis(200),
            )
            .with_wait_write_response(false),
        );
        match servo.position_lower_limit() {
            Ok(_) => {
                let _ = servo.output_enable();
                true
            }
            Err(e) => {
                warn!("servo id={} not responding ({:?}); disabling", id, e);
                false
            }
        }
    };
    let pan_present = probe(PAN_ID);
    let tilt_present = probe(TILT_ID);
    let uart = uart_cell.into_inner();
    if !(pan_present || tilt_present) {
        warn!("no SCS0009 detected; avatar only");
    } else {
        info!(
            "servo initialized pan: {}, tilt: {}",
            pan_present, tilt_present
        );
    }
    HeadParts {
        uart,
        pan_present,
        tilt_present,
    }
}

pub fn init() -> Board {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 200 * 1024);
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let mut delay = Delay::new();

    info!("booting (CoreS3)");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // ---- esp-rtos scheduler / embassy time driver (TIMG0) ------------------
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Internal I2C bus (AXP2101 + AW9523B + PY32 + FT6336 + AW88298).
    let i2c_bus = init_i2c_bus(peripherals.I2C0, peripherals.GPIO12, peripherals.GPIO11);
    let aw = init_power(i2c_bus);
    let display = init_display(
        peripherals.SPI2,
        peripherals.DMA_CH1,
        peripherals.GPIO36,
        peripherals.GPIO37,
        peripherals.GPIO3,
        peripherals.GPIO35,
        aw,
        &mut delay,
    );
    let touch = init_touch(i2c_bus);
    let speaker = init_speaker(
        i2c_bus,
        peripherals.I2S0,
        peripherals.DMA_CH0,
        peripherals.GPIO34,
        peripherals.GPIO33,
        peripherals.GPIO13,
    );
    enable_servo_power(i2c_bus, &mut delay);
    let head_parts = init_servo_bus(peripherals.UART2, peripherals.GPIO6, peripherals.GPIO7);
    let head_present = head_parts.pan_present || head_parts.tilt_present;

    Board {
        display,
        head_parts,
        head_present,
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
