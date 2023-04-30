#![no_std]
#![no_main]

mod axp192;
mod lcd;

use core::cell::RefCell;
use core::sync::atomic::AtomicU32;
use core::fmt::Write;

use critical_section::Mutex;
use axp192::{Axp192, Axp192Gpio};
use embedded_graphics::draw_target::{DrawTarget, DrawTargetExt};
use embedded_graphics::pixelcolor::{Rgb565, BinaryColor};
use embedded_graphics::prelude::RgbColor;
use embedded_hal::{digital::OutputPin, spi::{SpiBus, SpiDevice}};
use esp32_hal::{Timer, interrupt, peripherals, Priority};
use esp32_hal::gpio::{GpioPin, Bank0GpioRegisterAccess, DualCoreInteruptStatusRegisterAccessBank0, InputOutputAnalogPinType};
use esp32_hal::soc::gpio::{Gpio13Signals, Gpio14Signals};
use esp32_hal::mcpwm::operator::{Operator, PwmPin};
use esp32_hal::peripherals::{TIMG0, MCPWM0};
use esp32_hal::timer::{Timer0, Timer1};
use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc, i2c::I2C, Delay, pdma::Dma, Spi, spi::{SpiMode, SpiBusController}, dma::DmaPriority, ehal::timer,
};
use esp_backtrace as _;
use esp_println::println;
use lcd::Lcd;
use m5stack_avatar_rs::components::eye::GazeContext;
use m5stack_avatar_rs::{Expression};
use m5stack_avatar_rs::{components::{face::DrawContext, balloon::BalloonContext}, BasicPaletteKey, Palette, Avatar};
use stackchan_rs::path_generator::PathGenerator;

use esp_alloc::{EspHeap};

#[global_allocator]
static HEAP: EspHeap = EspHeap::empty();
static mut HEAP_AREA: [u8; 1024*100] = [0; 1024*100];


struct DummyPin {}
impl embedded_hal::digital::ErrorType for DummyPin {
    type Error = ();
}
impl OutputPin for DummyPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_state(&mut self, state: embedded_hal::digital::PinState) -> Result<(), Self::Error> {
        Ok(())
    }
}

type AvatarString = heapless::String<64>;

static TIMER00: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static TICK_COUNTER_1MS: AtomicU32 = AtomicU32::new(0);

struct ServoTimerContext<'a> {
    timer: Timer<Timer1<TIMG0>>,
    path_gen_tilt: PathGenerator<256>,
    path_gen_pan: PathGenerator<256>,
    pwm_tilt: esp32_hal::mcpwm::operator::PwmPin<'a, GpioPin<esp32_hal::gpio::Unknown, Bank0GpioRegisterAccess, DualCoreInteruptStatusRegisterAccessBank0, InputOutputAnalogPinType, Gpio14Signals, 14>, esp32_hal::peripherals::MCPWM0, 0, true>,
    pwm_pan: esp32_hal::mcpwm::operator::PwmPin<'a, GpioPin<esp32_hal::gpio::Unknown, Bank0GpioRegisterAccess, DualCoreInteruptStatusRegisterAccessBank0, InputOutputAnalogPinType, Gpio13Signals, 13>, esp32_hal::peripherals::MCPWM0, 1, true>,
    pan_step: u32,
    tilt_step: u32,
}

static SERVO_TIMER_CONTEXT: Mutex<RefCell<Option<ServoTimerContext>>> = Mutex::new(RefCell::new(None));

struct TickTimer {}
impl m5stack_avatar_rs::Timer for TickTimer {
    fn timestamp_milliseconds(&self) -> u64 {
        TICK_COUNTER_1MS.load(core::sync::atomic::Ordering::Relaxed) as u64
    }
}

const SERVO_PERIOD_US: u32 = 20000;
const SERVO_CENTER_US: u32 = 1500;
const SERVO_RANGE_US: u32 = 1000;
const SERVO_RESOLUTION: u32 = 4096;

fn degree_1024_to_pwm_count(degree_1024: u32) -> u16 {
    (((SERVO_RESOLUTION * 2 * degree_1024) / (180 * 1024) + SERVO_RESOLUTION / 2) / (SERVO_PERIOD_US / 1000)) as u16
}



#[entry]
fn main() -> ! {
    // Initialize heap
    unsafe { HEAP.init(HEAP_AREA.as_mut_ptr(), HEAP_AREA.len()); }

    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    
    let mut timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    
    // Disable watchdog timer
    wdt.disable();
    rtc.rwdt.disable();

    interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();
    interrupt::enable(peripherals::Interrupt::TG0_T1_LEVEL, Priority::Priority2).unwrap();
    
    // Start periodic timer
    timer_group0.timer0.start(1u32.millis());
    timer_group0.timer0.listen();
    critical_section::with(|cs| {
        TIMER00.borrow_ref_mut(cs).replace(timer_group0.timer0);
    });

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = io.pins.gpio4;
    let sclk = io.pins.gpio18;
    let miso = io.pins.gpio38;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio5;
    let dc = io.pins.gpio15.into_push_pull_output();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);
    
    let mut axp = RefCell::new(Axp192::new(i2c, 0x34));
    axp.borrow_mut().reset_core2(|delay_ms| delay.delay_ms(delay_ms) ).unwrap();

    let dma = Dma::new(system.dma, &mut system.peripheral_clock_control);
    let dma_channel = dma.spi2channel;

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let spi_bus = Spi::new_no_cs(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        40u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );
    // .with_dma(dma_channel.configure(
    //     false,
    //     &mut descriptors,
    //     &mut rx_descriptors,
    //     DmaPriority::Priority0,
    // ));
    
    let lcd_reset = Axp192Gpio::new(&axp, axp192::GpioNum::Gpio4);
    let spi_controller = SpiBusController::from_spi(spi_bus);
    let spi_device = spi_controller.add_device(cs);
    //let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, DummyPin{});
    // let spi_interface = SPIInterface::new(spi_device, dc, DummyPin{});
    // let mut lcd = ili9341::Ili9341::new(spi_interface, lcd_reset, &mut delay, ili9341::Orientation::Landscape, DisplaySize240x320).unwrap();

    let mut display = Lcd::new(spi_device, dc, lcd_reset, Rgb565::default()).unwrap();
    display.reset(&mut delay).unwrap();
    display.fill(0, 0, 320, 240, Rgb565::new(0, 0, 0)).unwrap();

    // initialize peripheral
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 2u32.MHz()).unwrap();
    let mut mcpwm = MCPWM::new(
        peripherals.MCPWM0,                                           
        clock_cfg,
        &mut system.peripheral_clock_control,
    );

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_tilt = mcpwm
        .operator0
        .with_pin_a(io.pins.gpio14, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut pwm_pan = mcpwm
        .operator1
        .with_pin_a(io.pins.gpio13, PwmPinConfig::UP_ACTIVE_HIGH);
    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(SERVO_RESOLUTION as u16, PwmWorkingMode::Increase, 50u32.Hz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    pwm_pan.set_timestamp(degree_1024_to_pwm_count(90*1024));   // 1.5[ms]
    pwm_tilt.set_timestamp(degree_1024_to_pwm_count(70*1024));   // 1.5[ms]
    let path_gen_pan = PathGenerator::<256>::new(90 * 1024, 1.0 * 1024.0, 30.0 * 1024.0);
    let path_gen_tilt = PathGenerator::<256>::new(90 * 1024, 1.0 * 1024.0, 30.0 * 1024.0);

    timer_group0.timer1.start(SERVO_PERIOD_US.micros());
    timer_group0.timer1.listen();
    critical_section::with(|cs| {
        SERVO_TIMER_CONTEXT.borrow_ref_mut(cs).replace(ServoTimerContext {
            timer: timer_group0.timer1,
            path_gen_pan,
            path_gen_tilt,
            pwm_pan,
            pwm_tilt,
            pan_step: 0,
            tilt_step: 0,
        });
    });

    let mut context: DrawContext<Rgb565, AvatarString> = DrawContext::default();
    context.palette.set_color(&BasicPaletteKey::Primary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Secondary, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::Background, Rgb565::BLACK);
    context.palette.set_color(&BasicPaletteKey::BalloonForeground, Rgb565::WHITE);
    context.palette.set_color(&BasicPaletteKey::BalloonBackground, Rgb565::BLACK);
    context.set_text(Some("Rusty Stack-chan!"));
    let mut avatar = Avatar::new(context, 30);
    let tick_timer = TickTimer{};
    display.clear(Rgb565::BLACK).unwrap();

    println!("degree_to_pwm_count({}) = {}", 0, degree_1024_to_pwm_count(0*1024));
    println!("degree_to_pwm_count({}) = {}", 90, degree_1024_to_pwm_count(90*1024));
    println!("degree_to_pwm_count({}) = {}", 180, degree_1024_to_pwm_count(180*1024));

    loop {

        // let pan = path_gen_pan.step_next();
        // let degree = pan.clamp(45*1024, 135*1024);
        // let pwm = degree_1024_to_pwm_count(degree);
        // pwm_pan.set_timestamp(pwm);

        // println!("count={}, deg={}, pwm={}, h={}", pan, degree, pwm, avatar.context().horizontal());
        
        avatar.context().expression = Expression::Happy;
        avatar.run(&mut display, &tick_timer).unwrap();
        
        let servo_step = critical_section::with(|cs| {
            let mut context = SERVO_TIMER_CONTEXT.borrow_ref_mut(cs);
            if let Some(context) = context.as_mut() {
                if !context.path_gen_pan.is_moving() {
                    context.path_gen_pan.begin_move_to((avatar.context().horizontal()  * 22.5 * 1024.0) as u32 +  90 * 1024)
                }
                if !context.path_gen_tilt.is_moving() {
                    context.path_gen_tilt.begin_move_to(65 * 1024 - (avatar.context().vertical()  * 5.0 * 1024.0) as u32)
                }
                
                Some((context.pan_step, context.tilt_step))
            } else {
                None
            }
        });
        if let Some((pan_step, tilt_step)) = servo_step {
            println!("pan={}, tilt={}, h={}, v={}", pan_step, tilt_step, avatar.context().horizontal(), avatar.context().vertical());
        }
        //delay.delay_ms(1000/30u32);
    }
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER00.borrow_ref_mut(cs);
        if let Some(timer) = timer.as_mut() {
            if timer.is_interrupt_set() {
                timer.clear_interrupt();
                timer.start(1000u64.millis());
                TICK_COUNTER_1MS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
    });
}


#[interrupt]
fn TG0_T1_LEVEL() {
    critical_section::with(|cs| {
        let mut context = SERVO_TIMER_CONTEXT.borrow_ref_mut(cs);
        if let Some(context) = context.as_mut() {
            if context.timer.is_interrupt_set() {
                context.timer.clear_interrupt();
                context.timer.start(SERVO_PERIOD_US.micros());
                
                context.path_gen_pan.step_next();
                let pan = context.path_gen_pan.step_next();
                let degree = pan.clamp(45*1024, 135*1024);
                let pwm = degree_1024_to_pwm_count(degree);
                context.pwm_pan.set_timestamp(pwm);
                context.pan_step = pan;

                context.path_gen_tilt.step_next();
                let tilt = context.path_gen_tilt.step_next();
                let degree = tilt.clamp(60*1024, 70*1024);
                let pwm = degree_1024_to_pwm_count(degree);
                context.pwm_tilt.set_timestamp(pwm);
                context.tilt_step = tilt;
            }
        }
    });
}

#[cfg(test)]
mod test {
    use super::degree_to_pwm_count;
    #[test]
    fn test_degree_to_pwm_count() {
        assert_eq!(degree_to_pwm_count(90), 76);
    }
}