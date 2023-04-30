use core::{cell::RefCell, marker::PhantomData};

use num_enum::{IntoPrimitive, TryFromPrimitive};
use embedded_hal::i2c;
use bitflags::bitflags;

#[derive(Clone, Copy, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
enum Axp192Reg {
    PowerInputStatus = 0x00,
    ExtenDcdc2OutputConfig = 0x10,
    OutputEnable = 0x12,
    Dcdc2OutputVoltage = 0x23,
    Dcdc1OutputVoltage = 0x26,
    Dcdc3OutputVoltage = 0x27,
    Ldo2Ldo3OutputVoltage = 0x28,
    AdcEnable1 = 0x82,
    VoffShutdownVoltageConfig = 0x31,
    ChargingConfig = 0x33,
    DcdcOperatingMode = 0x80,
    CoulombCounterCharge = 0xB0,
    CoulombCounterDischarge = 0xB4,
    CoulombCounterControl = 0xB8,
    AdcRateTsControl = 0x84,
    PekConfig = 0x36,
    Gpio0Function = 0x90,
    Gpio0LdoOutputVoltage = 0x91,
    Gpio1Function = 0x92,
    Gpio2Function = 0x93,
    GpioInOut = 0x94,
    Gpio3Gpio4ControlFunction = 0x95,
    Gpio3Gpio4InOut = 0x96,
    VbusIpsoutPathConfig = 0x30,
    VhtfChargeThreshold = 0x39,
    BackupBatteryCharging = 0x35,
    AdcBase = 0x56,
}

#[derive(Clone, Copy, Debug, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum GpioNum {
    Gpio0 = 0,
    Gpio1 = 1,
    Gpio2 = 2,
    Gpio3 = 3,
    Gpio4 = 4,
}

bitflags! {
    #[derive(Clone, Copy, Debug)]
    pub struct OutputEnableReg: u8 {
        const Reserved1 = (1u8 << 7);
        const ExtEnEnable = (1u8 << 6);
        const Reserved0 = (1u8 << 5);
        const DcDc2Enable = (1u8 << 4);
        const Ldo3Enable = (1u8 << 3);
        const Ldo2Enable = (1u8 << 2);
        const DcDc3Enable = (1u8 << 1);
        const DcDc1Enable = (1u8 << 0);
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug)]
    pub struct PowerInputStatusReg: u8 {
        const AcIn = (1u8 << 7);
        const AcInAvailable = (1u8 << 6);
        const Vbus = (1u8 << 5);
        const VbusAvailable = (1u8 << 4);
        const VbusExceedsVhold = (1u8 << 3);
        const Charging = (1u8 << 2);
        const AcInVbusShort = (1u8 << 1);
        const IsAcInVbusPowered = (1u8 << 0);
    }
}

const TFT_OUTPUT_VALUE: u8 = 0x0c;  // TFT Output Voltage = 1.8 + 0.1*12 = 3.0[V]

pub const MAX_SCREEN_BRIGHTNESS: u8 = 0x0c;

#[derive(Clone, Copy, Debug)]
pub struct RegulatorRange {
    min: u16,
    max: u16,
    step: u16,
}

impl RegulatorRange {
    const fn to_reg_value(&self, voltage_mv: u16) -> Result<u8, AxpError> {
        if voltage_mv < self.min || self.max < voltage_mv {
            Err(AxpError::InvalidVoltage)
        } else {
            Ok(((voltage_mv - self.min) / self.step) as u8)
        }
    }
}

pub const DCDC1_RANGE: RegulatorRange = RegulatorRange { min: 700, max: 3500, step: 25 };
pub const DCDC2_RANGE: RegulatorRange = RegulatorRange { min: 700, max: 2275, step: 25 };
pub const DCDC3_RANGE: RegulatorRange = RegulatorRange { min: 700, max: 3500, step: 25 };
pub const LDO2_RANGE: RegulatorRange = RegulatorRange { min: 1800, max: 3300, step: 100 };
pub const LDO3_RANGE: RegulatorRange = RegulatorRange { min: 1800, max: 3300, step: 100 };

#[derive(Debug)]
pub enum AxpError {
    I2c(i2c::ErrorKind),
    InvalidValue,
    InvalidVoltage,
}

impl<I2cError: i2c::Error> From<I2cError> for AxpError {
    fn from(value: I2cError) -> Self {
        Self::I2c(value.kind())
    }
}

pub struct Axp192Gpio<'a, I2C: i2c::I2c> {
    axp: &'a RefCell<Axp192<I2C>>,
    gpio: GpioNum,
}

impl<'a, I2C: i2c::I2c> Axp192Gpio<'a, I2C> {
    pub fn new(axp: &'a RefCell<Axp192<I2C>>, gpio: GpioNum) -> Self {
        Self {
            axp,
            gpio,
        }
    }
}

impl<'a, I2C: i2c::I2c> embedded_hal::digital::ErrorType for Axp192Gpio<'a, I2C> {
    type Error = AxpError;
}
impl<'a, I2C: i2c::I2c> embedded_hal::digital::OutputPin for Axp192Gpio<'a, I2C> {
    fn set_state(&mut self, state: embedded_hal::digital::PinState) -> Result<(), Self::Error> {
        let value = match state {
            embedded_hal::digital::PinState::High => true,
            embedded_hal::digital::PinState::Low => false,
        };
        self.axp.borrow_mut().set_gpio_output(self.gpio, value)?;
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_state(embedded_hal::digital::PinState::High)
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_state(embedded_hal::digital::PinState::Low)
    }
}

pub struct Axp192<I2c: i2c::I2c> {
    i2c: I2c,
    address: u8,
    screen_brightness: u8,
}

impl<I2c: i2c::I2c> Axp192<I2c> {
    pub fn new(i2c: I2c, address: u8) -> Self {
        Self {
            i2c,
            address,
            screen_brightness: 0,
        }
    }

    fn read_reg_single(&mut self, reg: Axp192Reg) -> Result<u8, AxpError> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg.into()], &mut buffer)?;
        Ok(buffer[0])
    }
    fn read_reg(&mut self, reg: Axp192Reg, buffer: &mut [u8]) -> Result<usize, AxpError> {
        self.i2c.write_read(self.address, &[reg.into()], buffer)?;
        Ok(buffer.len())
    }
    fn write_reg_single(&mut self, reg: Axp192Reg, value: u8) -> Result<(), AxpError> {
        self.i2c.write(self.address, &[reg.into(), value])?;
        Ok(())
    }

    fn update_reg_single<F: FnOnce(u8) -> u8>(&mut self, reg: Axp192Reg, update: F) -> Result<(), AxpError> {
        let value = self.read_reg_single(reg)?;
        self.write_reg_single(reg, update(value))?;
        Ok(())
    }

    pub fn set_screen_brightness(&mut self, brightness: u8) -> Result<(), AxpError> {
        if brightness > MAX_SCREEN_BRIGHTNESS {
            Err(AxpError::InvalidValue)
        } else {
            self.write_reg_single(Axp192Reg::Ldo2Ldo3OutputVoltage, TFT_OUTPUT_VALUE | (brightness << 4))?;
            self.screen_brightness = brightness;
            Ok(())
        }
    }

    fn enable_coulomb_counter(&mut self) -> Result<(), AxpError> {
        self.write_reg_single(Axp192Reg::CoulombCounterControl, 0x80)?;
        Ok(())
    }
    fn enable_power_outputs(&mut self) -> Result<(), AxpError> {
        self.write_reg_single(Axp192Reg::OutputEnable, 0x4d)?;  // Enable EXTEN, LDO3, LDO2, DC-DC1 output
        Ok(())
    }

    fn set_dcdc_voltage(&mut self, reg: Axp192Reg, range: RegulatorRange, voltage_mv: u16) -> Result<(), AxpError> {
        let reg_value = range.to_reg_value(voltage_mv)?;
        self.update_reg_single(reg, |value| (value & 0x80) | reg_value)?;
        Ok(())
    }
    fn set_dcdc1_voltage(&mut self, voltage_mv: u16) -> Result<(), AxpError> {
        self.set_dcdc_voltage(Axp192Reg::Dcdc1OutputVoltage, DCDC1_RANGE, voltage_mv)
    }
    fn set_dcdc2_voltage(&mut self, voltage_mv: u16) -> Result<(), AxpError> {
        self.set_dcdc_voltage(Axp192Reg::Dcdc2OutputVoltage, DCDC2_RANGE, voltage_mv)
    }
    fn set_dcdc3_voltage(&mut self, voltage_mv: u16) -> Result<(), AxpError> {
        self.set_dcdc_voltage(Axp192Reg::Dcdc3OutputVoltage, DCDC3_RANGE, voltage_mv)
    }
    fn set_ldo2_voltage(&mut self, voltage_mv: u16) -> Result<(), AxpError> {
        let reg_value = LDO2_RANGE.to_reg_value(voltage_mv)?;
        self.update_reg_single(Axp192Reg::Ldo2Ldo3OutputVoltage, |value| (value & 0x0f) | (reg_value << 4))?;
        Ok(())
    }
    fn set_ldo3_voltage(&mut self, voltage_mv: u16) -> Result<(), AxpError> {
        let reg_value = LDO3_RANGE.to_reg_value(voltage_mv)?;
        self.update_reg_single(Axp192Reg::Ldo2Ldo3OutputVoltage, |value| (value & 0xf0) | reg_value)?;
        Ok(())
    }

    fn set_output_enable(&mut self, bit: OutputEnableReg, enable: bool) -> Result<(), AxpError> {
        self.update_reg_single(Axp192Reg::OutputEnable, |value| {
            let reg = OutputEnableReg::from_bits(value).unwrap();
            let reg = if enable { reg.union(bit) } else { reg.intersection(bit.complement()) };
            reg.bits()
        })
    }
    fn set_ldo2_enable(&mut self, enable: bool) -> Result<(), AxpError> {
        self.set_output_enable(OutputEnableReg::Ldo2Enable, enable)
    }
    fn set_ldo3_enable(&mut self, enable: bool) -> Result<(), AxpError> {
        self.set_output_enable(OutputEnableReg::Ldo3Enable, enable)
    }
    fn set_dcdc1_enable(&mut self, enable: bool) -> Result<(), AxpError> {
        self.set_output_enable(OutputEnableReg::DcDc1Enable, enable)
    }
    fn set_dcdc2_enable(&mut self, enable: bool) -> Result<(), AxpError> {
        self.set_output_enable(OutputEnableReg::DcDc2Enable, enable)
    }
    fn set_dcdc3_enable(&mut self, enable: bool) -> Result<(), AxpError> {
        self.set_output_enable(OutputEnableReg::DcDc3Enable, enable)
    }

    fn get_gpio_input(&mut self, gpio: GpioNum) -> Result<bool, AxpError> {
        let bit = 1u8 << Into::<u8>::into(gpio);
        use GpioNum::*;
        match gpio {
            Gpio0 | Gpio1 | Gpio2 => {
                let input = self.read_reg_single(Axp192Reg::GpioInOut)?;
                Ok(((input >> 4) & bit) != 0)
            },
            Gpio3 | Gpio4 => {
                let input = self.read_reg_single(Axp192Reg::Gpio3Gpio4InOut)?;
                Ok(((input >> 4) & (bit >> 3)) != 0)
            },
        }
    }
    fn set_gpio_output(&mut self, gpio: GpioNum, value: bool) -> Result<(), AxpError> {
        let bit = 1u8 << Into::<u8>::into(gpio);
        use GpioNum::*;
        match gpio {
            Gpio0 | Gpio1 | Gpio2 => self.update_reg_single(Axp192Reg::GpioInOut, |reg_value| if value { reg_value | bit } else { reg_value & !bit } )?,
            Gpio3 | Gpio4 => self.update_reg_single(Axp192Reg::Gpio3Gpio4InOut, |reg_value| if value { reg_value | (bit >> 3) } else { reg_value & !(bit >> 3) } )?,
        }
        Ok(())
    }

    pub fn reset_stickc(&mut self) -> Result<(), AxpError> {
        self.write_reg_single(Axp192Reg::ExtenDcdc2OutputConfig, 0xff)?; // Enable all DC-DC2 outputs
        self.set_screen_brightness(MAX_SCREEN_BRIGHTNESS)?;
        self.write_reg_single(Axp192Reg::AdcEnable1, 0xff)?;        // Enable all ADCs
        self.write_reg_single(Axp192Reg::ChargingConfig, 0xc1)?;    // Charging, 192mA, 4.2V, 90%
        self.enable_coulomb_counter()?;
        self.enable_power_outputs()?;
        self.write_reg_single(Axp192Reg::PekConfig, 0x4c)?;         // Shutdown 4s, Shutdown with button press longer than 4s
        self.write_reg_single(Axp192Reg::Gpio0Function, 0x02)?;     // GPIO0 = Low Noise LDO
        self.write_reg_single(Axp192Reg::VbusIpsoutPathConfig, 0xe0)?;    // No use N_VBUSen, Limit Vbus voltage = 4.4V, Limit Vbus current = 500mA
        self.write_reg_single(Axp192Reg::VhtfChargeThreshold, 0xfc)?;     // 
        self.write_reg_single(Axp192Reg::BackupBatteryCharging, 0xa2)?;   // Enable charging backup battery, 3.0V, 200uA 

        Ok(())
    }

    pub fn reset_core2<DelayMs: FnMut(u32)>(&mut self, mut delay: DelayMs) -> Result<(), AxpError> {
        self.update_reg_single(Axp192Reg::VbusIpsoutPathConfig, |value| (value & 0x04) | 0x02)?;    // Limit Vbus current = 500mA
        self.update_reg_single(Axp192Reg::Gpio1Function, |value| value & 0xf8)?; // GPIO1: NMOS Open drain.
        self.update_reg_single(Axp192Reg::Gpio2Function, |value| value & 0xf8)?; // GPIO2: NMOS Open drain.
        self.update_reg_single(Axp192Reg::BackupBatteryCharging, |value| (value & 0x1c) | 0xa2)?;   // Enable charging backup battery, 3.0V, 100uA 
        self.set_dcdc1_voltage(3350)?;  // Set ESP32 Vdd to 3.350V
        self.set_dcdc3_voltage(2800)?;  // Set LCD Voltage to 2.8V
        self.set_ldo2_voltage(3300)?;   // Set Peripheral power voltage to 3.3V
        self.set_ldo3_voltage(2000)?;   // Set vibrator power voltage to 2.0V
        self.set_ldo2_enable(true)?;
        self.set_ldo3_enable(false)?;
        self.set_dcdc3_enable(true)?;
        self.set_gpio_output(GpioNum::Gpio1, false)?; // turn on LED
        self.write_reg_single(Axp192Reg::ChargingConfig, 0xc1)?;    // Charging, 192mA, 4.2V, 90%
        self.update_reg_single(Axp192Reg::Gpio3Gpio4ControlFunction, |value| (value & 0x72) | 0x84)?; // GPIO3=Dis, GPIO4=OD
        self.write_reg_single(Axp192Reg::PekConfig, 0x4c)?;         // Shutdown 4s, Shutdown with button press longer than 4s
        self.write_reg_single(Axp192Reg::AdcEnable1, 0xff)?;        // Enable all ADCs
        
        // LCD reset
        self.set_gpio_output(GpioNum::Gpio4, false)?; // Assert LCD Reset
        delay(100);
        self.set_gpio_output(GpioNum::Gpio4, true)?; // Deassert LCD Reset
        delay(100);
        
        self.update_reg_single(Axp192Reg::ExtenDcdc2OutputConfig, |value| (value | 0x04))?; // Enable 5V boost.


        if (self.read_reg_single(Axp192Reg::PowerInputStatus)? & 0x08) != 0 {
            // External power
            self.update_reg_single(Axp192Reg::VbusIpsoutPathConfig, |value| value | 0x80)?;      // Limit Vbus current = 500mA
            self.update_reg_single(Axp192Reg::ExtenDcdc2OutputConfig, |value: u8| (value & !0x04))?; // Disable 5V boost
            self.update_reg_single(Axp192Reg::Gpio0Function, |value: u8| (value & 0xf8) | 0x07)?;    // GPIO0: float
        } else {
            self.update_reg_single(Axp192Reg::Gpio0LdoOutputVoltage, |value| (value & 0x0f) | 0xf0)?;    // GPIO0: LDO, 3.3V
            self.update_reg_single(Axp192Reg::Gpio0Function, |value| (value & 0xf8) | 0x02)?;    // GPIO0: LDO
        }

        Ok(())
    }
}

