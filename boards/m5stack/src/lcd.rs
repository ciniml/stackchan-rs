use core::iter::Iterator;
use core::marker::PhantomData;
use core::ops::Range;

use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::{Dimensions, Point, Size, RawData};
use embedded_graphics::primitives::Rectangle;
use embedded_hal::digital::*;
use embedded_hal::spi::{ErrorKind as SpiErrorKind, SpiDevice, SpiBus, SpiBusWrite, SpiBusRead, Error};
use embedded_graphics;
use embedded_graphics::Pixel;
use embedded_graphics::pixelcolor::{Rgb565, PixelColor};

#[derive(Debug)]
pub enum LcdError {
    Generic,
    SpiError(SpiErrorKind),
}

impl From<SpiErrorKind> for LcdError {
    fn from(error: SpiErrorKind) -> LcdError {
        LcdError::SpiError(error)
    }
}

pub struct Lcd<Spi, DC, RST, Color> {
    spi: Spi,
    pin_dc:  DC,
    pin_rst: RST,
    line_buffer: [u8; 640],
    _color: PhantomData<Color>,
}

const TFT_NOP:u8 = 0x00;
const TFT_SWRST:u8 = 0x01;

const TFT_CASET:u8 = 0x2A;
const TFT_PASET:u8 = 0x2B;
const TFT_RAMWR:u8 = 0x2C;

const TFT_RAMRD:u8 = 0x2E;
const TFT_IDXRD:u8 = 0xDD; // ILI9341 only, indexed control register read

const TFT_MADCTL:u8 = 0x36;
const TFT_MAD_MY:u8 = 0x80;
const TFT_MAD_MX:u8 = 0x40;
const TFT_MAD_MV:u8 = 0x20;
const TFT_MAD_ML:u8 = 0x10;
const TFT_MAD_BGR:u8 = 0x08;
const TFT_MAD_MH:u8 = 0x04;
const TFT_MAD_RGB:u8 = 0x00;

const TFT_INVOFF:u8 = 0x20;
const TFT_INVON:u8 = 0x21;

const ILI9341_NOP:u8 = 0x00;
const ILI9341_SWRESET:u8 = 0x01;
const ILI9341_RDDID:u8 = 0x04;
const ILI9341_RDDST:u8 = 0x09;

const ILI9341_SLPIN:u8 = 0x10;
const ILI9341_SLPOUT:u8 = 0x11;
const ILI9341_PTLON:u8 = 0x12;
const ILI9341_NORON:u8 = 0x13;

const ILI9341_RDMODE:u8 = 0x0A;
const ILI9341_RDMADCTL:u8 = 0x0B;
const ILI9341_RDPIXFMT:u8 = 0x0C;
const ILI9341_RDIMGFMT:u8 = 0x0A;
const ILI9341_RDSELFDIAG:u8 = 0x0F;

const ILI9341_INVOFF:u8 = 0x20;
const ILI9341_INVON:u8 = 0x21;
const ILI9341_GAMMASET:u8 = 0x26;
const ILI9341_DISPOFF:u8 = 0x28;
const ILI9341_DISPON:u8 = 0x29;

const ILI9341_CASET:u8 = 0x2A;
const ILI9341_PASET:u8 = 0x2B;
const ILI9341_RAMWR:u8 = 0x2C;
const ILI9341_RAMRD:u8 = 0x2E;

const ILI9341_PTLAR:u8 = 0x30;
const ILI9341_VSCRDEF:u8 = 0x33;
const ILI9341_MADCTL:u8 = 0x36;
const ILI9341_VSCRSADD:u8 = 0x37;
const ILI9341_PIXFMT:u8 = 0x3A;

const ILI9341_WRDISBV:u8 = 0x51;
const ILI9341_RDDISBV:u8 = 0x52;
const ILI9341_WRCTRLD:u8 = 0x53;

const ILI9341_FRMCTR1:u8 = 0xB1;
const ILI9341_FRMCTR2:u8 = 0xB2;
const ILI9341_FRMCTR3:u8 = 0xB3;
const ILI9341_INVCTR:u8 = 0xB4;
const ILI9341_DFUNCTR:u8 = 0xB6;

const ILI9341_PWCTR1:u8 = 0xC0;
const ILI9341_PWCTR2:u8 = 0xC1;
const ILI9341_PWCTR3:u8 = 0xC2;
const ILI9341_PWCTR4:u8 = 0xC3;
const ILI9341_PWCTR5:u8 = 0xC4;
const ILI9341_VMCTR1:u8 = 0xC5;
const ILI9341_VMCTR2:u8 = 0xC7;

const ILI9341_RDID4:u8 = 0xD3;
const ILI9341_RDINDEX:u8 = 0xD9;
const ILI9341_RDID1:u8 = 0xDA;
const ILI9341_RDID2:u8 = 0xDB;
const ILI9341_RDID3:u8 = 0xDC;
const ILI9341_RDIDX:u8 = 0xDD; // TBC

const ILI9341_GMCTRP1:u8 = 0xE0;
const ILI9341_GMCTRN1:u8 = 0xE1;

const ILI9341_MADCTL_MY:u8 = 0x80;
const ILI9341_MADCTL_MX:u8 = 0x40;
const ILI9341_MADCTL_MV:u8 = 0x20;
const ILI9341_MADCTL_ML:u8 = 0x10;
const ILI9341_MADCTL_RGB:u8 = 0x00;
const ILI9341_MADCTL_BGR:u8 = 0x08;
const ILI9341_MADCTL_MH:u8 = 0x04;

const LCD_WIDTH:u16 = 320;
const LCD_HEIGHT:u16 = 240;

impl<Spi: SpiDevice, DC: OutputPin, RST: OutputPin, Color> Lcd<Spi, DC, RST, Color> 
    where Spi::Bus: SpiBus
{
    pub fn new(spi: Spi, mut pin_dc: DC, mut pin_rst: RST, _color: Color) -> Result<Self, LcdError> {
        pin_dc.set_high().unwrap();
        pin_rst.set_low().unwrap();

        let lcd = Self{
            spi,
            pin_dc,
            pin_rst,
            line_buffer: [0; 640],
            _color: PhantomData {},
        };
        Ok(lcd)
    }

    pub fn reset<Delay: embedded_hal::delay::DelayUs>(&mut self, delay: &mut Delay) -> Result<(), LcdError> {
        self.pin_rst.set_low().unwrap();
        delay.delay_ms(150).unwrap();
        self.pin_rst.set_high().unwrap();
        delay.delay_ms(150).unwrap();

        self.write_cmd_data(0xef, &[0x03, 0x80, 0x02])?;
        self.write_cmd_data(0xcf, &[0x00, 0xc1, 0x30])?;
        self.write_cmd_data(0xed, &[0x64, 0x03, 0x12, 0x81])?;
        self.write_cmd_data(0xe8, &[0x85, 0x00, 0x78])?;
        self.write_cmd_data(0xcb, &[0x39, 0x2c, 0x00, 0x34, 0x02])?;
        self.write_cmd_data(0xf7, &[0x20])?;
        self.write_cmd_data(0xea, &[0x00, 0x00])?;
        self.write_cmd_data(ILI9341_PWCTR1, &[0x23])?;
        self.write_cmd_data(ILI9341_PWCTR2, &[0x10])?;
        self.write_cmd_data(ILI9341_VMCTR1, &[0x3e, 0x28])?;
        self.write_cmd_data(ILI9341_VMCTR2, &[0x86])?;
        self.write_cmd_data(ILI9341_MADCTL, &[0xa8])?;
        self.write_cmd_data(ILI9341_PIXFMT, &[0x55])?;
        self.write_cmd_data(ILI9341_FRMCTR1, &[0x00, 0x13])?;
        self.write_cmd_data(ILI9341_DFUNCTR, &[0x08, 0x82, 0x1d, 0x04])?;
        self.write_cmd_data(0xf2, &[0x00])?;
        self.write_cmd_data(ILI9341_GAMMASET, &[0x01])?;
        self.write_cmd_data(ILI9341_GMCTRP1, &[0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89, 0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34, 0x0F])?;
        self.write_cmd_data(ILI9341_GMCTRN1, &[0x00, 0x0B, 0x11, 0x05,0x13,0x09,0x33,0x67,0x48,0x07,0x0E,0x0B,0x2E,0x33,0x0F])?;
        self.write_cmd(ILI9341_INVON)?;
        self.write_cmd(ILI9341_SLPOUT)?;
        
        delay.delay_ms(120).unwrap();
        self.write_cmd(ILI9341_DISPON)?;
        self.write_cmd_data(TFT_MADCTL, &[TFT_MAD_BGR])?;
        Ok(())
    }

    fn read_id(&mut self) -> Result<[u8;3], LcdError> {
        let mut buffer = [0, 0, 0]; 
        self.spi.transaction(|bus| {
            self.pin_dc.set_low().unwrap();
            bus.write(&[0x04])?;
            self.pin_dc.set_high().unwrap();
            bus.read(&mut buffer)?;
            Ok(())
        })
        .map_err(|err| LcdError::SpiError(err.kind()))?;
        Ok(buffer)
    }

    fn set_column_address(&mut self, start: u16, end: u16) -> Result<(), LcdError> {
        let buffer: [u8; 4] = [
            (start >> 8) as u8,
            (start & 0xff) as u8,
            (end >> 8) as u8,
            (end & 0xff) as u8,
        ];

        self.write_cmd_data(0x2a, &buffer)
    }
    fn set_page_address(&mut self, start: u16, end: u16) -> Result<(), LcdError> {
        let buffer: [u8; 4] = [
            (start >> 8) as u8,
            (start & 0xff) as u8,
            (end >> 8) as u8,
            (end & 0xff) as u8,
        ];
        self.write_cmd_data(0x2b, &buffer)
    }
    fn start_memory_write(&mut self) -> Result<(), LcdError> {
        self.write_cmd(0x2c)
    }

    fn write_cmd(&mut self, command: u8) -> Result<(), LcdError> {
        let buffer = [command];
        self.pin_dc.set_low().unwrap();
        self.spi.transaction(|bus| bus.write(&buffer))
            .map_err(|err| LcdError::SpiError(err.kind()))?;
        Ok(())
    }
    
    fn write_data(&mut self, data: &[u8]) -> Result<(), LcdError> {
        self.pin_dc.set_high().unwrap();
        self.spi.transaction(|bus: &mut <Spi as SpiDevice>::Bus| bus.write(data) )
            .map_err(|err| LcdError::SpiError(err.kind()))?;
        Ok(())
    }

    fn write_line_buffer(&mut self, range: Range<usize>) -> Result<(), LcdError> {
        self.pin_dc.set_high().unwrap();
        let line_buffer = &self.line_buffer[range];
        self.spi.transaction(|bus: &mut <Spi as SpiDevice>::Bus| bus.write(line_buffer) )
            .map_err(|err| LcdError::SpiError(err.kind()))?;
        Ok(())
    }

    fn write_cmd_data(&mut self, command: u8, values: &[u8]) -> Result<(), LcdError> {
        self.spi.transaction(|bus: &mut <Spi as SpiDevice>::Bus| {
            self.pin_dc.set_low().unwrap();  
            bus.write(&[command])?;;
            self.pin_dc.set_high().unwrap();
            bus.write(values)?;
            Ok(())
        })
        .map_err(|err| LcdError::SpiError(err.kind()))?;
        Ok(())
    }

    pub fn fill(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: Rgb565) -> Result<(), LcdError> {
        let width = x1 - x0;
        for x in 0..width {
            let raw_color: u16 = RawU16::from(color).into_inner();
            self.line_buffer[(x*2 + 0) as usize] = (raw_color >> 8) as u8;
            self.line_buffer[(x*2 + 1) as usize] = (raw_color & 0xffu16) as u8;
        }
        self.set_column_address(x0, x1)?;
        self.set_page_address(y0, y1)?;
        self.start_memory_write()?;
        
        for _y in y0..y1 {
            self.write_line_buffer(0..((width*2) as usize))?;
        }
        Ok(())
    }

    fn inner_draw<T>(&mut self, item: T)
        where T: IntoIterator<Item = Pixel<Rgb565>>,
    {
        let mut buffer = [0u8; (LCD_WIDTH*2) as usize];
        let mut count:usize = 0;
        let mut last_x_opt:Option<i32> = None;
        let mut last_y_opt:Option<i32> = None;

        for Pixel(coord, color) in item {
            let x = coord[0];
            let y = coord[1];
            if let Some(last_y) = last_y_opt {
                if last_y != y {
                    // Flush the last line
                    if let Some(last_x) = last_x_opt {
                        if count > 0 {
                            self.set_column_address((last_x + 1 - (count as i32)) as u16, (last_x + 1) as u16);
                            self.set_page_address(last_y as u16, last_y as u16);
                            self.start_memory_write();
                            self.write_data(&buffer[..count*2]);
                            count = 0;
                        }
                    }
                    last_x_opt = None;
                }
            }
            match last_x_opt {
                None => {
                    // Nothing to do.
                },
                Some(last_x) => {
                    if last_x + 1 != x && count > 0 {
                        if let Some(last_y) = last_y_opt {
                            self.set_column_address((last_x + 1 - (count as i32)) as u16, (last_x + 1) as u16);
                            self.set_page_address(last_y as u16, last_y as u16);
                            self.start_memory_write();
                            self.write_data(&buffer[..count*2]);
                            last_x_opt = None;
                            count = 0;
                        }
                    }
                },
            }
            let raw_color = RawU16::from(color).into_inner();
            buffer[count*2 + 0] = (raw_color >> 8)   as u8;
            buffer[count*2 + 1] = (raw_color & 0xff) as u8;
            last_x_opt = Some(x);
            last_y_opt = Some(y);
            count += 1;
        }

        // Flush the last line
        if let Some(last_x) = last_x_opt {
            if let Some(last_y) = last_y_opt {
                if count > 0 {
                    self.set_column_address((last_x + 1 - (count as i32)) as u16, (last_x + 1) as u16);
                    self.set_page_address(last_y as u16, last_y as u16);
                    self.start_memory_write();
                    self.write_data(&buffer[..count*2]);
                }
            }
        }
    }


}

impl<Spi, DC, RST, Color> Dimensions for Lcd<Spi, DC, RST, Color> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, 0), Size::new(320, 240))
    }
}


impl<Spi: SpiDevice, DC: OutputPin, RST: OutputPin, Color: PixelColor + Into<Rgb565>> embedded_graphics::draw_target::DrawTarget for Lcd<Spi, DC, RST, Color> 
    where Spi::Bus: SpiBus
{
    type Color = Color;
    type Error = LcdError;
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>> {
        self.inner_draw(pixels.into_iter().map(|pixel| Pixel(pixel.0, pixel.1.into())));
        Ok(())
    }
    
    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let bottom_right = if let Some(bottom_right) = area.bottom_right() {
            bottom_right
        } else {
            return Ok(())
        };
        let raw_color = RawU16::from(color.into()).into_inner();
        for i in 0..area.size.width as usize{
            self.line_buffer[i*2 + 0] = (raw_color & 0xff) as u8;
            self.line_buffer[i*2 + 1] = (raw_color >> 8) as u8;
        }
        self.set_column_address(area.top_left.x as u16, bottom_right.x as u16)?;
        self.set_page_address(area.top_left.y as u16, bottom_right.y as u16)?;
        self.start_memory_write()?;
        for _y in 0..area.size.height {
            self.write_line_buffer(0..(area.size.width * 2) as usize)?;
        }
        Ok(())
    }
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Self::Color>, {
        let bottom_right = if let Some(bottom_right) = area.bottom_right() {
            bottom_right
        } else {
            return Ok(())
        };
        self.set_column_address(area.top_left.x as u16, bottom_right.x as u16)?;
        self.set_page_address(area.top_left.y as u16, bottom_right.y as u16)?;
        self.start_memory_write()?;
        let mut index = 0;
        for color in colors {
            if index == area.size.width as usize {
                self.write_line_buffer(0..(area.size.width * 2) as usize)?;
                index = 0;
            }
            let raw_color = RawU16::from(Rgb565::try_from(color).unwrap()).into_inner();
            self.line_buffer[index*2 + 0] = (raw_color & 0xff) as u8;
            self.line_buffer[index*2 + 1] = (raw_color >> 8) as u8;
            index += 1;
        }
        // Flush the last line.
        if index == area.size.width as usize {
            self.write_line_buffer(0..(area.size.width * 2) as usize)?;
            index = 0;
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_solid(&self.bounding_box(), color)
    }
}
