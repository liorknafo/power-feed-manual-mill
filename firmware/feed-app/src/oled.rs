//! Minimal 128x64 I²C OLED driver (SH1106 by default, SSD1306 selectable) with an
//! embedded-graphics frame buffer. The whole frame is pushed page by page.

use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::I2C0;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;

pub const ADDR: u8 = 0x3C;

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // SSD1306 kept for modules that use it; select it in main.rs
pub enum Controller {
    /// 132-column RAM, image starts at column 2.
    Sh1106,
    Ssd1306,
}

pub struct Oled<'d> {
    i2c: I2c<'d, I2C0, Blocking>,
    ctrl: Controller,
    pub buf: [u8; 1024],
}

impl<'d> Oled<'d> {
    pub fn new(i2c: I2c<'d, I2C0, Blocking>, ctrl: Controller) -> Self {
        Oled { i2c, ctrl, buf: [0; 1024] }
    }

    fn cmds(&mut self, c: &[u8]) -> bool {
        let mut b = [0u8; 8];
        b[0] = 0x00;
        b[1..=c.len()].copy_from_slice(c);
        self.i2c.blocking_write(ADDR, &b[..=c.len()]).is_ok()
    }

    pub fn init(&mut self) -> bool {
        let mut ok = true;
        ok &= self.cmds(&[0xAE, 0xD5, 0x80, 0xA8, 0x3F]);
        ok &= self.cmds(&[0xD3, 0x00, 0x40]);
        ok &= match self.ctrl {
            Controller::Sh1106 => self.cmds(&[0xAD, 0x8B]), // DC-DC on
            Controller::Ssd1306 => self.cmds(&[0x8D, 0x14, 0x20, 0x02]), // charge pump, page addressing
        };
        ok &= self.cmds(&[0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF]);
        ok &= self.cmds(&[0xD9, if self.ctrl == Controller::Sh1106 { 0x22 } else { 0xF1 }, 0xDB, 0x40]);
        ok &= self.cmds(&[0xA4, 0xA6, 0xAF]);
        ok
    }

    pub fn clear(&mut self) {
        self.buf = [0; 1024];
    }

    /// Push the frame buffer to the panel.
    pub fn flush(&mut self) -> bool {
        let col = if self.ctrl == Controller::Sh1106 { 2 } else { 0 };
        let mut ok = true;
        for page in 0..8u8 {
            ok &= self.cmds(&[0xB0 | page, col & 0x0F, 0x10 | (col >> 4)]);
            let mut data = [0u8; 129];
            data[0] = 0x40;
            data[1..].copy_from_slice(&self.buf[page as usize * 128..page as usize * 128 + 128]);
            ok &= self.i2c.blocking_write(ADDR, &data).is_ok();
        }
        ok
    }
}

impl OriginDimensions for Oled<'_> {
    fn size(&self) -> Size {
        Size::new(128, 64)
    }
}

impl DrawTarget for Oled<'_> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<BinaryColor>>>(&mut self, pixels: I) -> Result<(), Self::Error> {
        for Pixel(p, c) in pixels {
            if p.x < 0 || p.y < 0 || p.x >= 128 || p.y >= 64 {
                continue;
            }
            let idx = (p.y as usize / 8) * 128 + p.x as usize;
            let bit = 1 << (p.y as usize % 8);
            if c.is_on() {
                self.buf[idx] |= bit;
            } else {
                self.buf[idx] &= !bit;
            }
        }
        Ok(())
    }
}
