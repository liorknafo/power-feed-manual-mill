//! What the OLED shows. The control task publishes a `Status` snapshot; the display task draws it.

use core::cell::Cell;
use core::fmt::Write;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use feed_core::menu::Item;
use feed_core::state::{Dir, State};

#[derive(Clone, Copy)]
pub struct Status {
    pub state: State,
    pub feed_mm_min: f32,
    pub rapid: bool,
    pub fine: bool,
    pub bus_mv: u32,
    pub limit: Option<Dir>,
    pub menu: Option<(Item, f32)>,
    pub saved: bool,
    pub tmc_ok: bool,
}

impl Status {
    pub const fn boot() -> Status {
        Status {
            state: State::Interlock,
            feed_mm_min: 0.0,
            rapid: false,
            fine: false,
            bus_mv: 0,
            limit: None,
            menu: None,
            saved: false,
            tmc_ok: true,
        }
    }
}

pub static STATUS: Mutex<CriticalSectionRawMutex, Cell<Status>> = Mutex::new(Cell::new(Status::boot()));

/// Small fixed-capacity string for formatting.
pub struct Buf<const N: usize> {
    b: [u8; N],
    n: usize,
}

impl<const N: usize> Buf<N> {
    pub fn new() -> Self {
        Buf { b: [0; N], n: 0 }
    }
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.b[..self.n]).unwrap_or("")
    }
}

impl<const N: usize> Write for Buf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let room = N - self.n;
        let take = s.len().min(room);
        self.b[self.n..self.n + take].copy_from_slice(&s.as_bytes()[..take]);
        self.n += take;
        Ok(())
    }
}

pub fn draw<D: DrawTarget<Color = BinaryColor>>(d: &mut D, s: &Status) {
    let small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let small_inv = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
    let big = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let center = TextStyleBuilder::new().alignment(Alignment::Center).baseline(Baseline::Top).build();
    let left = TextStyleBuilder::new().alignment(Alignment::Left).baseline(Baseline::Top).build();
    let right = TextStyleBuilder::new().alignment(Alignment::Right).baseline(Baseline::Top).build();

    if let Some((item, value)) = s.menu {
        let _ = Text::with_text_style("SETTINGS", Point::new(64, 0), small, center).draw(d);
        let _ = Text::with_text_style(item.label(), Point::new(64, 16), small, center).draw(d);
        if item == Item::UsbFlash {
            let _ = Text::with_text_style("hold: reboot to", Point::new(64, 30), small, center).draw(d);
            let _ = Text::with_text_style("USB drive RPI-RP2", Point::new(64, 41), small, center).draw(d);
            let _ = Text::with_text_style("press=next", Point::new(64, 54), small, center).draw(d);
            return;
        }
        let mut v: Buf<16> = Buf::new();
        let _ = if value < 20.0 { write!(v, "{:.2}", value) } else { write!(v, "{:.0}", value) };
        let _ = Text::with_text_style(v.as_str(), Point::new(64, 30), big, center).draw(d);
        let _ = Text::with_text_style("press=next hold=save", Point::new(64, 54), small, center).draw(d);
        return;
    }

    // top bar: state (inverted when faulted)
    let faulted = matches!(s.state, State::Fault(_));
    if faulted {
        let _ = Rectangle::new(Point::new(0, 0), Size::new(128, 11)).into_styled(PrimitiveStyle::with_fill(BinaryColor::On)).draw(d);
    }
    let mut top: Buf<24> = Buf::new();
    let _ = write!(top, "{}", s.state.name());
    if let State::Running { dir, .. } | State::Engaging { dir, .. } = s.state {
        let _ = write!(top, " {}", if dir == Dir::Left { "<<" } else { ">>" });
    }
    let _ = Text::with_text_style(top.as_str(), Point::new(64, 1), if faulted { small_inv } else { small }, center).draw(d);

    // feed rate, big
    let mut f: Buf<12> = Buf::new();
    let _ = write!(f, "{:.0}", s.feed_mm_min);
    let _ = Text::with_text_style(f.as_str(), Point::new(64, 18), big, center).draw(d);
    let _ = Text::with_text_style("mm/min", Point::new(64, 39), small, center).draw(d);

    // bottom line: rapid / limit / step size, bus voltage
    let mut b: Buf<12> = Buf::new();
    if s.rapid {
        let _ = write!(b, "RAPID");
    } else if let Some(l) = s.limit {
        let _ = write!(b, "LIMIT {}", if l == Dir::Left { "L" } else { "R" });
    } else if !s.tmc_ok {
        let _ = write!(b, "NO DRIVER");
    } else if s.saved {
        let _ = write!(b, "SAVED");
    } else {
        let _ = write!(b, "{}", if s.fine { "fine" } else { "coarse" });
    }
    let _ = Text::with_text_style(b.as_str(), Point::new(0, 53), small, left).draw(d);
    let mut v: Buf<8> = Buf::new();
    let _ = write!(v, "{:.1}V", s.bus_mv as f32 / 1000.0);
    let _ = Text::with_text_style(v.as_str(), Point::new(127, 53), small, right).draw(d);
}
