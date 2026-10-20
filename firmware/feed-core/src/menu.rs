//! Dial push-button handling and the settings menu.
//!
//! * short press while running/idle: toggle fine/coarse dial steps (handled by the app);
//! * long press while IDLE: enter the settings menu;
//! * in the menu: turn = change value, short press = next item, long press = save and exit.

use crate::Settings;

/// Debounced push button with short/long press events.
pub struct Button {
    stable: bool,
    raw_since: u32,
    raw: bool,
    pressed_at: u32,
    long_sent: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Press {
    Short,
    Long,
}

const DEBOUNCE_MS: u32 = 20;
const LONG_MS: u32 = 800;

impl Button {
    pub const fn new() -> Self {
        Button { stable: false, raw_since: 0, raw: false, pressed_at: 0, long_sent: false }
    }

    /// `down` = button currently pressed. Returns an event at most once per press.
    pub fn update(&mut self, down: bool, now: u32) -> Option<Press> {
        if down != self.raw {
            self.raw = down;
            self.raw_since = now;
        }
        if self.raw != self.stable && now.wrapping_sub(self.raw_since) >= DEBOUNCE_MS {
            self.stable = self.raw;
            if self.stable {
                self.pressed_at = now;
                self.long_sent = false;
            } else if !self.long_sent {
                return Some(Press::Short);
            }
        }
        if self.stable && !self.long_sent && now.wrapping_sub(self.pressed_at) >= LONG_MS {
            self.long_sent = true;
            return Some(Press::Long);
        }
        None
    }
}

impl Default for Button {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Item {
    Pitch,
    BeltRatio,
    RunCurrent,
    HoldCurrent,
    ClutchHold,
    Rapid,
    FeedMax,
    Accel,
}

pub const ITEMS: [Item; 8] =
    [Item::Pitch, Item::BeltRatio, Item::RunCurrent, Item::HoldCurrent, Item::ClutchHold, Item::Rapid, Item::FeedMax, Item::Accel];

impl Item {
    pub fn label(self) -> &'static str {
        match self {
            Item::Pitch => "SCREW PITCH mm",
            Item::BeltRatio => "BELT RATIO",
            Item::RunCurrent => "MOTOR mA",
            Item::HoldCurrent => "HOLD mA",
            Item::ClutchHold => "CLUTCH HOLD mA",
            Item::Rapid => "RAPID mm/min",
            Item::FeedMax => "FEED MAX mm/min",
            Item::Accel => "ACCEL mm/s2",
        }
    }

    pub fn value(self, s: &Settings) -> f32 {
        match self {
            Item::Pitch => s.leadscrew_pitch_mm,
            Item::BeltRatio => s.belt_ratio,
            Item::RunCurrent => s.run_current_ma as f32,
            Item::HoldCurrent => s.hold_current_ma as f32,
            Item::ClutchHold => s.clutch_hold_ma as f32,
            Item::Rapid => s.rapid_mm_min,
            Item::FeedMax => s.feed_max_mm_min,
            Item::Accel => s.accel_mm_s2,
        }
    }

    /// Apply `detents` to a copy; the change is kept only if the result is still sane.
    pub fn adjust(self, s: &Settings, detents: i32) -> Settings {
        let mut n = *s;
        let d = detents as f32;
        match self {
            Item::Pitch => n.leadscrew_pitch_mm = round2(n.leadscrew_pitch_mm + d * 0.05),
            Item::BeltRatio => n.belt_ratio = round2(n.belt_ratio + d * 0.05),
            Item::RunCurrent => n.run_current_ma = (n.run_current_ma as i32 + detents * 100).max(0) as u32,
            Item::HoldCurrent => n.hold_current_ma = (n.hold_current_ma as i32 + detents * 100).max(0) as u32,
            Item::ClutchHold => n.clutch_hold_ma = (n.clutch_hold_ma as i32 + detents * 10).max(0) as u32,
            Item::Rapid => n.rapid_mm_min = (n.rapid_mm_min + d * 25.0).max(0.0),
            Item::FeedMax => n.feed_max_mm_min = (n.feed_max_mm_min + d * 10.0).max(0.0),
            Item::Accel => n.accel_mm_s2 = (n.accel_mm_s2 + d * 5.0).max(0.0),
        }
        if n.is_sane() {
            n
        } else {
            *s
        }
    }
}

fn round2(x: f32) -> f32 {
    ((x * 100.0) + if x >= 0.0 { 0.5 } else { -0.5 }) as i32 as f32 / 100.0
}

pub struct Menu {
    pub active: bool,
    pub idx: usize,
}

impl Menu {
    pub const fn new() -> Self {
        Menu { active: false, idx: 0 }
    }
    pub fn item(&self) -> Item {
        ITEMS[self.idx]
    }
    pub fn next(&mut self) {
        self.idx = (self.idx + 1) % ITEMS.len();
    }
}

impl Default for Menu {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_and_long_presses() {
        let mut b = Button::new();
        let mut events = vec![];
        let mut feed = |b: &mut Button, down: bool, from: u32, to: u32| {
            for t in from..to {
                if let Some(e) = b.update(down, t) {
                    events.push((t, e));
                }
            }
        };
        feed(&mut b, false, 0, 100);
        feed(&mut b, true, 100, 300); // 200 ms press
        feed(&mut b, false, 300, 400);
        feed(&mut b, true, 400, 1500); // long press
        feed(&mut b, false, 1500, 1600);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].1, Press::Short);
        assert_eq!(events[1].1, Press::Long);
    }

    #[test]
    fn bounce_is_ignored() {
        let mut b = Button::new();
        let mut n = 0;
        for t in 0..200 {
            let down = t % 3 == 0 && t < 100; // chatter, never stable for 20 ms
            if b.update(down, t).is_some() {
                n += 1;
            }
        }
        assert_eq!(n, 0);
    }

    #[test]
    fn menu_adjust_stays_sane() {
        let s = Settings::default();
        let s2 = Item::Pitch.adjust(&s, 10);
        assert!((s2.leadscrew_pitch_mm - 2.5).abs() < 1e-4);
        // motor current cannot be pushed past the 4.2 A limit
        let s3 = Item::RunCurrent.adjust(&s, 100);
        assert_eq!(s3.run_current_ma, s.run_current_ma);
        let mut m = Menu::new();
        for _ in 0..ITEMS.len() {
            m.next();
        }
        assert_eq!(m.item(), Item::Pitch);
    }
}
