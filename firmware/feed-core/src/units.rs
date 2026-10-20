//! Conversions between table feed (mm/min) and motor microsteps.

use crate::Settings;

/// TMC5160 native resolution.
pub const MICROSTEPS: u32 = 256;

/// Microsteps per mm of table travel.
pub fn usteps_per_mm(s: &Settings) -> f32 {
    (s.motor_steps * MICROSTEPS) as f32 * s.belt_ratio / s.leadscrew_pitch_mm
}

/// Feed in mm/min -> motor speed in microsteps/s.
pub fn feed_to_usteps_s(s: &Settings, mm_min: f32) -> f32 {
    mm_min / 60.0 * usteps_per_mm(s)
}

/// Motor speed in microsteps/s -> feed in mm/min.
pub fn usteps_s_to_feed(s: &Settings, usteps_s: f32) -> f32 {
    usteps_s * 60.0 / usteps_per_mm(s)
}

/// Motor RPM at a given feed.
pub fn feed_to_motor_rpm(s: &Settings, mm_min: f32) -> f32 {
    mm_min / s.leadscrew_pitch_mm * s.belt_ratio
}

/// Table acceleration mm/s² -> microsteps/s².
pub fn accel_to_usteps_s2(s: &Settings, mm_s2: f32) -> f32 {
    mm_s2 * usteps_per_mm(s)
}

/// Motor microsteps per output-encoder count. The encoder is on the leadscrew, after the belt:
/// one encoder revolution = `belt_ratio` motor revolutions.
pub fn usteps_per_count(s: &Settings) -> f32 {
    (s.motor_steps * MICROSTEPS) as f32 * s.belt_ratio / s.encoder_counts as f32
}

/// Output-encoder counts -> motor microsteps.
pub fn encoder_counts_to_usteps(s: &Settings, counts: i32) -> i32 {
    (counts as f32 * usteps_per_count(s)) as i32
}

/// Dial step size: finer at low feeds so the whole range is reachable but precise.
pub fn dial_step(mm_min: f32, fast: bool) -> f32 {
    let base = if mm_min < 20.0 {
        1.0
    } else if mm_min < 100.0 {
        5.0
    } else {
        10.0
    };
    if fast {
        base * 5.0
    } else {
        base
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s() -> Settings {
        let mut s = Settings::default();
        s.leadscrew_pitch_mm = 2.0;
        s.belt_ratio = 2.0;
        s
    }

    #[test]
    fn usteps_per_mm_matches_hand_calc() {
        // 200 * 256 * 2 / 2 = 51200 usteps per mm
        assert_eq!(usteps_per_mm(&s()), 51_200.0);
    }

    #[test]
    fn feed_roundtrip() {
        let v = feed_to_usteps_s(&s(), 120.0);
        assert!((v - 102_400.0).abs() < 0.01); // 2 mm/s * 51200
        assert!((usteps_s_to_feed(&s(), v) - 120.0).abs() < 1e-3);
    }

    #[test]
    fn rapid_motor_rpm_is_reasonable() {
        // 600 mm/min, 2 mm pitch, 2:1 -> 600 motor rpm
        assert!((feed_to_motor_rpm(&s(), 600.0) - 600.0).abs() < 1e-3);
    }

    #[test]
    fn encoder_scaling() {
        // 2400 counts = one leadscrew revolution = two motor revolutions (2:1 belt) = 102400 usteps
        assert_eq!(encoder_counts_to_usteps(&s(), 2400), 102_400);
        assert_eq!(encoder_counts_to_usteps(&s(), -1200), -51_200);
        assert!((usteps_per_count(&s()) - 42.6667).abs() < 1e-3);
    }

    #[test]
    fn dial_steps_get_coarser() {
        assert_eq!(dial_step(10.0, false), 1.0);
        assert_eq!(dial_step(50.0, false), 5.0);
        assert_eq!(dial_step(200.0, true), 50.0);
    }
}
