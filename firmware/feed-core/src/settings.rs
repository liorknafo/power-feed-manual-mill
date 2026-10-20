//! User-adjustable settings, with a small versioned binary format for flash storage.

/// All tunables. Mechanical values marked PLACEHOLDER must be measured on the mill (docs/mill.md).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// Table travel per leadscrew turn, mm. PLACEHOLDER until measured on the SIEG X3.5D.
    pub leadscrew_pitch_mm: f32,
    /// Motor turns per leadscrew turn (belt reduction).
    pub belt_ratio: f32,
    /// Full steps per motor revolution.
    pub motor_steps: u32,
    /// Output-side encoder counts per clutch-output revolution (600 PPR x4).
    pub encoder_counts: u32,

    /// Feed range and default, mm/min.
    pub feed_min_mm_min: f32,
    pub feed_max_mm_min: f32,
    pub feed_default_mm_min: f32,
    /// Rapid traverse speed, mm/min.
    pub rapid_mm_min: f32,
    /// Table acceleration, mm/s².
    pub accel_mm_s2: f32,

    /// Motor RMS current while running / holding, mA (torque cap, design R7).
    pub run_current_ma: u32,
    pub hold_current_ma: u32,

    /// Clutch: full-on pull-in time, hold current (sets the cam-out torque), mA.
    pub clutch_pull_in_ms: u32,
    pub clutch_hold_ma: u32,
    /// Clutch release settle time after the coil is switched off, ms.
    pub clutch_release_ms: u32,

    /// Engagement jog: max motor travel (full steps) and speed (full steps/s).
    pub jog_max_steps: u32,
    pub jog_speed_steps_s: u32,
    /// Output movement (encoder counts) that proves the teeth are meshed.
    pub engage_detect_counts: u32,

    /// Allowed lag between commanded and measured position before a stall fault, full steps.
    pub stall_tolerance_steps: u32,
    /// Bus voltage limits, mV.
    pub bus_min_mv: u32,
    pub bus_max_mv: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            leadscrew_pitch_mm: 2.0, // PLACEHOLDER
            belt_ratio: 2.0,
            motor_steps: 200,
            encoder_counts: 2400,
            feed_min_mm_min: 5.0,
            feed_max_mm_min: 400.0,
            feed_default_mm_min: 50.0,
            rapid_mm_min: 600.0,
            accel_mm_s2: 50.0,
            run_current_ma: 3000,
            hold_current_ma: 1000,
            clutch_pull_in_ms: 60,
            clutch_hold_ma: 350,
            clutch_release_ms: 40,
            jog_max_steps: 12, // ~1.5 tooth pitches of a 30-tooth clutch (6.7 steps per pitch)
            jog_speed_steps_s: 60,
            engage_detect_counts: 8,
            stall_tolerance_steps: 8,
            bus_min_mv: 30_000,
            bus_max_mv: 42_000,
        }
    }
}

const MAGIC: u32 = 0x5046_4431; // "PFD1"
pub const BLOB_LEN: usize = 4 + 4 * 21 + 4;

impl Settings {
    /// Serialise to a fixed little-endian layout: magic, fields, checksum.
    pub fn to_bytes(&self) -> [u8; BLOB_LEN] {
        let words: [u32; 21] = [
            self.leadscrew_pitch_mm.to_bits(),
            self.belt_ratio.to_bits(),
            self.motor_steps,
            self.encoder_counts,
            self.feed_min_mm_min.to_bits(),
            self.feed_max_mm_min.to_bits(),
            self.feed_default_mm_min.to_bits(),
            self.rapid_mm_min.to_bits(),
            self.accel_mm_s2.to_bits(),
            self.run_current_ma,
            self.hold_current_ma,
            self.clutch_pull_in_ms,
            self.clutch_hold_ma,
            self.clutch_release_ms,
            self.jog_max_steps,
            self.jog_speed_steps_s,
            self.engage_detect_counts,
            self.stall_tolerance_steps,
            self.bus_min_mv,
            self.bus_max_mv,
            0,
        ];
        let mut out = [0u8; BLOB_LEN];
        out[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        for (i, w) in words.iter().enumerate() {
            out[4 + 4 * i..8 + 4 * i].copy_from_slice(&w.to_le_bytes());
        }
        let sum = checksum(&out[..BLOB_LEN - 4]);
        out[BLOB_LEN - 4..].copy_from_slice(&sum.to_le_bytes());
        out
    }

    /// Parse a blob written by `to_bytes`; `None` if blank, corrupt or out of range.
    pub fn from_bytes(b: &[u8]) -> Option<Settings> {
        if b.len() < BLOB_LEN {
            return None;
        }
        let rd = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        if rd(0) != MAGIC || rd(BLOB_LEN - 4) != checksum(&b[..BLOB_LEN - 4]) {
            return None;
        }
        let w = |i: usize| rd(4 + 4 * i);
        let f = |i: usize| f32::from_bits(w(i));
        let s = Settings {
            leadscrew_pitch_mm: f(0),
            belt_ratio: f(1),
            motor_steps: w(2),
            encoder_counts: w(3),
            feed_min_mm_min: f(4),
            feed_max_mm_min: f(5),
            feed_default_mm_min: f(6),
            rapid_mm_min: f(7),
            accel_mm_s2: f(8),
            run_current_ma: w(9),
            hold_current_ma: w(10),
            clutch_pull_in_ms: w(11),
            clutch_hold_ma: w(12),
            clutch_release_ms: w(13),
            jog_max_steps: w(14),
            jog_speed_steps_s: w(15),
            engage_detect_counts: w(16),
            stall_tolerance_steps: w(17),
            bus_min_mv: w(18),
            bus_max_mv: w(19),
        };
        s.is_sane().then_some(s)
    }

    /// Reject values that would make the machine unsafe or the maths meaningless.
    pub fn is_sane(&self) -> bool {
        self.leadscrew_pitch_mm > 0.2
            && self.leadscrew_pitch_mm < 20.0
            && self.belt_ratio >= 0.5
            && self.belt_ratio <= 10.0
            && self.motor_steps >= 48
            && self.encoder_counts > 0
            && self.feed_min_mm_min > 0.0
            && self.feed_min_mm_min <= self.feed_max_mm_min
            && self.feed_default_mm_min >= self.feed_min_mm_min
            && self.feed_default_mm_min <= self.feed_max_mm_min
            && self.rapid_mm_min > 0.0
            && self.rapid_mm_min <= 2000.0
            && self.accel_mm_s2 > 0.0
            && self.run_current_ma <= 4200
            && self.hold_current_ma <= self.run_current_ma
            && self.clutch_hold_ma <= 800
            && self.clutch_pull_in_ms <= 200
            && self.bus_min_mv < self.bus_max_mv
            && self.bus_max_mv <= 48_000
    }
}

fn checksum(b: &[u8]) -> u32 {
    // FNV-1a
    let mut h: u32 = 0x811c_9dc5;
    for &x in b {
        h ^= x as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let mut s = Settings::default();
        s.leadscrew_pitch_mm = 2.5;
        s.clutch_hold_ma = 420;
        let b = s.to_bytes();
        assert_eq!(Settings::from_bytes(&b), Some(s));
    }

    #[test]
    fn blank_and_corrupt_flash_rejected() {
        assert_eq!(Settings::from_bytes(&[0xFF; BLOB_LEN]), None);
        let mut b = Settings::default().to_bytes();
        b[10] ^= 1;
        assert_eq!(Settings::from_bytes(&b), None);
    }

    #[test]
    fn insane_values_rejected() {
        let mut s = Settings::default();
        s.run_current_ma = 6000; // above the motor's 4.2 A rating
        assert_eq!(Settings::from_bytes(&s.to_bytes()), None);
    }

    #[test]
    fn defaults_are_sane() {
        assert!(Settings::default().is_sane());
    }
}
