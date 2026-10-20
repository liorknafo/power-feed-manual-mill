//! Clutch coil current control (docs/design.md §7.6).
//!
//! Hardware: low-side FET on while engaged, high-side P-FET PWM'd; the shunt sees the coil current
//! continuously (slow-decay recirculation), so the ADC gives the average current directly.
//! This module turns a requested mode + the measured current into a PWM duty.

/// What the state machine asks of the clutch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClutchCmd {
    /// Both switches off: fast decay, teeth released by the springs.
    Off,
    /// Full-on burst to pull the armature across the gap.
    PullIn,
    /// Closed-loop hold current (sets the cam-out torque).
    Hold,
}

/// PWM output for the two switches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClutchDrive {
    /// Low-side FET enable.
    pub low_side: bool,
    /// High-side duty, 0..=DUTY_MAX.
    pub duty: u16,
}

pub const DUTY_MAX: u16 = 1000;

/// Shunt 0.1 Ω, op-amp gain 21, 3.3 V / 4096-count ADC -> mA per count.
pub fn adc_to_ma(counts: u16) -> u32 {
    // I = V / (0.1 * 21); V = counts * 3300 / 4095 mV
    counts as u32 * 3300 * 10 / (4095 * 21)
}

/// Bus divider 100k / 7.5k -> mV per ADC count.
pub fn adc_to_bus_mv(counts: u16) -> u32 {
    (counts as u32 * 3300 / 4095) * 1075 / 75
}

pub struct ClutchController {
    integ: f32,
    last_duty: u16,
    /// Coil resistance estimate for feed-forward, ohms (≈24 Ω from the coil design).
    pub coil_ohm: f32,
    pub kp: f32,
    pub ki: f32,
}

impl Default for ClutchController {
    fn default() -> Self {
        ClutchController { integ: 0.0, last_duty: 0, coil_ohm: 24.0, kp: 0.0008, ki: 0.02 }
    }
}

impl ClutchController {
    /// High-side duty of the previous step (for plausibility checks).
    pub fn last_duty(&self) -> u16 {
        self.last_duty
    }

    /// One control step. `dt_s` is the loop period.
    pub fn step(&mut self, cmd: ClutchCmd, hold_ma: u32, measured_ma: u32, bus_mv: u32, dt_s: f32) -> ClutchDrive {
        let d = self.step_inner(cmd, hold_ma, measured_ma, bus_mv, dt_s);
        self.last_duty = d.duty;
        d
    }

    fn step_inner(&mut self, cmd: ClutchCmd, hold_ma: u32, measured_ma: u32, bus_mv: u32, dt_s: f32) -> ClutchDrive {
        match cmd {
            ClutchCmd::Off => {
                self.integ = 0.0;
                ClutchDrive { low_side: false, duty: 0 }
            }
            ClutchCmd::PullIn => {
                self.integ = 0.0;
                ClutchDrive { low_side: true, duty: DUTY_MAX }
            }
            ClutchCmd::Hold => {
                let bus_v = (bus_mv as f32 / 1000.0).max(1.0);
                // feed-forward: duty that would give hold_ma into the nominal coil resistance
                let ff = hold_ma as f32 / 1000.0 * self.coil_ohm / bus_v;
                let err = hold_ma as f32 - measured_ma as f32;
                self.integ = (self.integ + err * self.ki * dt_s).clamp(-0.5, 0.5);
                let d = (ff + err * self.kp + self.integ).clamp(0.0, 1.0);
                ClutchDrive { low_side: true, duty: (d * DUTY_MAX as f32) as u16 }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn off_and_pull_in() {
        let mut c = ClutchController::default();
        assert_eq!(c.step(ClutchCmd::Off, 350, 0, 36_000, 0.001), ClutchDrive { low_side: false, duty: 0 });
        assert_eq!(c.step(ClutchCmd::PullIn, 350, 0, 36_000, 0.001), ClutchDrive { low_side: true, duty: DUTY_MAX });
    }

    #[test]
    fn hold_converges_on_a_simulated_coil() {
        // first-order coil model: I' = (V*duty - I*R) / L, L = 0.1 H, R = 24 Ω
        let mut c = ClutchController::default();
        let (r, l, dt) = (26.0f32, 0.1f32, 0.001f32); // real coil a bit off the 24 Ω estimate
        let mut i = 0.0f32;
        for _ in 0..3000 {
            let d = c.step(ClutchCmd::Hold, 350, (i * 1000.0) as u32, 36_000, dt);
            let v = 36.0 * d.duty as f32 / DUTY_MAX as f32;
            i += (v - i * r) / l * dt;
        }
        assert!((i * 1000.0 - 350.0).abs() < 15.0, "settled at {} mA", i * 1000.0);
    }

    #[test]
    fn adc_scaling() {
        // 1.5 A -> 0.15 V * 21 = 3.15 V -> ~3909 counts
        let ma = adc_to_ma(3909);
        assert!((1490..=1510).contains(&ma), "{ma}");
        // 36 V bus -> 36 * 7.5 / 107.5 = 2.512 V -> 3117 counts
        let mv = adc_to_bus_mv(3117);
        assert!((35_700..=36_300).contains(&mv), "{mv}");
    }
}
