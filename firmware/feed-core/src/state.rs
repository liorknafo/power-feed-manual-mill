//! The power-feed state machine (docs/design.md §3), as a pure function of inputs.
//!
//! ```text
//! INTERLOCK -(switch centre)-> IDLE -(L/R)-> ENGAGING -(teeth meshed)-> RUNNING
//!                               ^                                        |(centre / flip / limit)
//!                               +---- RELEASING <---- STOPPING <---------+
//! any state --(E-stop, driver error, stall, bus V, switch wiring, no engage)--> FAULT
//! FAULT --(switch centre and cause gone)--> IDLE
//! ```
//! Rules enforced here:
//! * the clutch is only energised with the motor stopped (plus the slow engagement jog), and only
//!   released after the motor has stopped and the driver is disabled;
//! * a fault drops the driver enable and the clutch in the same step;
//! * nothing starts moving until the switch has been seen in the centre (at power-up, after a
//!   fault, after a limit stop);
//! * a limit switch blocks motion toward that limit only.

use crate::clutch::ClutchCmd;
use crate::settings::Settings;
use crate::units::{self, MICROSTEPS};

/// 3-position switch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Center,
    Left,
    Right,
    /// Both contacts closed: wiring fault.
    Invalid,
}

/// Feed direction. `Right` is positive motor velocity (flip the motor wires or `Settings` sign later
/// if the table moves the wrong way).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    Left,
    Right,
}

impl Dir {
    pub fn sign(self) -> i32 {
        match self {
            Dir::Left => -1,
            Dir::Right => 1,
        }
    }
    fn of(mode: Mode) -> Option<Dir> {
        match mode {
            Mode::Left => Some(Dir::Left),
            Mode::Right => Some(Dir::Right),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    EStop,
    DriverError,
    Stall,
    OverVoltage,
    UnderVoltage,
    SwitchWiring,
    ClutchNoEngage,
    StopTimeout,
}

impl Fault {
    pub fn text(self) -> &'static str {
        match self {
            Fault::EStop => "E-STOP",
            Fault::DriverError => "DRIVER FAULT",
            Fault::Stall => "STALL / JAM",
            Fault::OverVoltage => "BUS OVERVOLT",
            Fault::UnderVoltage => "NO 36V",
            Fault::SwitchWiring => "SWITCH WIRING",
            Fault::ClutchNoEngage => "CLUTCH NO ENGAGE",
            Fault::StopTimeout => "STOP TIMEOUT",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EngagePhase {
    /// Driver enabled, motor holding.
    Enable,
    /// Coil full-on.
    PullIn,
    /// Slow jog until the output encoder follows the motor.
    Jog { motor0: i32, enc0: i32, t_jog: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum State {
    Interlock,
    Idle,
    Engaging { dir: Dir, phase: EngagePhase, t0: u32 },
    Running { dir: Dir },
    Stopping { dir: Dir, t0: u32 },
    /// `coil_off` = second phase (driver already off, coil now off).
    Releasing { coil_off: bool, t0: u32 },
    Fault(Fault),
}

impl State {
    pub fn name(&self) -> &'static str {
        match self {
            State::Interlock => "CENTRE SWITCH",
            State::Idle => "READY",
            State::Engaging { .. } => "ENGAGING",
            State::Running { .. } => "RUN",
            State::Stopping { .. } => "STOPPING",
            State::Releasing { .. } => "RELEASE",
            State::Fault(f) => f.text(),
        }
    }
}

/// Everything the controller needs to know, sampled once per control step.
#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub now_ms: u32,
    pub mode: Mode,
    pub rapid: bool,
    pub estop_ok: bool,
    pub limit_left: bool,
    pub limit_right: bool,
    /// TMC5160 driver error (overtemperature, short) from SPI status / DIAG0.
    pub driver_fault: bool,
    /// Encoder deviation (TMC5160 ENC_STATUS.deviation_warn) or the Pico's own cross-check.
    pub deviation: bool,
    /// TMC5160 reports zero velocity.
    pub standstill: bool,
    /// TMC5160 XACTUAL (microsteps).
    pub motor_usteps: i32,
    /// Output-side encoder position (counts).
    pub output_counts: i32,
    pub bus_mv: u32,
    /// Speed-dial detents since the last step.
    pub dial_detents: i32,
    pub dial_fast: bool,
}

impl Inputs {
    /// A quiet, healthy machine with the switch centred; tests start from here.
    pub fn idle(now_ms: u32) -> Inputs {
        Inputs {
            now_ms,
            mode: Mode::Center,
            rapid: false,
            estop_ok: true,
            limit_left: false,
            limit_right: false,
            driver_fault: false,
            deviation: false,
            standstill: true,
            motor_usteps: 0,
            output_counts: 0,
            bus_mv: 36_000,
            dial_detents: 0,
            dial_fast: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    /// Decelerate to zero and hold (velocity mode, v = 0).
    Stop,
    /// Signed speed in microsteps/s (the TMC5160 ramps to it).
    Velocity(f32),
    /// Position move to an absolute XACTUAL target at a given speed (engagement jog).
    Jog { target: i32, speed: f32 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outputs {
    pub driver_enable: bool,
    pub motion: Motion,
    pub clutch: ClutchCmd,
    /// Arm the encoder-deviation (stall) check.
    pub deviation_monitor: bool,
    /// One-shot: set X_ENC := XACTUAL and clear the deviation flag before arming the monitor.
    pub resync_encoder: bool,
}

const DRIVER_SETTLE_MS: u32 = 20;
const DRIVER_OFF_MS: u32 = 5;
const JOG_TIMEOUT_MS: u32 = 3000;
const STOP_TIMEOUT_MS: u32 = 5000;

pub struct Controller {
    pub settings: Settings,
    pub state: State,
    /// Feed setpoint, mm/min.
    pub feed_mm_min: f32,
    /// Movement is blocked until the switch has been seen in the centre.
    need_center: bool,
    /// Which limit stopped us last (for the display).
    pub limit_hold: Option<Dir>,
}

impl Controller {
    pub fn new(settings: Settings) -> Self {
        Controller {
            feed_mm_min: settings.feed_default_mm_min,
            settings,
            state: State::Interlock,
            need_center: true,
            limit_hold: None,
        }
    }

    fn active_fault(&self, i: &Inputs) -> Option<Fault> {
        if i.mode == Mode::Invalid {
            return Some(Fault::SwitchWiring);
        }
        if !i.estop_ok {
            return Some(Fault::EStop);
        }
        if i.driver_fault {
            return Some(Fault::DriverError);
        }
        if i.bus_mv > self.settings.bus_max_mv {
            return Some(Fault::OverVoltage);
        }
        if i.bus_mv < self.settings.bus_min_mv {
            return Some(Fault::UnderVoltage);
        }
        if i.deviation && self.monitoring() {
            return Some(Fault::Stall);
        }
        None
    }

    fn monitoring(&self) -> bool {
        matches!(self.state, State::Running { .. } | State::Stopping { .. })
    }

    fn limit_toward(i: &Inputs, dir: Dir) -> bool {
        match dir {
            Dir::Left => i.limit_left,
            Dir::Right => i.limit_right,
        }
    }

    /// Run one control step.
    pub fn step(&mut self, i: &Inputs) -> Outputs {
        let s = self.settings;
        let now = i.now_ms;

        // speed dial works in every state
        if i.dial_detents != 0 {
            let step = units::dial_step(self.feed_mm_min, i.dial_fast);
            self.feed_mm_min = (self.feed_mm_min + i.dial_detents as f32 * step).clamp(s.feed_min_mm_min, s.feed_max_mm_min);
        }

        if i.mode == Mode::Center {
            self.need_center = false;
            self.limit_hold = None;
        }

        // faults pre-empt everything
        let mut resync = false;
        if !matches!(self.state, State::Fault(_)) {
            if let Some(f) = self.active_fault(i) {
                self.state = State::Fault(f);
                self.need_center = true;
            }
        }

        self.state = match self.state {
            State::Interlock => {
                if i.mode == Mode::Center {
                    State::Idle
                } else {
                    State::Interlock
                }
            }
            State::Fault(f) => {
                if i.mode == Mode::Center && self.active_fault(i).is_none() {
                    State::Idle
                } else {
                    State::Fault(f)
                }
            }
            State::Idle => match Dir::of(i.mode) {
                Some(dir) if !self.need_center => {
                    if Self::limit_toward(i, dir) {
                        self.need_center = true;
                        self.limit_hold = Some(dir);
                        State::Idle
                    } else {
                        State::Engaging { dir, phase: EngagePhase::Enable, t0: now }
                    }
                }
                _ => State::Idle,
            },
            State::Engaging { dir, phase, t0 } => {
                if Dir::of(i.mode) != Some(dir) || Self::limit_toward(i, dir) {
                    if Self::limit_toward(i, dir) {
                        self.need_center = true;
                        self.limit_hold = Some(dir);
                    }
                    State::Releasing { coil_off: false, t0: now }
                } else {
                    match phase {
                        EngagePhase::Enable if now.wrapping_sub(t0) >= DRIVER_SETTLE_MS => {
                            State::Engaging { dir, phase: EngagePhase::PullIn, t0: now }
                        }
                        EngagePhase::PullIn if now.wrapping_sub(t0) >= s.clutch_pull_in_ms => State::Engaging {
                            dir,
                            phase: EngagePhase::Jog { motor0: i.motor_usteps, enc0: i.output_counts, t_jog: now },
                            t0,
                        },
                        EngagePhase::Jog { motor0, enc0, t_jog } => {
                            let out_moved = (i.output_counts - enc0) * dir.sign();
                            let motor_moved = (i.motor_usteps - motor0) * dir.sign();
                            let jog_len = (s.jog_max_steps * MICROSTEPS) as i32;
                            if out_moved >= s.engage_detect_counts as i32 {
                                resync = true;
                                State::Running { dir }
                            } else if (motor_moved >= jog_len - (MICROSTEPS as i32 / 2) && i.standstill)
                                || now.wrapping_sub(t_jog) > JOG_TIMEOUT_MS
                                || out_moved < -(s.engage_detect_counts as i32)
                            {
                                self.need_center = true;
                                State::Fault(Fault::ClutchNoEngage)
                            } else {
                                State::Engaging { dir, phase, t0 }
                            }
                        }
                        _ => State::Engaging { dir, phase, t0 },
                    }
                }
            }
            State::Running { dir } => {
                if Self::limit_toward(i, dir) {
                    self.need_center = true;
                    self.limit_hold = Some(dir);
                    State::Stopping { dir, t0: now }
                } else if Dir::of(i.mode) != Some(dir) {
                    // centre, or flipped straight to the other side: full stop and release first
                    State::Stopping { dir, t0: now }
                } else {
                    State::Running { dir }
                }
            }
            State::Stopping { dir, t0 } => {
                if i.standstill && now != t0 {
                    State::Releasing { coil_off: false, t0: now }
                } else if now.wrapping_sub(t0) > STOP_TIMEOUT_MS {
                    self.need_center = true;
                    State::Fault(Fault::StopTimeout)
                } else {
                    State::Stopping { dir, t0 }
                }
            }
            State::Releasing { coil_off, t0 } => {
                if !coil_off && now.wrapping_sub(t0) >= DRIVER_OFF_MS {
                    State::Releasing { coil_off: true, t0: now }
                } else if coil_off && now.wrapping_sub(t0) >= s.clutch_release_ms {
                    State::Idle
                } else {
                    State::Releasing { coil_off, t0 }
                }
            }
        };

        self.outputs(i, resync)
    }

    fn outputs(&self, i: &Inputs, resync: bool) -> Outputs {
        let s = &self.settings;
        let off = Outputs {
            driver_enable: false,
            motion: Motion::Stop,
            clutch: ClutchCmd::Off,
            deviation_monitor: false,
            resync_encoder: false,
        };
        match self.state {
            State::Interlock | State::Idle | State::Fault(_) => off,
            State::Engaging { dir, phase, .. } => match phase {
                EngagePhase::Enable => Outputs { driver_enable: true, ..off },
                EngagePhase::PullIn => Outputs { driver_enable: true, clutch: ClutchCmd::PullIn, ..off },
                EngagePhase::Jog { motor0, .. } => Outputs {
                    driver_enable: true,
                    clutch: ClutchCmd::Hold,
                    motion: Motion::Jog {
                        target: motor0 + dir.sign() * (s.jog_max_steps * MICROSTEPS) as i32,
                        speed: (s.jog_speed_steps_s * MICROSTEPS) as f32,
                    },
                    ..off
                },
            },
            State::Running { dir } => {
                let mm_min = if i.rapid { s.rapid_mm_min } else { self.feed_mm_min };
                Outputs {
                    driver_enable: true,
                    clutch: ClutchCmd::Hold,
                    motion: Motion::Velocity(dir.sign() as f32 * units::feed_to_usteps_s(s, mm_min)),
                    deviation_monitor: !resync,
                    resync_encoder: resync,
                }
            }
            State::Stopping { .. } => Outputs { driver_enable: true, clutch: ClutchCmd::Hold, deviation_monitor: true, ..off },
            // driver off first (unloads the teeth), then the coil
            State::Releasing { coil_off, .. } => Outputs { clutch: if coil_off { ClutchCmd::Off } else { ClutchCmd::Hold }, ..off },
        }
    }

    /// Speed currently shown on the display, mm/min.
    pub fn display_feed(&self, rapid: bool) -> f32 {
        match self.state {
            State::Running { .. } if rapid => self.settings.rapid_mm_min,
            _ => self.feed_mm_min,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Small simulator: TMC5160 ramps instantly, clutch meshes after `mesh_after` motor microsteps of jog.
    struct Sim {
        c: Controller,
        i: Inputs,
        mesh_after: Option<i32>,
        engaged: bool,
        log: Vec<Outputs>,
    }

    impl Sim {
        fn new() -> Sim {
            Sim { c: Controller::new(Settings::default()), i: Inputs::idle(0), mesh_after: Some(256 * 3), engaged: false, log: vec![] }
        }
        fn tick(&mut self, n: u32) -> Outputs {
            let mut o = None;
            for _ in 0..n {
                self.i.now_ms += 1;
                let out = self.c.step(&self.i);
                // crude plant
                match out.motion {
                    Motion::Jog { target, .. } if out.driver_enable => {
                        let d = (target - self.i.motor_usteps).signum() * 64;
                        self.i.motor_usteps += d;
                        self.i.standstill = d == 0;
                        if let Some(m) = self.mesh_after {
                            if self.i.motor_usteps.abs() >= m && out.clutch != ClutchCmd::Off {
                                self.engaged = true;
                            }
                        }
                        if self.engaged {
                            self.i.output_counts += d.signum() * 3;
                        }
                    }
                    Motion::Velocity(v) if out.driver_enable => {
                        self.i.standstill = v == 0.0;
                        self.i.motor_usteps += (v / 1000.0) as i32;
                    }
                    _ => self.i.standstill = true,
                }
                if out.clutch == ClutchCmd::Off {
                    self.engaged = false;
                }
                self.log.push(out);
                o = Some(out);
            }
            o.unwrap()
        }
        fn run_to_running(&mut self, mode: Mode) {
            self.i.mode = Mode::Center;
            self.tick(2);
            self.i.mode = mode;
            for _ in 0..2000 {
                self.tick(1);
                if matches!(self.c.state, State::Running { .. }) {
                    return;
                }
            }
            panic!("never reached Running: {:?}", self.c.state);
        }
    }

    #[test]
    fn powers_up_in_interlock_until_switch_centred() {
        let mut s = Sim::new();
        s.i.mode = Mode::Right;
        let o = s.tick(100);
        assert_eq!(s.c.state, State::Interlock);
        assert!(!o.driver_enable && o.clutch == ClutchCmd::Off);
        s.i.mode = Mode::Center;
        s.tick(1);
        assert_eq!(s.c.state, State::Idle);
    }

    #[test]
    fn engage_sequence_order() {
        let mut s = Sim::new();
        s.run_to_running(Mode::Right);
        // driver enabled before the clutch, clutch pulled in before any motion
        let first_drv = s.log.iter().position(|o| o.driver_enable).unwrap();
        let first_clutch = s.log.iter().position(|o| o.clutch != ClutchCmd::Off).unwrap();
        let first_move = s.log.iter().position(|o| !matches!(o.motion, Motion::Stop)).unwrap();
        assert!(first_drv < first_clutch && first_clutch < first_move);
        let pullin: Vec<_> = s.log.iter().filter(|o| o.clutch == ClutchCmd::PullIn).collect();
        assert!(pullin.iter().all(|o| matches!(o.motion, Motion::Stop)), "no motion during pull-in");
        // the step that enters Running re-syncs the encoder before arming the stall monitor
        let enter = s.log.iter().position(|o| matches!(o.motion, Motion::Velocity(_))).unwrap();
        assert!(s.log[enter].resync_encoder && !s.log[enter].deviation_monitor);
        let o = s.tick(1);
        assert!(o.deviation_monitor && !o.resync_encoder);
        assert!(matches!(o.motion, Motion::Velocity(v) if v > 0.0));
    }

    #[test]
    fn stop_then_release_driver_before_coil() {
        let mut s = Sim::new();
        s.run_to_running(Mode::Left);
        s.log.clear();
        s.i.mode = Mode::Center;
        for _ in 0..200 {
            s.tick(1);
            if s.c.state == State::Idle {
                break;
            }
        }
        assert_eq!(s.c.state, State::Idle);
        // the coil is never switched off while the driver is still enabled or the motor commanded to move
        for o in &s.log {
            if o.clutch == ClutchCmd::Off {
                assert!(!o.driver_enable);
            }
        }
        let drv_off = s.log.iter().position(|o| !o.driver_enable).unwrap();
        let coil_off = s.log.iter().position(|o| o.clutch == ClutchCmd::Off).unwrap();
        assert!(drv_off < coil_off);
    }

    #[test]
    fn direct_reversal_goes_through_idle() {
        let mut s = Sim::new();
        s.run_to_running(Mode::Left);
        s.i.mode = Mode::Right; // flip straight across
        let mut seen = vec![];
        for _ in 0..3000 {
            s.tick(1);
            let n = s.c.state.name();
            if seen.last() != Some(&n) {
                seen.push(n);
            }
            if matches!(s.c.state, State::Running { dir: Dir::Right }) {
                break;
            }
        }
        assert_eq!(seen, ["STOPPING", "RELEASE", "READY", "ENGAGING", "RUN"]);
    }

    #[test]
    fn estop_drops_everything_in_the_same_step_and_latches() {
        let mut s = Sim::new();
        s.run_to_running(Mode::Right);
        s.i.estop_ok = false;
        let o = s.tick(1);
        assert_eq!(s.c.state, State::Fault(Fault::EStop));
        assert!(!o.driver_enable && o.clutch == ClutchCmd::Off);
        // releasing the E-stop does not restart while the switch is still on
        s.i.estop_ok = true;
        let o = s.tick(500);
        assert!(matches!(s.c.state, State::Fault(_)) && !o.driver_enable);
        // centre clears it, and only then can a new run start
        s.i.mode = Mode::Center;
        s.tick(1);
        assert_eq!(s.c.state, State::Idle);
    }

    #[test]
    fn fault_does_not_clear_while_cause_persists() {
        let mut s = Sim::new();
        s.tick(2);
        s.i.estop_ok = false;
        s.tick(5);
        s.i.mode = Mode::Center;
        s.tick(50);
        assert_eq!(s.c.state, State::Fault(Fault::EStop));
    }

    #[test]
    fn stall_only_counts_while_monitoring() {
        let mut s = Sim::new();
        s.tick(2);
        s.i.deviation = true; // stale flag while idle is ignored
        s.tick(10);
        assert_eq!(s.c.state, State::Idle);
        s.i.deviation = false;
        s.run_to_running(Mode::Right);
        s.tick(2);
        s.i.deviation = true;
        let o = s.tick(1);
        assert_eq!(s.c.state, State::Fault(Fault::Stall));
        assert!(!o.driver_enable && o.clutch == ClutchCmd::Off);
    }

    #[test]
    fn limit_blocks_only_its_direction_and_needs_centre() {
        let mut s = Sim::new();
        s.run_to_running(Mode::Right);
        s.i.limit_right = true;
        s.tick(200);
        assert_eq!(s.c.state, State::Idle);
        // limit released but switch still Right: must not restart by itself
        s.i.limit_right = false;
        s.tick(200);
        assert_eq!(s.c.state, State::Idle);
        // the other direction works after passing through centre
        s.i.limit_right = true;
        s.run_to_running(Mode::Left);
        // and Right stays blocked while its limit is active
        s.i.mode = Mode::Center;
        s.tick(200);
        s.i.mode = Mode::Right;
        s.tick(200);
        assert_eq!(s.c.state, State::Idle);
        assert_eq!(s.c.limit_hold, Some(Dir::Right));
    }

    #[test]
    fn clutch_that_never_meshes_faults() {
        let mut s = Sim::new();
        s.mesh_after = None;
        s.i.mode = Mode::Center;
        s.tick(2);
        s.i.mode = Mode::Left;
        s.tick(4000);
        assert_eq!(s.c.state, State::Fault(Fault::ClutchNoEngage));
    }

    #[test]
    fn switch_back_to_centre_during_engage_releases() {
        let mut s = Sim::new();
        s.mesh_after = None;
        s.i.mode = Mode::Center;
        s.tick(2);
        s.i.mode = Mode::Right;
        s.tick(30); // in pull-in
        assert!(matches!(s.c.state, State::Engaging { .. }));
        s.i.mode = Mode::Center;
        s.tick(100);
        assert_eq!(s.c.state, State::Idle);
    }

    #[test]
    fn bus_voltage_and_wiring_faults() {
        let mut s = Sim::new();
        s.tick(2);
        s.i.bus_mv = 45_000;
        s.tick(1);
        assert_eq!(s.c.state, State::Fault(Fault::OverVoltage));
        s.i.bus_mv = 36_000;
        s.i.mode = Mode::Invalid;
        s.tick(5);
        s.i.mode = Mode::Center;
        s.tick(1);
        assert_eq!(s.c.state, State::Idle);
        s.i.bus_mv = 5_000; // PSU off, Pico on USB
        s.tick(1);
        assert_eq!(s.c.state, State::Fault(Fault::UnderVoltage));
    }

    #[test]
    fn rapid_and_dial() {
        let mut s = Sim::new();
        s.i.dial_detents = 4; // 50 + 4*5 = 70
        s.tick(1);
        s.i.dial_detents = 0;
        assert_eq!(s.c.feed_mm_min, 70.0);
        s.i.dial_detents = -1000;
        s.tick(1);
        s.i.dial_detents = 0;
        assert_eq!(s.c.feed_mm_min, s.c.settings.feed_min_mm_min);
        s.run_to_running(Mode::Right);
        let slow = match s.tick(1).motion {
            Motion::Velocity(v) => v,
            m => panic!("{m:?}"),
        };
        s.i.rapid = true;
        let fast = match s.tick(1).motion {
            Motion::Velocity(v) => v,
            m => panic!("{m:?}"),
        };
        assert!(fast > slow * 10.0);
    }
}
