//! Mill power feed firmware (Raspberry Pi Pico / RP2040).
//!
//! * `control` runs every 1 ms on a high-priority interrupt executor: reads the panel, the TMC5160 and
//!   the ADCs, steps the `feed_core::Controller` state machine, and applies its outputs.
//! * `display` runs in thread mode at ~10 Hz and draws the OLED; the control loop pre-empts it.
//! * The hardware interlock (NAND/AND gates with E_OK) cuts the driver and clutch independently of
//!   this code; firmware faults are an additional layer. A watchdog resets the Pico if the loop hangs,
//!   and the pull-downs on the enable lines keep everything off through the reset.
//!
//! Pin map: docs/design.md §4.

#![no_std]
#![no_main]

mod oled;
mod storage;
mod tmc5160;
mod ui;

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_rp::adc::{self, Adc, Channel};
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::peripherals::*;
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::pwm::SetDutyCycle;
use feed_core::clutch::{adc_to_bus_mv, adc_to_ma, ClutchController, DUTY_MAX};
use feed_core::menu::{Button, Menu, Press};
use feed_core::quadrature::{DialAccel, Quadrature};
use feed_core::state::{Inputs, Mode, State};
use feed_core::{units, Controller, Settings};
use panic_probe as _;
use static_cell::StaticCell;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    EXECUTOR_HIGH.on_interrupt()
}

/// Clutch high-side PWM: 125 MHz / (div 62.5) / (top 1000) = 2 kHz (design.md §7.6 limit).
const PWM_TOP: u16 = DUTY_MAX - 1;

struct ControlHw {
    // panel / sensors
    mode_l: Input<'static>,
    mode_r: Input<'static>,
    rapid: Input<'static>,
    dial_a: Input<'static>,
    dial_b: Input<'static>,
    dial_sw: Input<'static>,
    lim_l: Input<'static>,
    lim_r: Input<'static>,
    e_ok: Input<'static>,
    // outputs
    drv_en: Output<'static>,
    clutch_ls: Output<'static>,
    clutch_pwm: Pwm<'static>,
    // buses
    tmc: tmc5160::Tmc5160<'static>,
    adc: Adc<'static, adc::Blocking>,
    vm_sense: Channel<'static>,
    clutch_sense: Channel<'static>,
    storage: storage::Storage<'static>,
    watchdog: Watchdog,
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    info!("power feed starting");

    // outputs first, all OFF (the board also has pull-downs on these lines)
    let drv_en = Output::new(p.PIN_20, Level::Low);
    let clutch_ls = Output::new(p.PIN_8, Level::Low);
    let mut pwm_cfg = pwm::Config::default();
    pwm_cfg.top = PWM_TOP;
    pwm_cfg.divider = fixed::FixedU16::from_num(62.5);
    pwm_cfg.compare_a = 0;
    let clutch_pwm = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, pwm_cfg);

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 2_000_000;
    spi_cfg.phase = spi::Phase::CaptureOnSecondTransition;
    spi_cfg.polarity = spi::Polarity::IdleHigh;
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
    let tmc = tmc5160::Tmc5160::new(spi, Output::new(p.PIN_17, Level::High));

    let hw = ControlHw {
        mode_l: Input::new(p.PIN_13, Pull::None), // external pull-ups on the board
        mode_r: Input::new(p.PIN_14, Pull::None),
        rapid: Input::new(p.PIN_15, Pull::None),
        dial_a: Input::new(p.PIN_10, Pull::None),
        dial_b: Input::new(p.PIN_11, Pull::None),
        dial_sw: Input::new(p.PIN_12, Pull::None),
        lim_l: Input::new(p.PIN_6, Pull::None),
        lim_r: Input::new(p.PIN_7, Pull::None),
        // E_OK_SENSE comes through 100k: the internal pull MUST be off (design.md §4)
        e_ok: Input::new(p.PIN_3, Pull::None),
        drv_en,
        clutch_ls,
        clutch_pwm,
        tmc,
        adc: Adc::new_blocking(p.ADC, adc::Config::default()),
        vm_sense: Channel::new_pin(p.PIN_26, Pull::None),
        clutch_sense: Channel::new_pin(p.PIN_27, Pull::None),
        storage: storage::Storage::new(Flash::new_blocking(p.FLASH)),
        watchdog: Watchdog::new(p.WATCHDOG),
    };

    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000;
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, i2c_cfg);

    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    high.spawn(control(hw).unwrap());

    let low = EXECUTOR_LOW.init(Executor::new());
    low.run(|spawner| spawner.spawn(display(i2c).unwrap()));
}

fn read_mode(hw: &ControlHw) -> Mode {
    // contacts switch to GND: low = selected
    match (hw.mode_l.is_low(), hw.mode_r.is_low()) {
        (true, true) => Mode::Invalid,
        (true, false) => Mode::Left,
        (false, true) => Mode::Right,
        (false, false) => Mode::Center,
    }
}

fn configure_tmc(hw: &mut ControlHw, s: &Settings) -> bool {
    let usteps_per_count = units::usteps_per_count(s);
    let amax = feed_core::tmc::amax_from_usteps_s2(units::accel_to_usteps_s2(s, s.accel_mm_s2));
    hw.tmc.init(s.run_current_ma, s.hold_current_ma, usteps_per_count, amax)
}

#[embassy_executor::task]
async fn control(mut hw: ControlHw) {
    let settings = hw.storage.load().unwrap_or_else(|| {
        warn!("no valid settings in flash, using defaults");
        Settings::default()
    });
    let tmc_ok = configure_tmc(&mut hw, &settings);
    if !tmc_ok {
        warn!("TMC5160 not responding");
    }
    let mut ctrl = Controller::new(settings);
    let mut clutch = ClutchController::default();
    let mut quad = Quadrature::new(hw.dial_a.is_high(), hw.dial_b.is_high());
    let mut accel = DialAccel::new();
    let mut button = Button::new();
    let mut menu = Menu::new();
    let mut fine = false;
    let mut saved_until = 0u32;
    let mut deviation_armed = false;
    let mut bus_mv = 0u32;
    let mut clutch_ma = 0u32;
    let t_start = Instant::now();

    hw.watchdog.start(Duration::from_millis(100));
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut n: u32 = 0;
    loop {
        ticker.next().await;
        hw.watchdog.feed(Duration::from_millis(100));
        n = n.wrapping_add(1);
        let now = t_start.elapsed().as_millis() as u32;

        // ---- analog (bus every 10 ms, clutch current every tick)
        if n % 10 == 0 {
            if let Ok(c) = hw.adc.blocking_read(&mut hw.vm_sense) {
                bus_mv = adc_to_bus_mv(c);
            }
        }
        if let Ok(c) = hw.adc.blocking_read(&mut hw.clutch_sense) {
            clutch_ma = (clutch_ma * 7 + adc_to_ma(c)) / 8; // light smoothing of PWM ripple
        }

        // ---- dial + button (menu only while idle)
        let detents = quad.update(hw.dial_a.is_high(), hw.dial_b.is_high());
        let fast = detents != 0 && accel.is_fast(now);
        let idle = matches!(ctrl.state, State::Idle | State::Interlock);
        let mut dial_to_feed = detents;
        match button.update(hw.dial_sw.is_low(), now) {
            Some(Press::Long) if menu.active => {
                menu.active = false;
                if hw.storage.save(&ctrl.settings) {
                    saved_until = now + 1500;
                }
                configure_tmc(&mut hw, &ctrl.settings);
            }
            Some(Press::Long) if idle => menu.active = true,
            Some(Press::Short) if menu.active => menu.next(),
            Some(Press::Short) => fine = !fine,
            _ => {}
        }
        if menu.active {
            if !idle {
                menu.active = false; // never edit while the feed is running
            } else if detents != 0 {
                ctrl.settings = menu.item().adjust(&ctrl.settings, detents);
            }
            dial_to_feed = 0;
        }

        // ---- TMC5160 status (every read refreshes the SPI status byte)
        let motor_usteps = hw.tmc.xactual();
        let enc_usteps = hw.tmc.x_enc();
        let deviation = deviation_armed && hw.tmc.deviation();
        let usteps_per_count = units::usteps_per_count(&ctrl.settings);

        let inputs = Inputs {
            now_ms: now,
            mode: read_mode(&hw),
            rapid: hw.rapid.is_low(),
            estop_ok: hw.e_ok.is_high(),
            limit_left: hw.lim_l.is_high(), // NC to GND: high = hit or broken wire
            limit_right: hw.lim_r.is_high(),
            driver_fault: hw.tmc.driver_error(),
            deviation,
            standstill: hw.tmc.standstill(),
            motor_usteps,
            output_counts: (enc_usteps as f32 / usteps_per_count) as i32,
            bus_mv,
            dial_detents: if fine { dial_to_feed } else { dial_to_feed * 2 },
            dial_fast: fast,
        };
        let out = ctrl.step(&inputs);

        // ---- apply: driver enable, motion, stall monitor, clutch
        hw.drv_en.set_level(if out.driver_enable { Level::High } else { Level::Low });
        if out.driver_enable {
            hw.tmc.apply(out.motion);
        } else {
            hw.tmc.apply(feed_core::Motion::Stop);
            hw.tmc.invalidate();
        }
        if out.resync_encoder {
            hw.tmc.resync_encoder();
        }
        if out.deviation_monitor != deviation_armed {
            let lim = if out.deviation_monitor { ctrl.settings.stall_tolerance_steps * units::MICROSTEPS } else { 0 };
            hw.tmc.set_deviation_limit(lim);
            deviation_armed = out.deviation_monitor;
        }
        let drive = clutch.step(out.clutch, ctrl.settings.clutch_hold_ma, clutch_ma, bus_mv, 0.001);
        hw.clutch_ls.set_level(if drive.low_side { Level::High } else { Level::Low });
        hw.clutch_pwm.set_duty_cycle_fraction(drive.duty, DUTY_MAX).ok();

        // ---- publish for the display (every 50 ms)
        if n % 50 == 0 {
            let st = ui::Status {
                state: ctrl.state,
                feed_mm_min: ctrl.display_feed(inputs.rapid),
                rapid: inputs.rapid && matches!(ctrl.state, State::Running { .. }),
                fine,
                bus_mv,
                limit: ctrl.limit_hold,
                menu: if menu.active { Some((menu.item(), menu.item().value(&ctrl.settings))) } else { None },
                saved: now < saved_until,
                tmc_ok,
            };
            ui::STATUS.lock(|c| c.set(st));
        }
    }
}

#[embassy_executor::task]
async fn display(i2c: I2c<'static, I2C0, i2c::Blocking>) {
    let mut d = oled::Oled::new(i2c, oled::Controller::Sh1106);
    if !d.init() {
        warn!("OLED not responding");
    }
    loop {
        let st = ui::STATUS.lock(|c| c.get());
        d.clear();
        ui::draw(&mut d, &st);
        d.flush();
        Timer::after(Duration::from_millis(100)).await;
    }
}
