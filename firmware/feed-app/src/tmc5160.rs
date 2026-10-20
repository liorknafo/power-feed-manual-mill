//! TMC5160 over SPI (mode 3, ≤4 MHz with the internal clock). Register maths lives in feed-core::tmc.

use embassy_rp::gpio::Output;
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Blocking, Spi};
use feed_core::tmc::{self, reg, rampmode, spi_status};
use feed_core::Motion;

pub struct Tmc5160<'d> {
    spi: Spi<'d, SPI0, Blocking>,
    cs: Output<'d>,
    /// SPI_STATUS byte from the most recent transfer.
    pub status: u8,
    last_motion: Option<Motion>,
    amax: u32,
}

impl<'d> Tmc5160<'d> {
    pub fn new(spi: Spi<'d, SPI0, Blocking>, cs: Output<'d>) -> Self {
        Tmc5160 { spi, cs, status: 0, last_motion: None, amax: 1000 }
    }

    fn xfer(&mut self, tx: [u8; 5]) -> [u8; 5] {
        let mut buf = tx;
        self.cs.set_low();
        let _ = self.spi.blocking_transfer_in_place(&mut buf);
        self.cs.set_high();
        self.status = buf[0];
        buf
    }

    pub fn write(&mut self, addr: u8, value: u32) {
        self.xfer(tmc::write_datagram(addr, value));
    }

    /// Read a register (two datagrams: the reply to the second carries the value).
    pub fn read(&mut self, addr: u8) -> u32 {
        self.xfer(tmc::read_datagram(addr));
        let rx = self.xfer(tmc::read_datagram(addr));
        tmc::reply_value(&rx)
    }

    /// Full configuration. Returns false if the chip does not answer (IOIN version byte != 0x30).
    /// `usteps_per_count` is signed: negative reverses the encoder direction.
    pub fn init(&mut self, run_ma: u32, hold_ma: u32, usteps_per_count: f32, amax: u32, motor_reverse: bool) -> bool {
        let version = self.read(reg::IOIN) >> 24;
        if version != 0x30 {
            return false;
        }
        self.clear_gstat();
        for (addr, val) in tmc::init_sequence(run_ma, hold_ma, usteps_per_count, motor_reverse) {
            self.write(addr, val);
        }
        self.set_accel(amax);
        self.write(reg::RAMPMODE, rampmode::VELOCITY_POS);
        self.write(reg::VMAX, 0);
        self.write(reg::XACTUAL, 0);
        self.write(reg::X_ENC, 0);
        self.write(reg::ENC_DEVIATION, 0);
        self.last_motion = Some(Motion::Stop);
        self.clear_gstat();
        self.status & spi_status::RESET_FLAG == 0
    }

    /// GSTAT is write-1-to-clear: clears the reset, driver-error and undervoltage flags
    /// (the driver error comes back at once if its cause is still present).
    pub fn clear_gstat(&mut self) {
        self.write(reg::GSTAT, 0b111);
    }

    /// Clear only the latched driver-error flag (GSTAT.drv_err); leaves the reset flag visible.
    pub fn clear_driver_error(&mut self) {
        self.write(reg::GSTAT, 0b010);
    }

    /// The chip reset since it was configured (power loss on VM, brown-out).
    pub fn was_reset(&self) -> bool {
        self.status & spi_status::RESET_FLAG != 0
    }

    pub fn set_accel(&mut self, amax: u32) {
        self.amax = amax;
        self.write(reg::AMAX, amax);
        self.write(reg::DMAX, amax);
    }

    #[allow(dead_code)] // for live current changes; init() already applies the settings
    pub fn set_currents(&mut self, run_ma: u32, hold_ma: u32) {
        let (gs, irun) = tmc::current_to_scaler(run_ma);
        self.write(reg::GLOBALSCALER, gs);
        self.write(reg::IHOLD_IRUN, tmc::ihold_irun(tmc::cs_for(hold_ma, run_ma), irun, 6));
    }

    /// Apply a motion command; registers are only written when the command changes.
    pub fn apply(&mut self, m: Motion) {
        if self.last_motion == Some(m) {
            return;
        }
        match m {
            Motion::Stop => {
                if matches!(self.last_motion, Some(Motion::Jog { .. })) {
                    self.write(reg::RAMPMODE, rampmode::VELOCITY_POS);
                }
                self.write(reg::VMAX, 0);
            }
            Motion::Velocity(v) => {
                self.write(reg::RAMPMODE, if v >= 0.0 { rampmode::VELOCITY_POS } else { rampmode::VELOCITY_NEG });
                self.write(reg::VMAX, tmc::vmax_from_usteps_s(if v >= 0.0 { v } else { -v }));
            }
            Motion::Jog { target, speed } => {
                // target first, then position mode, then speed: never heads for a stale target
                // or keeps the old velocity-mode direction
                self.write(reg::XTARGET, target as u32);
                self.write(reg::RAMPMODE, rampmode::POSITION);
                self.write(reg::VMAX, tmc::vmax_from_usteps_s(speed));
            }
        }
        self.last_motion = Some(m);
    }

    /// Forget the cached command (after a driver disable the next command is always re-sent).
    pub fn invalidate(&mut self) {
        self.last_motion = None;
    }

    pub fn xactual(&mut self) -> i32 {
        self.read(reg::XACTUAL) as i32
    }

    /// Encoder position, already scaled to microsteps by ENC_CONST.
    pub fn x_enc(&mut self) -> i32 {
        self.read(reg::X_ENC) as i32
    }

    pub fn deviation(&mut self) -> bool {
        self.read(reg::ENC_STATUS) & 0b10 != 0
    }

    /// X_ENC := XACTUAL, clear the deviation flag, then (re)arm or disarm the check.
    pub fn resync_encoder(&mut self) {
        let x = self.read(reg::XACTUAL);
        self.write(reg::X_ENC, x);
        self.write(reg::ENC_STATUS, 0b11);
    }

    pub fn set_deviation_limit(&mut self, usteps: u32) {
        self.write(reg::ENC_DEVIATION, usteps);
    }

    pub fn standstill(&self) -> bool {
        self.status & spi_status::STANDSTILL != 0
    }

    pub fn driver_error(&self) -> bool {
        self.status & spi_status::DRIVER_ERROR != 0
    }
}
