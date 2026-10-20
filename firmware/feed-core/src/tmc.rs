//! TMC5160 register maths (datasheet rev 1.01: §4 SPI, §6 registers, §9 current, §20 encoder).
//! Pure functions, so the register values can be unit-tested.

/// Internal clock (CLK pin tied to GND). Nominal 12 MHz; the real oscillator is within a few %.
pub const FCLK_HZ: f32 = 12_000_000.0;
/// Sense resistors on the board.
pub const RSENSE_OHM: f32 = 0.050;
/// Full-scale sense voltage.
pub const VFS: f32 = 0.325;

/// Register addresses.
pub mod reg {
    pub const GCONF: u8 = 0x00;
    pub const GSTAT: u8 = 0x01;
    pub const IOIN: u8 = 0x04;
    pub const GLOBALSCALER: u8 = 0x0B;
    pub const IHOLD_IRUN: u8 = 0x10;
    pub const TPOWERDOWN: u8 = 0x11;
    pub const RAMPMODE: u8 = 0x20;
    pub const XACTUAL: u8 = 0x21;
    pub const VACTUAL: u8 = 0x22;
    pub const VSTART: u8 = 0x23;
    pub const A1: u8 = 0x24;
    pub const V1: u8 = 0x25;
    pub const AMAX: u8 = 0x26;
    pub const VMAX: u8 = 0x27;
    pub const DMAX: u8 = 0x28;
    pub const D1: u8 = 0x2A;
    pub const VSTOP: u8 = 0x2B;
    pub const XTARGET: u8 = 0x2D;
    pub const SW_MODE: u8 = 0x34;
    pub const RAMP_STAT: u8 = 0x35;
    pub const ENCMODE: u8 = 0x38;
    pub const X_ENC: u8 = 0x39;
    pub const ENC_CONST: u8 = 0x3A;
    pub const ENC_STATUS: u8 = 0x3B;
    pub const ENC_DEVIATION: u8 = 0x3D;
    pub const CHOPCONF: u8 = 0x6C;
    pub const COOLCONF: u8 = 0x6D;
    pub const DRV_STATUS: u8 = 0x6F;
    pub const PWMCONF: u8 = 0x70;
}

/// RAMPMODE values.
pub mod rampmode {
    pub const POSITION: u32 = 0;
    pub const VELOCITY_POS: u32 = 1;
    pub const VELOCITY_NEG: u32 = 2;
    pub const HOLD: u32 = 3;
}

/// 40-bit SPI datagram: address byte (bit 7 = write) + 32 data bits, MSB first.
pub fn write_datagram(addr: u8, value: u32) -> [u8; 5] {
    let v = value.to_be_bytes();
    [addr | 0x80, v[0], v[1], v[2], v[3]]
}

pub fn read_datagram(addr: u8) -> [u8; 5] {
    [addr & 0x7F, 0, 0, 0, 0]
}

/// Data part of a reply (the reply to datagram N carries the read requested by datagram N-1).
pub fn reply_value(rx: &[u8; 5]) -> u32 {
    u32::from_be_bytes([rx[1], rx[2], rx[3], rx[4]])
}

/// SPI_STATUS byte flags (first byte of every reply).
pub mod spi_status {
    pub const RESET_FLAG: u8 = 1 << 0;
    pub const DRIVER_ERROR: u8 = 1 << 1;
    pub const STALLGUARD: u8 = 1 << 2;
    pub const STANDSTILL: u8 = 1 << 3;
    pub const VELOCITY_REACHED: u8 = 1 << 4;
    pub const POSITION_REACHED: u8 = 1 << 5;
}

/// Velocity in microsteps/s -> VMAX register units: v = VMAX * fCLK / 2^24.
pub fn vmax_from_usteps_s(v: f32) -> u32 {
    let x = v * 16_777_216.0 / FCLK_HZ;
    (x.max(0.0) + 0.5).min(8_388_096.0) as u32 // VMAX is 23 bits (max 2^23 - 512)
}

/// Acceleration in microsteps/s² -> AMAX units: a = AMAX * fCLK² / (512 * 256) / 2^24.
pub fn amax_from_usteps_s2(a: f32) -> u32 {
    let x = a * 512.0 * 256.0 * 16_777_216.0 / (FCLK_HZ * FCLK_HZ);
    (x.max(1.0) + 0.5).min(65_535.0) as u32
}

/// RMS current -> (GLOBALSCALER, CS). I_rms = GS/256 * (CS+1)/32 * VFS/RSENSE / sqrt(2).
/// CS is kept at 31 for best resolution; GLOBALSCALER carries the scaling.
/// GLOBALSCALER 0 means 256 (full scale); values 1..31 are not allowed by the chip.
pub fn current_to_scaler(i_rms_ma: u32) -> (u32, u32) {
    let full_scale_ma = VFS / RSENSE_OHM / core::f32::consts::SQRT_2 * 1000.0; // ≈ 4596 mA at GS=256, CS=31
    let gs = (i_rms_ma as f32 / full_scale_ma * 256.0 + 0.5) as u32;
    let gs = gs.clamp(32, 256);
    (if gs == 256 { 0 } else { gs }, 31)
}

/// CS value (0..31) for a second current (e.g. hold) at the GLOBALSCALER chosen for run current.
pub fn cs_for(i_ma: u32, run_ma: u32) -> u32 {
    if run_ma == 0 {
        return 0;
    }
    let cs = (i_ma as f32 / run_ma as f32 * 32.0 + 0.5) as i32 - 1;
    cs.clamp(0, 31) as u32
}

/// IHOLD_IRUN register: IHOLD [4:0], IRUN [12:8], IHOLDDELAY [19:16].
pub fn ihold_irun(ihold: u32, irun: u32, delay: u32) -> u32 {
    (ihold & 31) | ((irun & 31) << 8) | ((delay & 15) << 16)
}

/// ENC_CONST in decimal mode (ENCMODE.enc_sel_decimal = 1): signed integer part in [31:16],
/// fractional part in [15:0] as 0..9999 (factor = int + frac/10000). A negative factor reverses
/// the encoder direction: e.g. -21.3333 = int -22, frac 6667.
pub fn enc_const_decimal(usteps_per_count: f32) -> u32 {
    let mut int = usteps_per_count as i32;
    if (int as f32) > usteps_per_count {
        int -= 1; // floor for negative values
    }
    let mut frac = ((usteps_per_count - int as f32) * 10_000.0 + 0.5) as u32;
    if frac >= 10_000 {
        int += 1;
        frac = 0;
    }
    ((int as i16 as u16 as u32) << 16) | frac
}

/// SW_MODE: enable both reference-switch stops, active HIGH (pol_stop_l/r = 0).
/// The limit switches are NC to GND with pull-ups, so the inputs go high when hit or broken.
pub const SW_MODE_LIMITS: u32 = 0b0011;

/// Recommended static configuration. Chopper values are the datasheet's quick-config defaults
/// for a 36 V, 3-4 A NEMA 23; tune on the bench.
pub fn init_sequence(run_ma: u32, hold_ma: u32, usteps_per_count: f32, motor_reverse: bool) -> [(u8, u32); 14] {
    let (gs, irun) = current_to_scaler(run_ma);
    let ihold = cs_for(hold_ma, run_ma);
    [
        // GCONF: en_pwm_mode=0 (spreadCycle for torque), multistep_filt=1; diag0_int_pushpull=0 (open drain),
        // diag0_error=1, diag0_otpw=1 -> DIAG0 interrupt on driver error / overtemp warning.
        // shaft (bit 4) reverses the motor without rewiring
        (reg::GCONF, (1 << 3) | (1 << 5) | (1 << 6) | ((motor_reverse as u32) << 4)),
        (reg::GLOBALSCALER, gs),
        (reg::IHOLD_IRUN, ihold_irun(ihold, irun, 6)),
        (reg::TPOWERDOWN, 10),
        // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, MRES=0 (256 usteps), intpol=1
        (reg::CHOPCONF, 3 | (4 << 4) | (1 << 7) | (2 << 15) | (1 << 28)),
        (reg::PWMCONF, 0xC40C_001E),
        (reg::VSTART, 0),
        (reg::VSTOP, 10),
        (reg::A1, 0),
        (reg::V1, 0), // V1 = 0: single-stage AMAX/DMAX ramp
        (reg::D1, 0x100), // must not be 0 in positioning mode, even with V1 = 0 (datasheet §14)
        // hard stop at a limit (en_softstop = 0); see SW_MODE_LIMITS for the polarity
        (reg::SW_MODE, SW_MODE_LIMITS),
        // ENCMODE: enc_sel_decimal = 1
        (reg::ENCMODE, 1 << 10),
        (reg::ENC_CONST, enc_const_decimal(usteps_per_count)),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn datagrams() {
        assert_eq!(write_datagram(reg::VMAX, 0x0001_2345), [0xA7, 0x00, 0x01, 0x23, 0x45]);
        assert_eq!(read_datagram(reg::XACTUAL), [0x21, 0, 0, 0, 0]);
        assert_eq!(reply_value(&[0x08, 0xDE, 0xAD, 0xBE, 0xEF]), 0xDEAD_BEEF);
    }

    #[test]
    fn vmax_scaling() {
        // 12 MHz: 1 VMAX unit = 0.715 usteps/s; 51200 usteps/s (1 rev/s) -> 71583
        let v = vmax_from_usteps_s(51_200.0);
        assert!((71_570..=71_600).contains(&v), "{v}");
    }

    #[test]
    fn amax_scaling_nonzero() {
        assert!(amax_from_usteps_s2(1000.0) >= 1);
        let a = amax_from_usteps_s2(2_560_000.0); // 50 mm/s² at 51200 usteps/mm
        assert!(a > 100 && a < 65_535, "{a}");
    }

    #[test]
    fn current_scaling() {
        let (gs, cs) = current_to_scaler(3000);
        assert_eq!(cs, 31);
        // 3000/4596*256 ≈ 167
        assert!((165..=169).contains(&gs), "{gs}");
        assert_eq!(current_to_scaler(10_000).0, 0); // clamps to full scale (0 = 256)
        assert_eq!(current_to_scaler(100).0, 32); // clamps to the lowest legal scaler
        assert_eq!(cs_for(1000, 3000), 10);
    }

    #[test]
    fn encoder_constant() {
        // 51200 usteps / 2400 counts = 21.3333
        assert_eq!(enc_const_decimal(21.3333), (21 << 16) | 3333);
        assert_eq!(enc_const_decimal(2.0), 2 << 16);
        // reversed encoder: -21.3333 = -22 + 0.6667
        assert_eq!(enc_const_decimal(-21.3333), (0xFFEA << 16) | 6667);
    }

    #[test]
    fn limit_switches_stop_on_high() {
        // datasheet SW_MODE: bit0/1 enable the stops; bit2/3 = pol_stop_l/r, 0 = high level stops.
        // Our limits read HIGH when hit (NC to GND, pulled up), so polarity bits must be 0.
        assert_eq!(SW_MODE_LIMITS & 0b0011, 0b0011, "both stops enabled");
        assert_eq!(SW_MODE_LIMITS & 0b1100, 0, "high-active polarity");
        let seq = init_sequence(3000, 1000, 21.33, false);
        assert!(seq.contains(&(reg::SW_MODE, SW_MODE_LIMITS)));
        let gconf = |rev| init_sequence(3000, 1000, 21.33, rev).iter().find(|(a, _)| *a == reg::GCONF).unwrap().1;
        assert_eq!(gconf(true) ^ gconf(false), 1 << 4, "motor_reverse only flips GCONF.shaft");
    }
}
