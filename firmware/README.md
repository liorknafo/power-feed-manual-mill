# Firmware (Rust, Raspberry Pi Pico / RP2040)

```
firmware/
  feed-core/   no_std logic, unit-tested on the PC: state machine, units, TMC5160 maths,
               clutch current control, dial decoder, button/menu, settings blob
  feed-app/    Embassy application for the Pico (hardware glue)
  tools/elf2uf2.py   ELF -> UF2 for drag-and-drop flashing
```

## Build, test, flash

```sh
# one-time
rustup target add thumbv6m-none-eabi

# logic tests on the PC (35 tests: every state-machine rule in docs/design.md §3)
cd firmware/feed-core && cargo test

# firmware
cd firmware/feed-app && cargo build --release
python3 ../tools/elf2uf2.py target/thumbv6m-none-eabi/release/feed-app ../power-feed.uf2
```

**Flashing options:**
- **UF2:** hold BOOTSEL on the Pico while plugging in USB, then copy `power-feed.uf2` onto the `RPI-RP2` drive.
- **probe-rs:** with a Raspberry Pi Debug Probe on the Pico's SWD pads, run `cargo run --release` (uses probe-rs, gives live `defmt` logs).

## Structure

### `control` (1 kHz, high-priority interrupt executor)
Each tick:
1. **Feed the watchdog** (100 ms).
2. **Read the ADCs:** the 36 V bus every 10 ms, the clutch current every tick.
3. **Read the panel:**
   - mode switch, active low;
   - rapid button;
   - E_OK, read through its 100 k tap with the internal pull disabled;
   - limit switches (NC: high = hit or broken);
   - speed dial and its push button.
4. **Read the TMC5160:** XACTUAL, X_ENC and ENC_STATUS. The SPI status byte gives the standstill and driver-error flags.
5. **Step the state machine:** `feed_core::Controller::step()`.
6. **Apply the outputs:**
   - DRV_EN, to the hardware NAND with E_OK;
   - the motion command, written to TMC5160 registers only when it changes;
   - the encoder re-sync and the stall-deviation limit;
   - clutch PI control: low-side enable plus high-side PWM at 2 kHz.

### `display` (thread mode, 10 Hz)
Draws the OLED from a status snapshot. The control loop pre-empts it.

### Safety layers, outermost first
1. **Hardware interlock:** the E-stop cuts DRV_ENN and both clutch gates, whatever the firmware does.
2. **Pull-downs:** the Pico's enable outputs have pull-downs, so a reset or crash means driver off and clutch off.
3. **Watchdog:** 100 ms. A hung loop resets the Pico.
4. **State-machine faults:**
   - E-stop;
   - TMC5160 driver error;
   - stall / encoder deviation;
   - bus over- and under-voltage;
   - switch wiring;
   - clutch failed to engage;
   - stop timeout.

   Every fault latches until the switch returns to centre.
5. **TMC5160 reference-switch inputs** stop the motor at a limit in hardware.

## User interface

**Main screen:** state, feed in mm/min (large), RAPID / LIMIT / fine-coarse, bus voltage.

**Dial:**
- turn: change the feed;
- short press: fine / coarse steps;
- **long press while stopped:** opens the settings menu.
  - turn: change the value;
  - press: next item;
  - long press: save to flash and exit.
  - Items: screw pitch, belt ratio, motor current, hold current, clutch hold current, rapid speed, max feed, acceleration.

## First power-up (bench, nothing attached to the mill)

1. **Power the board from USB only (no 36 V).**
   - The OLED should show `NO 36V`.
   - Check the logs with probe-rs if you have one. `TMC5160 not responding` is expected without VM.
2. **36 V on, motor connected, clutch disconnected.**
   - Switch to centre, and READY should appear.
   - Hold the motor shaft and flip to LEFT. The state goes ENGAGING, then `CLUTCH NO ENGAGE` after about 3 s, because there is no clutch encoder feedback yet.
   - That proves the interlock, the driver enable and the fault path.
3. **E-stop:** open the E-stop loop while in any state. The display shows `E-STOP`; the motor goes limp and the clutch drops.
4. **Measure, then set in the menu:**
   - the leadscrew pitch (docs/mill.md);
   - the belt ratio;
   - the clutch hold current. Calibrate it: hold current against cam-out torque, measured with a torque wrench (design.md §7.4).
5. **Direction:** if the table moves the wrong way, swap one motor coil pair (A+/A−).

## Defaults to check (`feed-core/src/settings.rs`)

| Setting | Default | Note |
|---|---|---|
| `leadscrew_pitch_mm` | 2.0 | **Placeholder.** Measure it. |
| belt ratio | 2:1 | |
| feed | 5–400 mm/min | |
| rapid | 600 mm/min | |
| run / hold current | 3.0 A / 1.0 A RMS | |
| clutch pull-in | 60 ms full-on | |
| clutch hold | 350 mA | |
| engagement jog | max 12 full steps (≈1.5 tooth pitches) | |
| stall tolerance | 8 full steps | |
| bus window | 30–42 V | |

## Not implemented yet
- **PIO cross-check:** a second, Pico-side quadrature count of the output encoder on GP0/GP1, comparing against the TMC5160's own deviation detection. The pins are wired; it's a firmware addition.
- **StallGuard2:** could be added as a third stall layer at higher speeds.
