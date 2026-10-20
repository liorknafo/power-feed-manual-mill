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

# logic tests on the PC (45 tests: every state-machine rule in docs/design.md §3)
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
   - TMC5160 driver error, or driver not configured (re-initialised automatically when 36 V appears or the chip resets);
   - stall / encoder deviation;
   - bus over- and under-voltage;
   - switch wiring;
   - clutch failed to engage (the output must follow the motor, not just move);
   - the limit *behind* the table tripping, which means the direction is reversed;
   - sensor fault (stuck ADC, open clutch coil);
   - stop timeout (derived from speed ÷ acceleration).

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
  - Items: screw pitch, belt ratio, motor current, hold current, clutch hold current, rapid speed, max feed, acceleration, motor reverse, encoder reverse.
  - Edits go to a working copy that's applied only on save. Leaving the menu any other way drops them.

## Commissioning (do these in order)

### 1. USB only, no 36 V
The OLED should show `NO 36V`. The TMC5160 is configured automatically once 36 V appears.

### 2. 36 V on, motor on the bench, clutch disconnected
- The state should reach READY with the switch centred. `DRIVER NOT READY` means the TMC5160 isn't answering: check SPI and VCC_IO.
- Flip the switch to LEFT. The coil stays off until the motor is at a standstill. The engagement jog then turns the motor about 12 full steps and faults with `CLUTCH NO ENGAGE`, because the encoder isn't connected yet.
- This proves the driver enable, the jog and the fault path.

### 3. E-stop
Open the E-stop loop in any state. The display shows `E-STOP`, the motor goes limp and the clutch drops. It stays that way until the loop is closed **and** the switch is centred.

### 4. Limit switches (still on the bench)
- Press each limit by hand while centred. The display should show `LIMIT L` or `LIMIT R` the next time you try that direction.
- The TMC5160 also hard-stops the motor at a limit. It is configured "high = hit", matching NC switches to GND.

### 5. Directions, before the unit goes on the mill
- Set the leadscrew pitch and belt ratio in the menu.
- Run RIGHT with the clutch connected and the belt off, and check which way the leadscrew stub turns.
  - If the table would move the wrong way, set **MOTOR REVERSE = 1**; don't rewire.
  - Then check that the encoder counts with the motor. If every engagement faults with the output clearly turning, set **ENCODER REVERSE = 1**.
- On the mill, a limit tripping *behind* the direction of travel gives `LIMIT WRONG WAY`. That means one of these two settings, or the limit wiring, is reversed.

### 6. Clutch hold current
Calibrate hold current against cam-out torque with a torque wrench (design.md §7.4).
- Keep it at 300 mA or less: the PTC fuse holds 0.5 A and derates when warm. The menu caps it at 400 mA.
- If you need more holding torque, fit a larger PTC first.

## Defaults to check (`feed-core/src/settings.rs`)

| Setting | Default | Note |
|---|---|---|
| `leadscrew_pitch_mm` | 2.0 | **Placeholder.** Measure it. |
| belt ratio | 2:1 | |
| feed | 5–400 mm/min | |
| rapid | 600 mm/min | |
| run / hold current | 3.0 A / 1.0 A RMS | |
| clutch pull-in | 60 ms full-on | |
| clutch hold | 250 mA | ≤300 mA recommended (PTC 0.5 A hold); menu cap 400 mA |
| engagement jog | max 12 full steps (≈1.5 tooth pitches) | engaged = output follows the motor within 0.6–1.6× over ≥8 counts |
| encoder | 2400 counts per **leadscrew** turn | the encoder sits on the leadscrew stub, after the belt |
| stall tolerance | 8 full steps | |
| bus window | 30–42 V | |

## Not implemented yet
- **PIO cross-check:** a second, Pico-side quadrature count of the output encoder on GP0/GP1, comparing against the TMC5160's own deviation detection. The pins are wired; it's a firmware addition.
- **StallGuard2:** could be added as a third stall layer at higher speeds.
