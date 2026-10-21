# Power feed for a manual mill — electronics design

Status: draft. The mechanical design is deferred until the mill model is known.

## 1. Requirements

| # | Requirement |
|---|---|
| R1 | X-axis power feed, driven by a NEMA 23 stepper (STEPPERONLINE 3 N·m, 4.2 A/phase, 0.9 Ω, 3.8 mH, single 10 mm shaft, 4-wire). |
| R2 | Speed set with a rotary encoder dial. |
| R3 | Small OLED shows the current speed. |
| R4 | Electromagnetic tooth (face-gear) clutch between the motor and the feed screw, built in-house. |
| R5 | 3-position switch: center = stop and clutch released (handwheel free). Left/right = clutch engaged, feed in that direction. |
| R6 | Rapid-traverse push-button. |
| R7 | Overload protection ("resistance limit"): cap the motor torque, and stop the feed when the table jams or meets too much resistance. |
| R8 | Powered from AC mains through a built-in 36 V supply (Mean Well LRS-75-36, 2.1 A) in an earthed aluminium mains tray inside the housing, behind a fused, switched IEC C14 inlet. (The S-350-36 was the original plan; it doesn't fit inside.) |
| R9 | Custom PCB carrying the controller, stepper driver and clutch driver. Mains AC never reaches the custom PCB: all 230 V stays inside the earthed aluminium mains tray, and only the 36 V pair leaves it. |
| R10 | E-stop and end-of-travel limit switches. |

## 2. Architecture

```
 AC mains ─► [IEC inlet + fuse + switch] ─► 36 V PSU (LRS-75-36), both in the earthed mains tray
                                               │ 36 V DC
 ┌─────────────────────────────── custom PCB ──┴──────────────────────────────┐
 │  input fuse, reverse-polarity protection, TVS, bulk caps                   │
 │     ├─► TMC5160 + 4× MOSFET half-bridges ─────────────────────► NEMA 23    │
 │     ├─► clutch driver (P+N half-bridge, PWM current control, fast decay) ─► EM clutch
 │     └─► 36 V → 5 V buck ─► 3.3 V LDO ─► RP2040 (bare chip on the PCB)      │
 │                                                                            │
 │  MCU  ◄─SPI─► TMC5160      TMC5160 ◄─ clutch-side encoder (A/B/N)          │
 │  MCU  ◄─ speed dial encoder, 3-pos switch, rapid button, E-stop sense      │
 │  MCU  ─► OLED (I²C)        TMC5160 ◄─ limit switches (REFL/REFR)           │
 │  E-stop chain ─► hardware disable of the driver (DRV_ENN) and the clutch gate │
 └────────────────────────────────────────────────────────────────────────────┘
```

### Why the TMC5160
- Motor current is set over SPI, so the torque cap (R7) is a firmware setting. Feed and rapid can use different caps.
- It has its own ramp generator. The MCU only sends a target velocity, and acceleration and deceleration happen in the chip.
- It has an encoder interface with deviation detection. With the encoder on the *output side of the clutch*, a motor stall, a clutch cam-out (teeth ratcheting) or a table jam all show up as a mismatch between commanded and actual motion.
  - The encoder inputs only work in SPI motion-controller mode (SD_MODE=0); in step/dir mode they become dcStep pins.
  - On a mismatch the chip only sets `ENC_STATUS.deviation_warn` and pulses DIAG0. It does **not** stop by itself, so the MCU's DIAG0 interrupt handler must stop the motor and release the clutch.
- StallGuard2 gives a sensorless load reading as a second protection layer at medium and higher speeds. It is unreliable at very low speed, which is why the encoder is the primary detector.
- Its reference-switch inputs (REFL/REFR) let the limit switches stop the motor in hardware.
- External MOSFETs keep the heat off the controller chip at about 3.5 A RMS.

## 3. Operating behaviour (firmware state machine)

```
            power-up
               │
               ▼
        ┌─ INTERLOCK ◄─────────────── (switch not centre at boot / after fault)
        │   wait for switch = centre
        ▼
       IDLE  driver disabled, clutch released, handwheel free
        │ switch → L or R
        ▼
     ENGAGING  enable driver, clutch pull-in burst → hold current,
        │      jog motor slowly in the travel direction (≤ 1.5 tooth pitch)
        │      until the output encoder follows the motor (teeth meshed)
        │      → re-sync encoder, arm deviation detection
        │      (output never follows → FAULT "clutch did not engage")
        ▼
     RUNNING  velocity mode, ±v_feed (or v_rapid while button held)
        │      dial changes v_feed live; ramps handled by TMC5160
        │ switch → centre
        ▼
     STOPPING  ramp to 0, wait for standstill
        │
        ▼
     RELEASE  driver disabled first (unloads the teeth), then clutch off,
              wait t_release ─► IDLE

  Any state ──(E-stop, stall/deviation, StallGuard, driver fault,
               bus under/over-voltage)──► FAULT
  FAULT: immediate stop, clutch released, driver disabled, reason on OLED.
         Latched until the switch returns to centre, then ─► IDLE.
```

Rules:
- The clutch is engaged only at standstill (plus the slow engagement jog), and released only after the motor has stopped and the driver is disabled.
- If the switch returns to centre during ENGAGING, the jog stops and the system goes straight to RELEASE.
- Flipping straight from left to right always goes through STOPPING → RELEASE → IDLE → ENGAGING, so there is no instant reversal.
- A limit switch stops motion toward that limit only. Moving the other way is still allowed.
- If the switch is off-centre at power-up, the system waits in INTERLOCK and does not move.
- When a fault occurs, the clutch is released at once, so the table is mechanically free.

### Display / UI
- Main screen: the current feed rate in large digits, plus direction, state, and the rapid indicator.
- Dial: turning it changes the feed rate, with acceleration on fast spins. A short press toggles fine/coarse steps.
- A long press opens settings: motor current limit, clutch hold current (sets the cam-out torque), stall sensitivity, rapid speed, units, leadscrew pitch, gear ratio. Settings are saved to flash.
- Units: mm/min or in/min. Both need the leadscrew pitch and gear ratio, which are set once the mechanics are known. Until then the screen shows motor RPM.

## 4. RP2040 pin allocation (GPIO numbers as on a Pico, so a Pico works for breadboard bring-up)

This is the authoritative version; it matches `hardware/tools/build_schematic.py`.

| GPIO | Net | Notes |
|---|---|---|
| GP0  | ENC_OUT_A | output-side encoder A. It also goes to TMC5160 ENCA, so the MCU can cross-check with PIO |
| GP1  | ENC_OUT_B | output-side encoder B. It also goes to TMC5160 ENCB |
| GP2  | CLUTCH_HS_PWM | clutch high-side PWM (PWM1A); ANDed with E_OK in hardware; 100k pull-down |
| GP3  | E_OK_SENSE | E-stop loop status, read through 100k so firmware can never drive the interlock; high = loop closed / OK. Keep GP3 an input **with its internal pull disabled**. It shares PWM slice 1 with GP2. A pull-down would make it read ~1 V (a fault at every boot); a pull-up would hide an open E-stop from the firmware |
| GP4  | I2C_SDA | OLED (I²C0) |
| GP5  | I2C_SCL | OLED (I²C0) |
| GP6  | LIM_L | limit switch L. It also goes to TMC5160 REFL (hardware stop) |
| GP7  | LIM_R | limit switch R. It also goes to TMC5160 REFR |
| GP8  | CLUTCH_LS_EN | clutch low-side enable; ANDed with E_OK in hardware; 100k pull-down |
| GP9  | — | spare |
| GP10 | DIAL_A | speed dial A |
| GP11 | DIAL_B | speed dial B |
| GP12 | DIAL_SW | speed dial push |
| GP13 | MODE_L | 3-pos switch LEFT (active low) |
| GP14 | MODE_R | 3-pos switch RIGHT (active low); both active = wiring fault |
| GP15 | RAPID | rapid button (active low) |
| GP16 | SPI_MISO | ← TMC5160 SDO |
| GP17 | SPI_CS | → TMC5160 CSN (10k pull-up) |
| GP18 | SPI_SCK | → TMC5160 SCK |
| GP19 | SPI_MOSI | → TMC5160 SDI |
| GP20 | DRV_EN | **active high**; TMC_DRV_ENN = NAND(DRV_EN, E_OK); 100k pull-down |
| GP21 | TMC_DIAG0 | stall / deviation / fault interrupt (open drain, 10k pull-up) |
| GP22 | TMC_DIAG1 | open drain, 10k pull-up |
| GP26 | VM_SENSE | ADC0: 36 V bus through 100k/7.5k (full scale ≈ 47 V) |
| GP27 | CLUTCH_ISENSE | ADC1: clutch coil current, 0.1 Ω shunt × 21 (≈ 0.47 A per volt) |
| GP28 | — | spare (ADC2) |
| RUN  | RUN | reset button |

## 5. Bill of materials (draft)

| Block | Part | Status |
|---|---|---|
| Stepper driver | TMC5160A-TA (eTQFP-48 7×7), LCSC C516354 | to buy |
| Bridge MOSFETs | 4× dual N-channel, 60 V, Qg ≲ 15 nC (reference: JMSL0615AUD as used on BTT TMC5160T Pro, which is not stocked at LCSC) | to pick from the LCSC parametric search |
| Sense resistors | 2× 0.050 Ω, low-inductance (≈ 4.7 A RMS full scale, trimmed down with GLOBALSCALER) | to pick |
| 5 V buck | TI LM5164DDAR (6–100 V in, 1 A), LCSC C477928. Fallback: LMR51610XDBVR (4–65 V), C20539658 | to buy |
| Clutch driver | 60 V P-FET (high side) + 60 V logic-level N-FET (low side), 2× SS210 diodes, 0.1 Ω shunt + TLV9061, ≥60 V PTC (see §7.6) | to pick |
| Controller | RP2040 + W25Q128JV flash + 12 MHz crystal, on the PCB (a Pico is still handy for breadboard bring-up) | on the BOM |
| Dial | rotary encoder with push button (Cylewet CYT1100) | owned |
| Display | 1.3" 128×64 I²C OLED — confirm SH1106 vs SSD1306 with the seller. Power it from **3.3 V**, because its on-board I²C pull-ups go to its supply | to buy |
| Clutch-side encoder | LPD3806-600BM-G5-24C (600 PPR, NPN open-collector, 5–24 V). Power it from 5 V, with 2.2–4.7 kΩ pull-ups to 3.3 V | to buy |
| Clutch | DIY electromagnetic tooth clutch, coil designed for the 36 V bus (see §7) | to build |
| Motor | STEPPERONLINE NEMA 23, 3 N·m | owned |
| PSU | Mean Well LRS-75-36, 36 V / 2.1 A + snap-in IEC C14 inlet with fuse (T 2 A ceramic) and switch + IEC retaining clip; folded 1.5 mm aluminium mains tray + lid (make) | to buy / make |
| Fallback driver | DM556 | owned — used for breadboard firmware bring-up |
| Panel | ON-OFF-ON toggle, rapid push-button, mushroom E-stop (NC), 2× limit switches | to buy |

## 6. Plan

1. Finish this spec and verify the part numbers.
2. Firmware bring-up on a breadboard: Pico + DM556 + OLED + dial + switch. This proves the UI and state machine before the PCB exists.
3. KiCad schematic → review → 4-layer layout → order bare PCBs + stencil. The user reflows the boards.
4. Port the firmware from the DM556 (step/dir) to the TMC5160 (SPI velocity mode), then tune the current cap and stall detection.
5. Mechanical design: the mill is a SIEG X3.5D (docs/mill.md). The feed unit (`mechanical/feed_unit`) mounts on the left table end: mount plate, 2:1 HTD-5M belt, leadscrew stub and encoder. Its mill-side dimensions are placeholders until measured.

## 7. Clutch — DIY electromagnetic tooth (face-gear) clutch

The user builds the clutch in their own shop (CNC, manual lathe, manual mill), so we design the teeth, coil and magnetic circuit ourselves. A tooth clutch was chosen over a friction clutch because:
- way oil and coolant don't affect it;
- it practically doesn't wear;
- it needs far less clamp force;
- it can only engage at standstill, which the state machine already guarantees.

### 7.1 How it works

```
             stationary field housing (bolted to the feed-unit frame)
             ┌────────────────┐
             │   ┌────────┐   │   ← coil (~850 turns), does not rotate,
             │   │  coil  │   │     so it needs no slip rings
             │   └────────┘   │
             └─┐            ┌─┘   ← ~0.3 mm radial air gaps to the rotor
 motor  ═══════╡   ROTOR    ╞═══  inner pole │ non-magnetic ring │ outer pole │ tooth ring
 shaft         └────────────┘                                                  /\/\/\/\
                 ↕ 1.1 mm stroke: teeth fully out when the coil is off
               ┌────────────┐
               │  ARMATURE  │   ← low-carbon steel, flat pole faces + mating tooth ring \/\/\/\/
               └──╥──────╥──┘     3 drive pins pressed into it
               ═══╬══════╬═══  output hub: the pins slide in bronze bushes in it ─► pulley ─► reduction ─► leadscrew
                 ⌇⌇      ⌇⌇    return springs on the pins, behind the hub: pull the armature back onto the hub
```

- **Coil off:** the three return springs pull the armature back against the hub face (a hard stop), so the teeth are fully out of mesh: the tips are 0.4 mm apart. The handwheel turns freely.
- **Fail-safe — no power means disengaged:** only the coil holds the clutch engaged; the springs always disengage it.
  - On power loss, E-stop, a firmware crash or an MCU reset, the gate drive drops: the 5 V AND gates go low and the MCU outputs have pull-downs.
  - The coil current then dies through the fast-decay diodes, and the springs lift the teeth out.
  - The springs are preloaded: ≈ 24 N even at the released stop, ≈ 32 N engaged (§7.4), so residual magnetism can't hold the armature engaged.
- **Coil on:** the flux runs inner pole → armature → outer pole and pulls the teeth into mesh. Torque passes through the tooth flanks.
- **Angled flanks:** the coil force presses the flanks together, so there's **zero backlash**.
- **Cam-out:** above a set torque the flanks push the armature back against the coil force and the teeth ratchet. That's the mechanical backup to the electronic torque cap (R7). The output encoder sees it at once and triggers a FAULT.
- **Pole gap when engaged:** the flat pole faces keep a designed gap of about 0.15 mm, so they never touch. All axial load goes through the tooth flanks, which keeps the cam-out torque well defined.

### 7.2 Placement
The clutch goes **on the motor side, before the belt reduction**, so it only has to carry motor torque (~3 N·m max). The output encoder sits on the **leadscrew stub, after the belt** (`mechanical/feed_unit`). It sees motor stall, cam-out, belt slip and table jams alike, and it also confirms engagement (§7.5). Firmware scales its counts by the belt ratio.

### 7.3 Tooth geometry (first pass)

| Quantity | Value | Notes |
|---|---|---|
| Tooth form | trapezoidal face teeth, flank angle α = 30° from the shaft axis | gives self-release (tan α ≫ μ) and a cam-out torque that is not very sensitive to oil |
| Tooth ring | mean radius ≈ 35 mm, outside the outer pole (ring ≈ 64–76 mm Ø) | larger radius → less force per N·m |
| Number of teeth | 30, pitch ≈ 7.3 mm | engagement jog ≤ 1 pitch = 12° at the motor ≈ 7 full steps |
| Tooth depth | 0.9 mm flank-to-flank, tips truncated 0.1 mm on both rings, so 0.8 mm working height | 0.1 mm tip/root clearance: the flanks carry the load and the tips never bottom out |
| Pole gap | 0.15 mm engaged | the armature's tooth roots sit 0.25 mm behind its pole face |
| Stroke | 1.1 mm (0.8 working height + 0.3 tip clearance when released) | released pole gap 1.25 mm |
| Material | same low-carbon steel as the armature/rotor; case-harden the tooth rings if wear shows up | |

### 7.4 First-pass sizing (estimate — verify on the bench)

| Quantity | Value | Basis |
|---|---|---|
| Target cam-out torque | ≈ 4.5 N·m | 1.5× the motor's 3 N·m holding torque. It only trips on inertial shocks or faults |
| Tangential tooth force at cam-out | 4.5 / 0.035 ≈ 130 N | |
| Axial push-apart force | F_t·(tan α − μ)/(1 + μ·tan α) ≈ 55 N | μ ≈ 0.1–0.15 steel on steel, so ±10% spread |
| Spring force | ≈ 24 N released (preload) → ≈ 32 N engaged | 3 compression springs on the drive pins, ≈ 2.5 N/mm each, 8.0 mm long released, 6.9 engaged. Preloaded, so the armature sits hard on its stop when off and ≥ 24 N always works against remanence |
| Hold force needed | ≈ 90 N | 55 + 32 |
| Pole-face area | 2 × 5 cm² (inner ring Ø16–30 mm, outer ring Ø54–60 mm) | |
| Hold MMF (0.15 mm engaged gap, ×2) | ≈ 170 A·turns → ≈ 0.2 A | **the hold current sets the cam-out torque**, so it is tunable in firmware |
| Pull-in at the 1.25 mm released gap (×2) | 1.5 A × 850 = 1.3 kA·turns → ≈ 70 N (with ×1.5 for leakage) | against the 24 N preload: ≈ 3× margin; the force rises steeply as the gap closes, faster than the springs' 32 N |
| Coil | ≈ 850 turns of 0.32 mm (AWG 28) enamelled copper, ≈ 24 Ω | winding window 10 mm radial × 12 mm axial, fill factor 0.6 |
| Pull-in burst | 36 V full-on → ≈ 1.5 A, ≈ 54 W, for ~50 ms (≈ 3 J) | short, so thermally harmless |
| Hold | ≈ 0.2–0.4 A, about 1–4 W | calibrated against the cam-out torque |
| Overall size | ≈ 80 mm OD | |

**Bench calibration (before final assembly):**
1. Wind the coil, then measure pull force against current at full stroke and at the engaged gap, using a spring or luggage scale.
2. Assemble the teeth and measure cam-out torque against hold current with a torque wrench.
3. Store the resulting table in the firmware settings.

### 7.5 Engagement and release (firmware)
- **Tooth-on-tooth** is normal. The armature then rests on the tooth tips, and the jog lines the teeth up so they snap in.
- **Engagement jog:**
  1. With the coil energised, move the motor slowly in the travel direction, at most 1.5 tooth pitches.
  2. "Engaged" means the output encoder moves in step with the motor (times the ratio) for N counts.
  3. Encoder deviation detection is off during the jog. After engagement, re-sync `X_ENC` to `XACTUAL` and turn deviation detection back on.
  4. If the output never follows, it's a FAULT: "clutch did not engage".
- **Release:** disable the driver first, which takes the torque off the flanks, then turn off the coil. On a FAULT, both happen at once.

### 7.6 Electrical drive (on the PCB)
The driver is an asymmetric half-bridge: a high-side P-FET and a low-side N-FET, plus two freewheel diodes.
- **Engage / hold:**
  - The low side is on and the high side is PWM'd (GP2).
  - During the off-time the current recirculates through the low-side FET, the shunt and a diode (slow decay). The shunt therefore sees the coil current all the time, and ADC1 reads the average directly.
  - Firmware closes a current loop on that reading.
- **Sequence:** a full-on pull-in burst (≈ 1.5 A, ~50 ms), then the calibrated hold current.
- **Release:** both FETs off. The coil sees −36 V through the diodes and its energy goes back to the bus (fast decay). No zener clamp is needed, and no FET sees more than about 37 V.
- **Hardware safety:**
  - Both gate signals are ANDed with E_OK.
  - A 60 V PTC (0.5 A hold / 1 A trip) in the clutch supply opens if firmware ever leaves the coil fully on (≈1.5 A). It passes the 50 ms pull-in burst and the ≤0.4 A hold.
- **PWM frequency ≤ 2 kHz:**
  - The high-side P-FET is switched by a simple resistive level shifter (2.2k/4.7k, 12 V zener clamp), so its edges take a few µs.
  - The coil's L/R time constant is milliseconds, so 1–2 kHz gives negligible ripple and negligible switching loss.
  - Faster PWM needs an active gate driver.

### 7.7 Build notes
- **Magnetic parts:** rotor, field housing and armature in low-carbon steel (1018/1020 or 12L14). Anneal them if possible. Avoid 300-series stainless, which is non-magnetic.
- **Pole separation:** a non-magnetic ring between the inner and outer pole on the rotor (brass, bronze or aluminium, or machined slots). It forces the flux through the armature.
- **Mating tooth rings:** cut them as a pair with the same setup (same cutter and indexing) so all 30 teeth bear evenly. A CNC with a V/chamfer cutter or the mill with an indexer both work.
- **What holds the armature:** three Ø6 drive pins pressed into it (Loctite 638) slide in sintered-bronze bushes in the hub flange, with 0.01–0.03 clearance. They do everything a solid connection would:
  - **rotation:** the armature can't turn without the hub, it drives the hub through the pins;
  - **centring and squareness:** three pins at 120° in 8 mm long bushes; when engaged the 30° teeth centre it the last hundredths;
  - **axially:** three compression springs on the pins, behind the hub (each held by a washer and an E-clip), pull the armature back onto the hub's face when the coil is off: a hard, preloaded stop (≈ 24 N), not a balance of forces. The magnet pulls it 1.1 mm off that stop onto the rotor.
  The same layout as a spring-applied brake's armature on its guide pins.
- **Torque path:** rotor teeth → armature → 3 pins → bronze bushes → hub → set screws on the shaft flat → output shaft → 20T pulley. The most torque it can see is the cam-out torque, 4.5 N·m (the teeth ratchet above it). At r 21 that is ≈ 71 N per pin:
  - bush pressure 71 N / (6 × 8 mm) ≈ 1.5 MPa (sintered bronze takes ~10 MPa);
  - the pin bends over at most the 1.1 mm gap between armature and hub: negligible for Ø6 steel;
  - backlash: the 0.01–0.03 pin clearance at r 21 is ≤ 0.1° at the motor, ≤ 0.05° at the leadscrew; the leadscrew nut's own backlash is far larger.
- **Rotor fixing:** a round Ø10 H7 bore glued with Loctite 641 onto the motor's D-shaft. That holds >10 N·m against the motor's 3 N·m, avoids broaching a D-bore, and comes off with moderate heat (648 would need the whole motor heated). An M3 screw into the tapped shaft end pulls the rotor's shoulder onto the shaft end, which sets its axial position.
- **Model and machining data:** `mechanical/clutch/clutch.py` holds the parametric model; `mechanical/clutch/MACHINING.md` has the dimension tables, fits and tooth-cutting notes, generated from the same parameters.
- **Shimming:** the rotor's axial position comes from its bore depth (measured shaft length − 2.2 mm). The released gap (1.25) is set by where the hub sits, which the stepped spacer between the hub boss and the inner bearing sets: grind it shorter or shim it (MACHINING.md, step 6).

## 8. Firmware

- **Language:** Rust, on [Embassy](https://embassy.dev) (`embassy-rp`) for the RP2040. It provides async tasks, SPI, I²C, PWM, ADC, PIO and flash support.
- **Display:** `embedded-graphics` plus the SH1106 or SSD1306 driver crate, whichever matches the OLED we buy.
- **TMC5160:** a small register driver of our own over `embedded-hal` SPI. It includes the encoder-deviation handling for the clutch engagement jog (§7.5).
- **Settings:** stored in flash.
- **Structure:**
  - **`core` crate**, `no_std` and hardware-free: state machine, speed/unit math, fault logic. It's unit-tested on the PC with `cargo test`.
  - **`app` crate:** the Embassy tasks that tie `core` to the hardware.
  - **Motor back-ends:** a step/dir one (PIO) for the DM556 breadboard stage and an SPI one for the TMC5160 on the PCB.
- **Flashing / debugging:** `probe-rs` with a Raspberry Pi Debug Probe (or a Pico running debugprobe). Plain UF2 drag-and-drop also works.

## 8a. Safety notes

- **Mains (230 V) inside the unit:** the unit rides on the table, so mains is kept in one place and double-fenced.
  - All 230 V parts (IEC inlet with fuse and switch, the LRS-75-36, L/N/PE wiring) sit in an **earthed folded-aluminium
    tray** with a lid over the terminals. It is a metal fire enclosure; the printed PETG housing never touches mains, and
    the floor below the tray has no vents.
  - Only the 36 V pair leaves the tray (grommet in the lid). No low-voltage wire enters it; `check_feed_unit.py` checks
    that nothing else intrudes into the tray and that the wiring zone (≥ 35 mm) is empty.
  - The tray is bolted to the plate, which fixes it and bonds it to PE; the plate in turn earths the motor, clutch,
    encoder and the mill table. Earth leads longer than L/N. Inlet fuse in L, T 2 A ceramic.
  - Commissioning: earth-pin to plate / motor / clutch / encoder / PSU case ≤ 0.1 Ω before the first plug-in.
  - The rocker switch on the inlet is the mains switch. The E-stop removes drive enable and clutch, not mains.
- **E-stop scope:** the E-stop removes the driver enable (TMC5160 outputs float) and the clutch drive (the table becomes mechanically free). It does **not** disconnect the 36 V bus. A mains-side contactor would be a separate addition.
- **Bus overvoltage:** the input TVS (SMBJ36A) clamps transients at ≤58 V, against 60 V MOSFETs and a 60 V TMC5160 abs-max limit.
  - It cannot absorb sustained regeneration.
  - Firmware limits deceleration and watches VM_SENSE (ADC0); it faults above about 42 V.
  - Don't trim the PSU above 36 V.
- **Restart after E-stop:** the hardware interlock stops the driver and clutch while the E-stop is open, but it does not latch. When the E-stop is released, it is the firmware that prevents a restart:
  - FAULT drops DRV_EN and both clutch requests.
  - FAULT stays latched until the mode switch returns to centre (§3).
  - A hardware latch with a reset button is a possible future addition.
- **Clutch PTC:** its hold current derates with temperature, so keep the clutch hold current ≤0.3 A. When confirming the 2920L050/60 part, check its derating curve against the enclosure temperature.
- **Debugging:** SWD on a 1×3 0.1" header (SWCLK, GND, SWDIO; the Debug Probe's JST-to-0.1" cable plugs on).
- **Blank board:** hold BOOTSEL, tap RESET, and the RP2040 shows up as the `RPI-RP2` USB drive (USB-C on the connector edge).

## 9. Open questions
- Mill model: determines the leadscrew pitch, the clutch dimensions and the mounting.
- Target feed range and rapid speed in mm/min, which depend on the leadscrew.
