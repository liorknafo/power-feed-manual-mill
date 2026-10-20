# The mill: SIEG X3.5D (Bar-Gal SKU SIEG-X3.5D)

Researched 2026-10-20. Sources are listed per fact. "Unverified" means the information comes from a forum, not a manual.

## Known
| Item | Value | Source |
|---|---|---|
| Model | SIEG X3.5D = SX3.5 family with a 3-axis DRO ("D"). It has **no** factory power feed; that is the "P" option. | SIEG SX3.5 manual p.15 (LittleMachineShop "6750 Large Bench Mill Users Guide" PDF); siegind.com SX3.5 page |
| Table | 740 × 180 mm. A different, larger casting than the X3/SX3, so X3 power-feed kits don't fit. | siegind.com |
| Travel X / Y / Z | 500 / 210 / 250 mm | Bar-Gal, siegind.com |
| Weight | 145 kg net | Bar-Gal, siegind.com |
| X leadscrew bearings | 6001 deep-groove ball bearing (12×28×8), 2 per end | SX3.5 manual, drawing 4A, item 25 |
| Handwheel | Right end only. Spring-loaded; you press it in to engage. Retained by an M12 hex nut and washer (the journal is probably threaded M12, inferred). | SX3.5 manual 4A-18…24; Grizzly G0935 manual p.27 |
| Left end | Bracket cover + end cap over the leadscrew end. The factory SX3.5P power feeder mounts **here**: an eccentric-shaft clutch and a coupling into the leadscrew end. | SX3.5 manual p.16, 6A |
| Factory feeder (reference) | 30 W DC motor, 0–240 rpm at the screw. Limit blocks run in the front T-slot. | Grizzly G0935 manual p.7–8 |
| DRO X scale | Magnetic, along the table front, with the reader under the table edge on the saddle. Not at either table end. | SX3.5 manual p.15, 37 |

## Unknown — please measure on the machine
1. **X leadscrew pitch:** mm of table travel per handwheel turn. Use a dial indicator or the DRO over 10 turns. The inch G0935 is 0.1 in per rev; the metric version is probably 2 or 2.5 mm, but that is unverified.
2. **Left end:** remove the end cap and the left bracket cover, then photograph with a ruler in frame:
   - the shape of the leadscrew end (slot? flat?), with slot width and depth;
   - the diameter of the hole in the left end plate;
   - the bolt pattern of the left bracket (M6 screws per the parts list).
3. **Space around the left end:** clearance to the column/saddle at full right travel, and the height from the table top to the leadscrew centre.
4. Confirm with Bar-Gal, or by looking, that the unit is "D" only, with nothing already mounted on the left end.

## Consequences for the design
- **Mount at the left table end**, where SIEG's own feeder goes. The right-end handwheel stays fully usable, and with the clutch released the screw turns freely.
- **Load:** the factory feeder is only about 30 W / ~1.3 N·m at the screw. Our NEMA 23 (3 N·m) through a 2:1 belt has much more margin.
- **Speed:** with a 2:1 belt, rapid at 600 mm/min needs the screw at 240–300 rpm (2.5–2 mm pitch), so the motor runs at 480–600 rpm. That is fine on 36 V.
- **Coupling:** the coupling to the leadscrew end can only be designed after measurement 2.
