# Osprey Rudder — Engineering Constraints

`[T]` thesis · `[SIM]` simulation · `[TEAM]` stated by the team · **bold** = unverified

![Installed: transom mount, rudder block atop the blade for the cable actuators. Thesis Fig 5.6.|right](figures/rudder_mount_installed_photo.jpg)

**Existing rudder.** MHZ Mystic C5000, 150 × 30 mm blade, about **3 in (76 mm)
submerged** on plane (thesis assumed 50%; team agrees; unmeasured). Mount is as
low as the transom allows — the previous team had *"concerns that the rudder we
had was not long enough to get enough engagement with the water"* `[T §3.4.2]`.
AGFRC A81FHM HV servo, **74 kg·cm stall at 8.4 V**, pull-pull cables `[T §3.4.1]`.

## Constraints

| # | Constraint | Reason |
|---|---|---|
| R1 | **Size on span, not area** | Authority is `A_r·CL_α`. Wider chord collapses aspect ratio: 30× the area at fixed span buys 32%; more span buys 5× for a third of the area. `[SIM]` |
| R2 | **Chord stays 30 mm** | Follows from R1. |
| R3 | **Useful deflection ±10°** — limit the autopilot here, not at the stop | Blade ventilates at ~10°; moment then *falls* (64 → 39 N·m at 30 mph) and stays down until ~6°. `[SIM]` |
| R4 | **Mechanical stop ±35° — assumed**, not in thesis | Set by horn and mount tabs. Measure with a protractor. |
| R5 | **Stock at 20–25% chord aft of LE** | Hinge moment is `F·(x_cp − x_stock)·c`. Near zero at the CoP; at the LE the lever is 7.5 mm and the servo is overloaded at speed (below). |
| R6 | **Mount stays; blade extends downward** | Mount cannot go below the transom bottom `[T §3.4.2]`. Block and cable geometry sit on top and are unchanged. |
| R7 | **Same thin section, no end plate** | A fat section ventilates earlier; an end plate does nothing once ventilated. |
| R8 | **Model valid to 44 mph only** | Cavitation number < 0.5 above 19.8 m/s. Numbers past that are extrapolated. `[SIM]` |

## Spec — 3 in + 2 in `[TEAM]`

| | current | **new** | | current | **new** |
|---|---|---|---|---|---|
| Submerged span | 3.0 in / 76 mm | **5.0 in / 127 mm** | Total blade, same mount | 5.9 in | **~8 in** |
| Chord | 30 mm | **30 mm** | Stock position | **unknown** | **20–25% c** |
| Wetted area | 3.5 in² | **5.9 in²** | Authority `A_r·CL_α` | 1.0× | **2.2×** |
| Aspect ratio | 2.5 | **4.2** | Turn radius, full useful | ~20 m (9.5 LOA) | **~11.5 m (5.4 LOA)** |

The course needs < 2° of steady rudder; the gain is margin. Across the parameter
uncertainty the current blade sits at its limit 41% of a lap; this one does not
saturate `[SIM]`.

## Servo check — 74 kg·cm stall

Peak hinge moment is at ventilation onset (~10°), not full deflection.

| speed | stock at LE | 15% c | **20–25% c** |
|---|---|---|---|
| 30 mph | 20 kg·cm (27%) | 8 (11%) | ~0 |
| 45 mph | 45 (60%) | 18 (24%) | ~0 |
| 70 mph † | **109 (148%)** | 44 (59%) | ~0 |

† past R8, ±30%. **Sufficient with a balanced stock; overloaded with a
leading-edge stock above ~55 mph.** The thesis's 66 kg·cm used the servo horn as
the lever; the hinge lever is CoP-to-stock, giving 45 kg·cm worst case at their
80 mph condition `[T §3.4.1]` `[SIM]`.

**Measure before building:** submerged depth on plane (one photo at 30 mph) ·
current stock position · tiller arm radius · mechanical stop angle · trailer and
ramp draft with 2 in more blade.
