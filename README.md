# Osprey USV — Rudder Sizing & Heading Control Simulation

<https://github.com/GrahamSmith2/Osprey>

Python simulation supporting rudder sizing, servo specification, and the
autopilot for **Osprey**, a 7 ft twin-motor electric planing catamaran USV that
runs a 2-mile autonomous course at the ASNE/ONR PEP Autonomy division. The
autopilot is a **Lua script** written against ArduPilot's scripting API, flown
here against the simulated boat.

**Prior work.** The vessel was designed and built by **Aidan Astudillo and Sean
Lee** (Princeton MAE senior thesis, April 2026). Their thesis is included at
[`docs/Astudillo_Lee_2026_Osprey_thesis.pdf`](docs/Astudillo_Lee_2026_Osprey_thesis.pdf)
— © the authors, all rights reserved, reproduced with attribution for project
continuity. It is the authoritative source for the as-built hardware; everything
in `python/` and `results/` is separate work built on top of it, and
`docs/WORK_REMAINING.md` tags which is which.

```bash
cd python
```

```bash
pip install -r requirements.txt
```

```bash
python -m osprey run controllers/oval_autopilot.lua
```

```bash
python -m osprey report
```

`python/README.md` has every command and option; `python/LUA_API.md` is the
scripting reference.

---

## THE CENTRAL CAVEAT — READ THIS BEFORE USING ANY NUMBER

At 30 mph this hull runs at Froude number **Fn ≈ 2.9**; at the 80 mph design
speed, **Fn ≈ 7.8**.

Every standard manoeuvring-derivative regression — Clarke, Norrbin, Inoue, and
the rest of the ship-hydrodynamics literature — is fitted to **slender
displacement hulls at Fn < 0.3**. This boat operates one to one-and-a-half
orders of magnitude outside that domain, on a planing catamaran with a
ram-air tunnel, which is not the hull form those regressions describe either.

**Any hull derivative in this repo taken from those correlations is a
placeholder with order-of-magnitude uncertainty, not an estimate.** They are
swept ±5× in `UNCERTAINTY` in `python/osprey/params.py` for exactly this reason.

### What this simulation therefore cannot do

It **cannot tell you what your gains should be.** Any specific gain it prints is
conditional on hull derivatives that are unknown to within 5×. It is also
**3-DOF** (surge, sway, yaw): it cannot see roll, hooking or blow-over, and a
quasi-static check stands in for them.

### What it can do, and is built to do

1. **A rudder that works across the whole plausible parameter space**, sized on
   span rather than area, with drag, hinge moment and ventilation as limits.
2. **A servo specification with margin**, from a correct hinge-moment
   formulation rather than force × horn radius.
3. **The structure of the gain schedule and its scaling law**: Nomoto `K ∝ U`,
   `T ∝ 1/U`, so an angle-domain rate gain goes as `1/U²` and a moment-domain
   one is speed-invariant. That survives not knowing `K′` and `T′`.
4. **A test bench for the real autopilot script**, including faults, GPS loss
   and the PEP Rule 21 kill.
5. **Which experiment to run** to collapse the remaining uncertainty.

Every deliverable is designed to survive being wrong by 5× on the hull
derivatives. If a conclusion in `results/summary.md` does not survive that, it
is labelled as not surviving it.

---

## Second caveat: the cavitation ceiling

The rudder cavitation number is

```
sigma = (p_atm + rho*g*h - p_v) / (0.5*rho*U^2)
```

For this blade at mid-span depth, **sigma drops below 0.5 at U ≈ 19.9 m/s
(44 mph)** and reaches **0.154 at the 35.8 m/s design speed**.

**The attached-flow lift model in `python/osprey/rudder.py` is not valid above
about 20 m/s.** The planned race cruise, 50 mph (22.4 m/s), is past it, so
rudder forces in the race are extrapolated. Sizing a rudder for 80 mph needs a
supercavitating or ventilated-wedge section, which is a different blade and a
different model.

---

## Race plan the simulation assumes

| | | source |
|---|---|---|
| Rudder | 45 mm chord × 5 in submerged, same mount | team decision |
| Cruise | 50 mph, from a standing start | team decision |
| Steering | differential thrust only below 10 mph, rudder only above | team decision |
| Course | two marks 0.25 mi apart, run as an oval, 2 miles | team, PEP |
| Propulsion | thrust curve scaled so the loaded boat tops out at 55 mph | **assumed**: the prop as logged tops out at 28.8 mph |

---

## Validation status

The model is anchored to the two independent numbers the design report
supplies (`python -m osprey report`, §1):

| Check | Reported | Model | Status |
|---|---|---|---|
| Aero lift fraction at 35.8 m/s | 36% of weight | 36.0% | calibrated to this (not a check) |
| Effective power at 13.4 m/s, no payload | ~2400 W (45 A × 44.4 V × 2, ~60% chain eff.) | 2324 W | **independent, ~3%** |
| Cavitation number at 35.8 m/s | ≈ 0.15 | 0.154 | **independent** |

The 13.4 m/s power check is the meaningful one: nothing in the hull resistance
model was fitted to it. (The MATLAB version reported 2534 W here because it used
the loaded mass for a no-payload run; see `python/PORTING.md`.)

---

## Repository layout

```
python/                 the model. Everything current lives here.
  osprey/               physics, simulation, Lua bridge, studies, report
  controllers/          Lua autopilot scripts
  tests/                pytest: physics checks, MATLAB parity, Lua interface
  README.md             commands and assumptions
  LUA_API.md            the scripting API
  PORTING.md            MATLAB -> Python map, bugs found, known defects
results/
  summary.md            findings with their caveats (the deliverable)
  python/               report.md, figures and Monte Carlo data from `report`
  *.png, *.mat          MATLAB-era outputs, kept for the record
docs/                   handoff, work remaining, constraints, thesis
reference-pack/         the pack for new leads
ASSUMPTIONS.md          every modelling assumption, its reason, and its
                        expected error direction
model/ params/ analysis/ control/ sensors/ tests/ run_all.m
                        the original MATLAB model: FROZEN, kept as the
                        reference the Python parity tests check against
```

## Requirements

Python 3.11+, with numpy, lupa (Lua 5.4), matplotlib and pytest
(`python/requirements.txt`). MATLAB is only needed to run the frozen reference
model.
