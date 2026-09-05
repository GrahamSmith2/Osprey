# Osprey — Lead Brief & Handover Meeting Agenda

**Internal.** For team leads. Read §1–3 before Monday (about ten minutes), bring
§4 to the meeting with Aidan and Sean.

**Meeting:** Monday, with Aidan Astudillo and Sean Lee, who designed, built and
raced Osprey.

> **This is probably the only time we get them in a room.** They graduate, and
> most of what is in their heads was never written down. §4 is ordered so that
> if we only get through the first block, we still got the things that cannot be
> recovered any other way.

---

## 1. Where the project stands

Osprey is a 7 ft, 97 lb twin-motor electric catamaran, carbon hull laid up
in-house. It planes and has run **30 mph** on the water. It races at the
**ASNE/ONR Promoting Electric Propulsion** competition in Portsmouth VA, in the
**Autonomy division** — a 2-mile course, four laps of an oval between two marks.

Three things happened at PEP26, all of them diagnosed by Aidan and Sean:

| | what | status |
|---|---|---|
| Driveline | flex shaft escaped its collet mid-race, stern flooded | root cause known, **redesign not done** |
| Motor controller | one ESC failed on the bench, traced to a battery discharge-rating mismatch | packs already replaced; **one ESC still needs replacing** |
| Cooling | raw-water side never flowed reliably through the heat exchangers | **never fixed, never properly tested** |

**There is an autonomous system on the boat already.** The thesis budgets 5 lb
for it in the weight table and lists it as integrated into the electrical system
alongside the RC gear — but then states plainly that it is *"outside the scope of
this thesis"* [T §3.2]. So it exists, it is wired in, and **it was documented
somewhere other than the thesis, or not at all.** Finding out which is the most
important thing we get from Monday.

What the thesis does confirm: the RC system worked, giving stable wireless
control and telemetry on the water, and early runs without payload reached
30 mph on plane [T §7.1]. Whether the autonomous system has ever actually
steered the boat is not stated either way.

---

## 2. What has been added since

A manoeuvring simulation and control design now exist at
[github.com/GrahamSmith2/Osprey](https://github.com/GrahamSmith2/Osprey). It is
**not** a trajectory predictor — the hull coefficients are unknown to within ±5×
and everything is built to survive that. Reference docs in the repo:
`results/summary.md` (findings), `ASSUMPTIONS.md`, `docs/HANDOFF.md`
(architecture and software gotchas), `docs/WORK_REMAINING.md` (full scope).

Four results that change what we should do:

**The rudder should be sized on span, not area.** Authority is the product
`A_r · CL_α`. Growing area at fixed span grows the chord, which collapses the
aspect ratio and cancels most of the gain: **30× the area buys 32% more
authority.** Growing the span instead buys **5.4× for a third of the area
increase.** A naive "make the rudder bigger" would have wasted the effort.

**The course does not need a bigger rudder — but the uncertainty does.** The
oval needs under 2° of steady rudder at every speed. However, across the
plausible parameter range the current blade spends **41% of a lap pinned at its
useful limit**, against 0% for a 150 mm span blade. The argument for a new
rudder is margin, not turning.

**The powertrain looks about 4× short of the 80 mph design point.** Hull
resistance implies ~10 kW effective at 35.8 m/s, so ~16 kW electrical at the
63% chain efficiency their own log implies — roughly **180 A per motor against
the 45 A they measured**. This is independent of prop assumptions.

**Rule 21 requires a kill on GPS loss.** The craft must stop when navigation
data is missing or corrupted, and judges watch it demonstrated. Whether the
existing autonomous system does this is unknown.

---

## 3. Decisions we owe ourselves

| # | Decision | Blocked on |
|---|---|---|
| D1 | **What speed are we actually building for?** | Sets the ESC, wiring, battery and cooling load. Everything electrical waits on this. The thesis already floats swapping to **LMT 7065** motors, which would need a new front support plate — so there is a defined upgrade path if we want one. |
| D2 | Driveline retention scheme | Nothing. Someone needs to own it. |
| D3 | Rudder span and stock position | One measurement (§4, Q3) |
| D4 | Steering allocation architecture | ArduPilot drives a rudder **or** twin motors, not both. Needs a decision before any tuning. |
| D5 | Are we entering PEP27 in Autonomy again? | Determines whether autonomy is the priority or a stretch goal |

**D1 is the one to force.** Nearly every electrical question downstream is
undefined until it is answered, and it is a genuine choice rather than a
constraint: a boat that reliably finishes at 20 mph scores better than a fast
one that does not, because completing the distance is worth 40 points against 20
for winning.

---

## 4. Questions for Aidan and Sean

### Block A — things only they know

*If we run out of time, we must have got through this block.*

| # | Question | Why |
|---|---|---|
| A1 | **What exactly wore on the collet, and do you still have the failed part?** | The redesign depends on the wear mode. A photo or the part itself is worth more than the write-up. |
| A2 | **Where does the rudder stock sit on the blade, as a fraction of chord?** | Decides whether the existing servo is adequate. The answer swings the requirement from ~0 to 45 kg·cm. We were going to measure it; you may just know. |
| A3 | **What is the rudder tiller arm radius?** | Scales servo torque linearly. Never recorded. |
| A4 | **Why do you think the raw-water side never flowed?** Air lock, pickup geometry, insufficient dynamic pressure, blockage? | Determines whether we add pumps or redesign the pickups. |
| A5 | **What is your read on 45 A when you expected 100?** Prop pitch, diameter, slip, cavitation? | This is the single biggest open question on the boat. |
| A6 | **Was there ever a problem with the 12 V battery, or is it just old?** | The work list says "new 12 V battery" but the thesis reports no fault. |
| A7 | **What did you try on cooling that did not work?** | Stops us repeating it. |
| A8 | **How much of the boat is one-off?** Which parts would be hard to replace if we broke them? | Risk register for on-water testing. |

### Block B — files and hardware to walk away with

*Ask for these explicitly. Get them on a drive in the room if possible.*

| # | Ask |
|---|---|
| B1 | **CAD** — hull, molds, driveline, steering assembly. Native files, not just STEP. |
| B2 | **The MATLAB scripts from Appendix A.1** — hull sizing, `aeroForce`, `calcWettedAreaIterative`. We reimplemented the hull model independently and want to cross-check. |
| B3 | **Raw data logs** — the 30 mph run, ESC ripple logs, any temperature data. Specifically **the Castle Link logs**: the thesis notes Castle Link exposes temperature, RPM and power output, and that they never got to use it properly. Those logs would go a long way toward answering A5. |
| B3b | **Anything from whoever built the autonomous system** — see Block D. |
| B4 | **Are the hull molds still usable, and where are they?** |
| B5 | **Spare parts inventory** — shafts, collets, props, connectors. What is on the shelf? |
| B6 | **Anything they'd want back**, and **permission to keep their thesis in our public repo** (currently there with attribution; better to have it explicitly). |

### Block C — competition and race day

| # | Question | Why |
|---|---|---|
| C1 | **Confirm the course: two marks, quarter mile apart, oval, four laps.** Is a lap really half a mile? | Marks 0.25 mi apart puts 0.5 mi of straight in a lap *before* turning, so a lap is 0.54–0.58 mi. The two figures cannot both be exact. |
| C2 | **How is distance actually scored — GPS track, lap counting, or judges?** | Changes whether tight rounding helps or hurts. |
| C3 | **What did the judges actually care about at inspection?** | Kill switch, buoyancy, tow points, C-rating. Where did teams lose points? |
| C4 | **How did the 5-minute launch go?** What slowed you down? | It is a hard gate. |
| C5 | **What surprised you about race day?** | Open-ended and usually the most valuable question in the room. |
| C6 | **What did the boats that beat you do differently?** | |

### Block D — the autonomous system, which the thesis deliberately excludes

*The thesis budgets 5 lb for an "Autonomous System" and lists it as integrated
into the electrical system, then says it is outside its scope. Everything below
follows from that one sentence, and D1–D3 are close to Block A in priority.*

| # | Question | Why |
|---|---|---|
| D1 | **What actually is the autonomous system?** Which flight controller, GPS, compass, telemetry radio, companion computer? | 5 lb is a substantial amount of hardware. We may already own most of what we need. |
| D2 | **Who built it, and where is it documented?** Another student, another thesis, a repo, or nothing? | If there is a second document we should be reading it before Monday, not rediscovering it. |
| D3 | **Has it ever driven the boat?** Even a straight line, even a single waypoint. | Decides whether next year is "integrate and tune" or "start from scratch". |
| D4 | **Is it wired to the steering servo and throttle now**, and what happens on RC/autonomous handover? | Handover behaviour is a safety item and a competition requirement. |
| D5 | **What is the RC failsafe today?** What does the boat do on signal loss? | Rule 21 needs a kill on bad navigation data; the RC failsafe is the obvious place to hang it. |
| D6 | **How does the boat behave at low speed?** Does it steer at all below planing, and what happens coming off the plane? | |
| D7 | **Does it track straight hands-off, or is there a turning bias?** | |
| D8 | **What does it do in a hard turn** — hook, slide, lift a sponson? | |

*D6–D8 matter because the simulation is guessing at exactly these behaviours
within a factor of five, and they have stood on the bank watching the boat do
them.*

### Block E — judgement, not data

*Worth reserving ten minutes for. Ask these even if the meeting is running long.*

- **If you had another year on this boat, what is the first thing you would
  change?**
- **What did you underestimate most?**
- **What is the biggest risk you think we are not seeing?**
- **Is 80 mph realistic with this powertrain, or was that always aspirational?**
  *(Our numbers say roughly 4× short. Their view on this settles D1.)*
- **You mention the LMT 7065 as a future motor. How seriously did you look at
  it, and what made you stop short?**
- **Would you do the autonomy division again, or race a different class?**

---

### Already answered — do not spend meeting time on these

The thesis §7.2 solves these. Listed so nobody rediscovers them the hard way.

- **Bending the brass stuffing tubes.** Kinking is easy and hard to repair, and
  brass springback beats a jig on its own. Their fix: anneal the tubes in a
  filament dryer at 85 °C while held in the jig. Print the jigs in **ASA**, not
  PLA — PLA's glass transition is 60–65 °C and the jigs warp at annealing
  temperature.
- **Cooling, the direction of the fix.** Fit pumps to pull raw water rather than
  relying on hydrodynamic pressure at speed. A4 is about *why* it failed, not
  what to do instead.
- **Tests they know were never done:** range testing (distance, time, speed and
  power draw for a given pack), CG sensitivity, and sustained thermal. Worth
  confirming none of these happened quietly after the thesis was submitted.

---

## 5. Running the meeting

- **Record it**, with their permission. Half of Block E will be worth
  re-listening to.
- **One person on questions, one taking notes.** Do not have everyone
  interrogating at once.
- **Get Block B on a drive before anyone leaves the room.** Files are the thing
  that quietly disappears after graduation.
- **Bring the boat if it is accessible.** A2, A3 and A8 get answered in thirty
  seconds standing next to it, and they will point at things they would never
  think to write down.
- Circulate answers back into `docs/WORK_REMAINING.md` afterwards — anything
  currently marked **[OPEN]** should be resolvable from this meeting.
