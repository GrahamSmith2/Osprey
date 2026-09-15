# Osprey — Reference Pack

**Assembled:** 9 September 2026.
**Scope:** the boat as designed, built and tested — hull, driveline, powertrain,
electrical and cooling. Rudder-sizing and autonomy analysis is deliberately
**not** in this pack; see *What was left out* below.

Read in numbered order. `01` is the authority for anything about the as-built
vessel; everything else exists to get you to the relevant part of it faster.

---

## Documents

| # | File | What it is | Read it for |
|---|---|---|---|
| 01 | `01_Osprey_thesis_Astudillo_Lee_2026.pdf` | Astudillo & Lee, *"Osprey: High-Speed Electric Unmanned Surface Vessel for Payload Transport"*, Princeton MAE senior thesis, 23 April 2026, 93 pp | **The authoritative as-built record.** Hull sizing and manufacture, driveline, powertrain selection, cooling, electrical architecture, test results, and their own recommendations (§7.2) |
| 02 | `02_Thesis_defense_presentation.pdf` | The defense presentation for the same work, 39 slides | The fast visual version of `01` — the build sequence and the component photography in particular |
| 03 | `03_Status_and_Work_Remaining.pptx` | Status deck, Fall 2026, 14 slides | Where the boat stands now and what the next cycle has to do. Speaker notes on every slide |
| 04 | `04_Work_remaining_next_cycle.md` | Next-cycle scope, tagged by source | The work item list — mechanical `M1–M12`, electrical `E1–E13` — with the reasoning and the open questions behind each |
| 05 | `05_Leads_brief_and_handover_agenda.pdf` (`.md` alongside) | Team-lead brief and the agenda for the handover meeting with Aidan and Sean | The questions that only the previous team can answer, and the decisions the team owes itself |

## `figures/`

Diagrams and schematics lifted from the thesis, at the page resolution they
appear there.

| File | Shows |
|---|---|
| `final_hull_cad.png` | Final hull scaling — CAD, front and rear views |
| `driveline_components.png` | Thrust bearing, shaft clutch collet, flex shaft, stuffing tube, stinger, propeller |
| `driveline_plates.png` | Middle, tube and front motor support plates |
| `powertrain_motor_esc_battery.png` | Castle 2535 motor, Hydra Cobra 5 HV ESC, Turnigy pack |
| `cooling_system_components.png` | Motor water jacket, pump, inline filter, B3-5A plate heat exchanger |
| `water_pickup_coolant_outlet.png` | Raw-water pickup and coolant outlet as installed |
| `lv_electrical_schematic.png` | KiCad schematic of the 12 V system |
| `planing_run_data_log.png` | The logged planing run and its discussion |

## `photos/`

Eighteen photographs extracted from the thesis PDF at full resolution — they are
not otherwise available as image files. Numbered roughly in build order: the boat
on the water, hull layup and infusion, assembly, the electronics bay, the
powertrain and cooling components, and the shaft clutch collet (`17`) whose wear ended the last
campaign. `19` is the Castle Link log showing 23,150 RPM at 45 A.

---

## What was left out, and why

You asked for the important documentation **without** the rudder-sizing and
autonomy material. Excluded on that basis:

- **`README.md`, `ASSUMPTIONS.md`, `docs/HANDOFF.md`** — the manoeuvring
  simulation: its architecture, its assumptions register, and the ArduPilot
  SITL recipe.
- **`results/summary.md`** and the simulation figures (`rudder_sizing_band`,
  `rudder_authority_and_cavitation`, `monte_carlo_robustness`,
  `openloop_turn_and_munk`, `course_sim`, `failure_modes`,
  `hull_regime_vs_speed`) — rudder sizing and control design.
- **`results/autopilot_params.txt`** — exported autopilot parameters.
- **`figures/rudder_block.png`, `steering_assembly_cad.png`,
  `steering_components.png`**, and the two steering photographs.

All of it remains in the repository at
<https://github.com/GrahamSmith2/Osprey> if it is wanted later.

### Three caveats on that boundary

1. **The thesis covers steering hardware** in §3.4 and the rudder appears
   throughout its weight, drag and control discussion. It has not been altered —
   the exclusion applies to the separate analysis documents, not to the
   as-built record.
2. **`04` and `05` are mixed documents.** `04_Work_remaining_next_cycle.md`
   carries a rudder-span section (§1.2) and a software/autonomy part (Part 3);
   `05` carries rudder findings in §2 and an autonomy block (Block D). They are
   included whole because they are the scope and handover records and cutting
   them would leave the work-item numbering with holes. Skip those sections.
3. **`03` is fourteen slides**, of which one covers rudder sizing and two cover
   autonomy. Delete slides 7, 8 and 11 for a version without them.
