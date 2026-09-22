# Three Innovations for PEP27

**Princeton Electric Speedboating** · Craft: *Osprey* · Division: Uncrewed — Autonomy
22 September 2026

To the PEP27 Selection Committee, American Society of Naval Engineers

---

Osprey raced at PEP26 as a hull, a powertrain and a radio. It carried an
autonomous system as installed weight, but every run it has ever made was driven
by a person standing on the bank. Our three innovations all close that gap,
because in the Autonomy division that gap is the race requirement.

---

### 1. GPS path planning, running the course for real this time

Osprey will fly the full two-mile course on waypoint guidance this cycle, autonomously from start to finish. The two details that quietly defeat naive implementations are already solved in simulation: a lookahead distance that scales with speed, which cut simulated cross-track error from 1.79 m to 0.65 m and rudder effort from 8.0° to 3.3° RMS, and an explicit correction between heading and course over ground, without which a boat holds its bearing while sailing steadily downstream and the heading error reads zero throughout. Over the full course in a 15-knot crosswind and a half-metre-per-second current, the simulated boat finishes in 400 seconds holding 1.6 m of track error.

### 2. Control gains derived from the boat, not found by trial and error

Rudder authority grows with the square of speed while hull damping grows roughly linearly, so no single set of PID gains can hold the line across the range a race demands. We fit a first-order Nomoto model from step-rudder responses and schedule the gains on measured speed using the scaling that falls out of it, which holds step overshoot flat at −3% from 4 to 13 m/s where fixed gains degrade threefold at the low-speed end. This cycle we identify those coefficients from the boat itself — zig-zag manoeuvres and turn circles at three speeds, plus a bifilar pendulum swing to measure the yaw inertia that is currently our largest single unknown.

### 3. Full telemetry and control commands that protect the craft

Both of last cycle's failures were diagnosable from data nobody was reading at the time, and the cooling loop has never been shown to flow reliably. We are integrating temperature sensors through the cooling system — at both motors, both controllers and across the heat exchanger — logged every run alongside Castle Link's own data, so that the first sustained thermal test this boat has had produces numbers rather than guesses. The controller will also refuse commands that hurt the craft: a kill on loss of navigation data as the rules require, rudder travel capped at ventilation onset, a yaw-rate ceiling from a roll and blow-over envelope, and an automatic speed reduction on a steering fault, since below about 5 m/s the twin motors regain control of a rudder jam that is unrecoverable at 18.

---

All three rest on a three-degree-of-freedom manoeuvring simulation of Osprey
that we have validated to within 6% against the powertrain data logged during
last year's 30 mph run. We are grateful for ASNE's and ONR's continued support,
and we look forward to racing in April.

Respectfully,

**Graham [Surname]**
President, Princeton Electric Speedboating
graham@smithsnet.us
