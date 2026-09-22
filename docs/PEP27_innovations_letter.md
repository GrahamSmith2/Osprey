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

The Risk category carries more weight this year, and both of last cycle's failures were diagnosable from data that nobody was reading at the time. Every run will now be logged end to end, and the controller will refuse commands that hurt the boat: a kill on loss or corruption of navigation data as the rules require, rudder travel limited to ventilation onset rather than the mechanical stop, a commanded yaw-rate ceiling from a roll and blow-over envelope, and an automatic speed reduction on a steering fault. Each of those limits comes from an analysis already completed — below about 5 m/s, for instance, the twin motors regain control of a rudder jam that is unrecoverable at 18.

---

All three rest on a three-degree-of-freedom manoeuvring simulation of Osprey
that we have validated to within 6% against the powertrain data logged during
last year's 30 mph run. We are grateful for ASNE's and ONR's continued support,
and we look forward to racing in April.

Respectfully,

**Graham [Surname]**
President, Princeton Electric Speedboating
graham@smithsnet.us
