"""Every physical constant and every declared uncertainty for the Osprey model.

Ported from params/*.m. SI units throughout; conversions happen here and nowhere
else. Model functions read a single ``Params`` object and nothing else, so a
Monte Carlo draw only has to vary the uncertainty sample and the whole model
follows.

Anything that is *not known* belongs in ``UNCERTAINTY`` as a range, never as a
constant in a model file.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

DEG = math.pi / 180.0
INCH = 0.0254
G = 9.80665
KGCM = 0.0980665          # N*m per kg*cm


# ---------------------------------------------------------------------------
# Vessel — measured geometry and mass properties  [T §3]
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Vessel:
    LOA: float = 2.134                 # [m] length overall, 7 ft 0 in
    beam_deck: float = 0.775           # [m]
    beam_tunnel: float = 0.457         # [m] tunnel width, 18 in
    b_pad: float = 0.141               # [m] sponson planing pad width
    chord_tunnel: float = 1.841        # [m] tunnel aero chord
    deadrise: float = 21.5 * DEG       # [rad] at aft step
    a_hydro: float = 1.45 * DEG        # [rad] step angle
    a_trim: float = 1.5 * DEG          # [rad] design-point running trim
    a_aero: float = 1.8 * DEG          # [rad] tunnel angle of attack
    h_cg: float = 0.12                 # [m] CG above planing surface  (UNVERIFIED)
    y_hull: float = 0.457 / 2 + 0.141 / 2   # [m] centreline to pad centre
    A_lateral: float = 0.30            # [m^2] lateral area above WL  (UNVERIFIED)
    h_cp_aero: float = 0.18            # [m]
    m_dry: float = 30.6                # [kg] measured
    m_payload: float = 13.6            # [kg] 30 lb
    m: float = 44.2                    # [kg] nominal loaded
    g: float = G
    x_cg_frac: float = 0.30            # [-] of LOA forward of transom (rails slide)
    prop_sep: float = 0.598            # [m] lateral prop separation (VERIFY CAD)
    D_prop: float = 0.076              # [m] Graupner K-series 76 mm
    n_motors: int = 2
    U_demo: float = 13.4               # [m/s] logged planing run, no payload
    U_design: float = 35.8             # [m/s] 80 mph design point
    U_plane_on: float = 8.0            # [m/s] planing-onset blend midpoint
    aero_lift_frac_design: float = 0.36

    @property
    def x_cg(self) -> float:           # [m] forward of transom
        return self.x_cg_frac * self.LOA

    @property
    def k_zz(self) -> float:           # [m] yaw radius of gyration, rule of thumb
        return 0.25 * self.LOA

    @property
    def Iz(self) -> float:             # [kg*m^2] NOT MEASURED — swept +/-50%
        return self.m * self.k_zz ** 2

    @property
    def y_p(self) -> float:            # [m] differential-thrust moment arm
        return self.prop_sep / 2


# ---------------------------------------------------------------------------
# Environment — fluid properties and disturbance defaults
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Environment:
    rho_w: float = 1010.0              # [kg/m^3] brackish
    nu_w: float = 1.05e-6              # [m^2/s]
    p_atm: float = 101325.0            # [Pa]
    p_vap: float = 2339.0              # [Pa] at 20 C
    g: float = G
    rho_a: float = 1.225               # [kg/m^3]
    V_current: float = 0.0             # [m/s]
    psi_current: float = 0.0           # [rad] direction the current flows TO
    V_wind: float = 0.0                # [m/s] true wind speed
    psi_wind: float = 0.0              # [rad] direction the wind blows FROM
    wave_N_amp: float = 0.0            # [N*m] band-limited yaw moment amplitude
    wave_f_lo: float = 0.2             # [Hz]
    wave_f_hi: float = 1.5             # [Hz]
    CY_beta: float = 1.2               # [1/rad] aero side-force slope (placeholder)
    x_cp_aero: float = 0.25            # [-] of LOA, aero CoP forward of CG


# ---------------------------------------------------------------------------
# Rudder — geometry is specified by submerged SPAN and CHORD; area and aspect
# ratio follow. That ordering matters: growing area at fixed span means a
# fatter chord and a LOWER aspect ratio, which is the trade the sizing study
# exists to expose.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Rudder:
    name: str
    h_sub: float                       # [m] submerged span at planing trim
    chord: float                       # [m]
    span_total: float                  # [m] full blade length
    k_surface: float = 1.0             # [-] free-surface image factor (1 = ventilated)
    CD0: float = 0.0085                # [-]
    e_oswald: float = 0.85             # [-]
    delta_max: float = 35 * DEG        # [rad] mechanical stop  (ASSUMED — measure)
    x_cp_frac: float = 0.25            # [-] CoP, fraction of chord aft of LE
    x_stock_frac: float = 0.25         # [-] stock axis, fraction of chord aft of LE
    delta_vent_on: float = 10 * DEG    # [rad] ventilation inception
    vent_hysteresis: float = 4 * DEG   # [rad] re-wet this far below inception
    k_vent_loss: float = 0.5           # [-] CL multiplier once ventilated
    vent_blend: float = 1.5 * DEG      # [rad] smoothing width
    CLa_fac: float = 1.0               # [-] knock-down on Whicker-Fehlner

    @property
    def A_r(self) -> float:            # [m^2] wetted area
        return self.h_sub * self.chord

    @property
    def AR_geom(self) -> float:        # [-]
        return self.h_sub / self.chord

    @property
    def AR_eff(self) -> float:
        return self.k_surface * self.AR_geom

    @property
    def CL_alpha(self) -> float:
        """Whicker-Fehlner low-aspect-ratio lift slope [1/rad]."""
        ar = self.AR_eff
        return 1.8 * math.pi * ar / (1.8 + math.sqrt(ar * ar + 4.0)) * self.CLa_fac

    @property
    def authority(self) -> float:
        """A_r * CL_alpha [m^2/rad] — the only design-controlled factor in N_rud."""
        return self.A_r * self.CL_alpha


# The blade on the boat: MHZ Mystic C5000, 150 x 30 mm, ~3 in submerged on
# plane. The thesis's own servo sizing assumed 50% of 150 mm submerged, and the
# team settled on 3 in. Still an estimate — photograph the transom on plane.
AS_BUILT = Rudder(name="as-built MHZ C5000 (30 mm x 3 in)",
                  h_sub=3 * INCH, chord=0.030, span_total=0.150)

# The team's chosen replacement (2026-09-26): chord +50%, 2 in deeper.
# Same mount; the blade extends downward by 2 in.
PROPOSED = Rudder(name="proposed (45 mm x 5 in)",
                  h_sub=5 * INCH, chord=0.045, span_total=0.150 + 2 * INCH)


# ---------------------------------------------------------------------------
# Actuators — servo, linkage, ESC/motor/prop
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Actuators:
    tau_stall_kgcm: float = 74.0       # AGFRC A81FHM HV at 8.4 V  [T §3.4.1]
    t60_noload: float = 0.10           # [s/60deg] PLACEHOLDER — datasheet not supplied
    rate_load_knockdown: bool = True
    tau_servo_lag: float = 0.040       # [s]
    deadband: float = 0.3 * DEG        # [rad] PLACEHOLDER
    f_pwm: float = 333.0               # [Hz]  PLACEHOLDER
    r_servo_horn: float = 0.015        # [m]
    r_rudder_arm: float = 0.015        # [m] NOT SUPPLIED — assumed 1:1
    eta_cable: float = 0.85            # [-]
    backlash: float = 0.5 * DEG        # [rad] free play referred to the rudder
    tau_thrust: float = 0.15           # [s] ESC + motor spool-up
    V_batt: float = 44.4               # [V] 12S nominal
    Kv: float = 680.0                  # [rpm/V]
    n_max: float = 23150 / 60          # [rev/s] logged peak
    I_peak: float = 45.0               # [A] logged peak per motor
    J0_prop: float = 1.35              # [-] zero-thrust advance ratio (ASSUMED)

    @property
    def tau_stall(self) -> float:      # [N*m]
        return self.tau_stall_kgcm * KGCM

    @property
    def rate_max(self) -> float:       # [rad/s]
        return 60 * DEG / self.t60_noload


# ---------------------------------------------------------------------------
# Uncertainty — nominal plus range for every parameter that is NOT known.
# Most entries are multiplicative factors on a nominal defined above.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Uncertain:
    nom: float
    lo: float
    hi: float
    dist: str                          # 'uniform' | 'logunif'
    why: str


UNCERTAINTY: dict[str, Uncertain] = {
    "Iz_fac":        Uncertain(1.0, 0.5, 1.5, "uniform", "Iz never measured; k_zz = 0.25 LOA."),
    "Xudot_frac":    Uncertain(0.05, 0.02, 0.15, "uniform", "Surge added mass / m."),
    "Yvdot_frac":    Uncertain(0.15, 0.05, 0.40, "uniform", "Sway added mass / m."),
    "Nrdot_frac":    Uncertain(0.15, 0.05, 0.40, "uniform", "Yaw added inertia / Iz."),
    "Yv_fac":        Uncertain(1.0, 0.2, 5.0, "logunif", "Clarke regression far outside its Fn domain."),
    "Yr_fac":        Uncertain(1.0, 0.2, 5.0, "logunif", "Clarke regression far outside its Fn domain."),
    "Nv_fac":        Uncertain(1.0, 0.2, 5.0, "logunif", "Clarke regression far outside its Fn domain."),
    "Nr_fac":        Uncertain(1.0, 0.2, 5.0, "logunif", "Clarke regression far outside its Fn domain."),
    "CLa_fac":       Uncertain(1.0, 0.4, 1.2, "uniform", "Whicker-Fehlner, knocked down by ventilation."),
    "k_surface":     Uncertain(1.0, 1.0, 2.0, "uniform", "1 if ventilated, 2 if it stays wetted."),
    "x_cp_frac":     Uncertain(0.25, 0.15, 0.35, "uniform", "Chordwise CoP."),
    "delta_vent_on": Uncertain(10 * DEG, 6 * DEG, 15 * DEG, "uniform", "Ventilation inception."),
    "k_vent_loss":   Uncertain(0.50, 0.30, 0.70, "uniform", "Residual CL once ventilated."),
    "x_cg_frac":     Uncertain(0.30, 0.25, 0.35, "uniform", "Battery/payload rails slide."),
    "backlash":      Uncertain(0.5 * DEG, 0.0, 2.0 * DEG, "uniform", "Cable stretch and horn slop."),
    "r_rudder_arm":  Uncertain(0.015, 0.010, 0.025, "uniform", "NOT SUPPLIED — measure."),
    "tau_thrust":    Uncertain(0.15, 0.10, 0.30, "uniform", "ESC + motor + prop spool-up."),
}
for _k, _u in UNCERTAINTY.items():
    assert _u.lo <= _u.nom <= _u.hi, _k
    assert _u.dist != "logunif" or _u.lo > 0, _k


def sample_uncertainty(mode: str = "nominal", n: int = 1, seed: int = 0) -> list[dict]:
    """Draw uncertainty samples. 'nominal' -> [nominals]; 'lhs' -> n Latin-hypercube
    draws; 'corners' -> [all-lo, all-hi]. LHS is hand-rolled: stratify each
    margin and permute independently, so 500 draws actually fill the space."""
    keys = list(UNCERTAINTY)
    if mode == "nominal":
        return [{k: UNCERTAINTY[k].nom for k in keys}]
    if mode == "corners":
        return [{k: UNCERTAINTY[k].lo for k in keys}, {k: UNCERTAINTY[k].hi for k in keys}]
    if mode != "lhs":
        raise ValueError(f"unknown mode {mode!r}")
    rng = np.random.default_rng(seed)
    q = np.empty((n, len(keys)))
    for j in range(len(keys)):
        q[:, j] = rng.permutation((np.arange(n) + rng.random(n)) / n)
    out = []
    for i in range(n):
        d = {}
        for j, k in enumerate(keys):
            u = UNCERTAINTY[k]
            if u.dist == "uniform":
                d[k] = u.lo + q[i, j] * (u.hi - u.lo)
            else:
                d[k] = math.exp(math.log(u.lo) + q[i, j] * (math.log(u.hi) - math.log(u.lo)))
        out.append(d)
    return out


# ---------------------------------------------------------------------------
# Params — the single object every model function reads
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Params:
    V: Vessel
    E: Environment
    R: Rudder
    A: Actuators
    S: dict
    m: float
    Iz: float
    X_udot: float                      # stored NEGATIVE (SNAME)
    Y_vdot: float
    N_rdot: float
    x_r: float                         # [m] rudder stock, positive FORWARD -> negative
    fac_Yv: float
    fac_Yr: float
    fac_Nv: float
    fac_Nr: float
    KT0: float                         # prop thrust coefficient, calibrated
    HT: object = field(default=None, repr=False)   # precomputed hull table

    def with_env(self, **kw) -> "Params":
        """Same boat, different disturbances (wind, current, waves)."""
        return replace(self, E=replace(self.E, **kw))

    def with_rudder(self, rudder: Rudder) -> "Params":
        """Same boat, same uncertainty draw, different rudder geometry. The
        uncertain rudder properties (k_surface, CoP, ventilation) are re-applied
        from the draw so the comparison is like-for-like."""
        return replace(self, R=replace(
            rudder, k_surface=self.S["k_surface"], x_cp_frac=self.S["x_cp_frac"],
            delta_vent_on=self.S["delta_vent_on"], k_vent_loss=self.S["k_vent_loss"],
            CLa_fac=self.S["CLa_fac"]))


def build_params(S: dict | None = None, *, rudder: Rudder = AS_BUILT,
                 vessel: Vessel | None = None, env: Environment | None = None,
                 act: Actuators | None = None, legacy_kt0: bool = False,
                 top_speed: float | None = None) -> Params:
    """Assemble Params from nominal constants plus one uncertainty draw.

    top_speed [m/s]: if given, the thrust curve is scaled so the loaded boat
    tops out at this speed (the planned propulsion upgrade). If None, the prop
    is calibrated to the logged run, and the loaded boat tops out near 29 mph.

    legacy_kt0 reproduces a bug in the MATLAB version (see PORTING.md): after the
    hull table was added, the prop calibration read the table built at LOADED
    mass instead of the intended dry-mass solve, inflating the thrust ceiling by
    about 9%. Default is the corrected behaviour.
    """
    from .hull import hull_solve, build_hull_table   # local: avoids import cycle
    from .propulsion import calibrate_kt0, size_kt0_for_top_speed

    S = dict(sample_uncertainty("nominal")[0] if S is None else S)
    V = vessel or Vessel()
    E = env or Environment()
    A = act or Actuators()

    V = replace(V, x_cg_frac=S["x_cg_frac"])
    R = replace(rudder, k_surface=S["k_surface"], x_cp_frac=S["x_cp_frac"],
                delta_vent_on=S["delta_vent_on"], k_vent_loss=S["k_vent_loss"],
                CLa_fac=S["CLa_fac"])
    A = replace(A, backlash=S["backlash"], r_rudder_arm=S["r_rudder_arm"],
                tau_thrust=S["tau_thrust"])

    m = V.m
    Iz = V.Iz * S["Iz_fac"]
    HT = build_hull_table(V, E, m)
    if top_speed:
        KT0 = size_kt0_for_top_speed(top_speed, V, E, A, HT)
    else:
        KT0 = calibrate_kt0(V, E, A, HT if legacy_kt0 else None)

    return Params(
        V=V, E=E, R=R, A=A, S=S, m=m, Iz=Iz,
        X_udot=-S["Xudot_frac"] * m,
        Y_vdot=-S["Yvdot_frac"] * m,
        N_rdot=-S["Nrdot_frac"] * Iz,
        x_r=-V.x_cg,
        fac_Yv=S["Yv_fac"], fac_Yr=S["Yr_fac"], fac_Nv=S["Nv_fac"], fac_Nr=S["Nr_fac"],
        KT0=KT0, HT=HT,
    )
