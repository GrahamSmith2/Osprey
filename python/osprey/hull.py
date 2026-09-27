"""Hull: wetted area, trim and resistance vs speed; sway/yaw damping forces.

Ported from model/hullSteadyState.m, buildHullTable.m and hullForces.m.

The steady-state solve is an independent implementation of Savitsky (1964) for
the two planing pads plus a displacement-mode branch, blended over 6-10 m/s,
anchored to the thesis's one checkable number (tunnel aero lift = 36% of weight
at 35.8 m/s). Running trim is SOLVED per speed, not fixed at the design 1.5 deg:
holding 1.5 deg everywhere gives a wetted length longer than the boat.

The hull derivatives use the Clarke (1982) regression, which is fitted at
Froude number < 0.3. Osprey runs at Fn 2.9-7.8, so those numbers are
placeholders swept +/-5x — the STRUCTURE (linear in U, proportional to immersed
area) is trusted, the magnitudes are not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

NAN = float("nan")


def _bisect(f, lo, hi, iters=40):
    """Plain bisection with MATLAB-identical NaN semantics. Returns NaN if the
    interval does not bracket a root."""
    flo, fhi = f(lo), f(hi)
    if flo * fhi > 0:
        return NAN
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if flo * fm <= 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


def _smoothstep(t):
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    return t * t * (3 - 2 * t)


def _invert_deadrise(CLb, beta_deg):
    """Solve CLb = CL0 - 0.0065*beta*CL0^0.6 for CL0."""
    f = lambda c: c - 0.0065 * beta_deg * c ** 0.6 - CLb
    return _bisect(f, max(CLb, 1e-8), 10 * CLb + 1)


def hull_solve(u, V, E, m):
    """Full steady-state solve at one speed. Returns a dict of every quantity.
    Scalar and pure-Python so it can be tabulated quickly."""
    u = abs(u)
    W = m * E.g

    # 1. Tunnel aero lift, CL_alpha calibrated to 36% of weight at design speed
    S_tunnel = V.beam_tunnel * V.chord_tunnel
    q_design = 0.5 * E.rho_a * V.U_design ** 2
    CL_needed = V.aero_lift_frac_design * W / (q_design * S_tunnel)
    CLa_t = CL_needed / V.a_aero
    q_a = 0.5 * E.rho_a * u * u
    L_aero = min(q_a * S_tunnel * CLa_t * V.a_aero, W)
    AR_t = V.beam_tunnel / V.chord_tunnel
    CL_t = CLa_t * V.a_aero
    CD_t = 0.02 + CL_t ** 2 / (math.pi * AR_t * 0.7)
    D_aero = q_a * S_tunnel * CD_t

    # 2. Savitsky planing branch, two pads sharing the hydrodynamic load
    L_hydro = max(W - L_aero, 0.02 * W)
    b = V.b_pad
    L_per = L_hydro / 2
    tau_deg = math.degrees(V.a_trim)
    beta_deg = math.degrees(V.deadrise)
    Cv = u / math.sqrt(E.g * b)
    denom = 0.5 * E.rho_w * u * u * b * b
    CL_req = L_per / denom if denom > 0 else math.inf
    lambda_max = 0.95 * V.LOA / b
    tau_max = 12.0

    lam, tau_run = NAN, NAN
    if u >= 0.5 and math.isfinite(CL_req):
        CL0 = _invert_deadrise(CL_req, beta_deg)
        sav = lambda l, td: td ** 1.1 * (0.0120 * math.sqrt(l) + 0.0055 * l ** 2.5 / Cv ** 2)
        if sav(lambda_max, tau_deg) >= CL0:
            # (a) enough lift at design trim -> solve for wetted length
            lam = _bisect(lambda l: sav(l, tau_deg) - CL0, 1e-4, lambda_max)
            tau_run = tau_deg
        else:
            # (b) pinned at full hull length -> solve for the trim required
            lam = lambda_max
            tau_run = _bisect(lambda td: sav(lambda_max, td) - CL0, tau_deg, 60)
            if not math.isfinite(tau_run) or tau_run > tau_max:
                tau_run = tau_max

    SW_plane = 2 * lam * b * b / math.cos(V.deadrise)

    # 3. Displacement branch
    Vol_half = m / E.rho_w / 2
    L_wl = 0.85 * V.LOA
    A_sect = Vol_half / L_wl
    T_draft = math.sqrt(A_sect * math.tan(V.deadrise))
    SW_disp = 2 * (1.7 * L_wl * T_draft + Vol_half / T_draft)

    # 4. C1 blend over planing onset
    U_lo, U_hi = 0.75 * V.U_plane_on, 1.25 * V.U_plane_on
    s = _smoothstep((u - U_lo) / (U_hi - U_lo))
    if not math.isfinite(SW_plane):
        SW_plane = SW_disp
    SW = min((1 - s) * SW_disp + s * SW_plane, SW_disp)

    # 5. Resistance
    Lc = max(lam * b, 0.1) if math.isfinite(lam) else 0.1   # MATLAB max() ignores NaN
    L_char = (1 - s) * L_wl + s * Lc
    Re = max(u * L_char / E.nu_w, 1e4)
    Cf = 0.075 / (math.log10(Re) - 2) ** 2
    R_fric = 0.5 * E.rho_w * u * u * SW * Cf * 1.15
    tau_eff = tau_run if math.isfinite(tau_run) else tau_max
    R_induced = L_hydro * math.tan(math.radians(tau_eff))
    Fn = u / math.sqrt(E.g * V.LOA)
    R_wave = (1 - s) * (0.06 * W * math.exp(-((Fn - 0.5) / 0.35) ** 2))
    R_total = R_fric + R_induced + R_wave + D_aero

    return dict(u=u, SW=SW, SW_plane=SW_plane, SW_disp=SW_disp, lam=lam,
                tau_run_deg=tau_eff, lambda_max=lambda_max, L_aero=L_aero,
                f_aero=L_aero / W, L_hydro=L_hydro, R_fric=R_fric,
                R_induced=R_induced, R_wave=R_wave, D_aero=D_aero,
                R_total=R_total, regime=s, T_draft=T_draft,
                CL_alpha_tunnel=CLa_t, Fn=Fn)


@dataclass(frozen=True)
class HullTable:
    """hull_solve tabulated on a speed grid. The RK4 inner loop calls the hull
    four times per step; every returned quantity is a smooth function of |u|
    for a fixed parameter set, so interpolating is the same model, faster."""
    du: float
    umax: float
    SW: tuple
    R_total: tuple
    SW_disp: float
    T_draft: float

    def lookup(self, u):
        uq = min(abs(u), self.umax)
        i = min(int(uq / self.du), len(self.SW) - 2)
        w = (uq - i * self.du) / self.du
        return (self.SW[i] * (1 - w) + self.SW[i + 1] * w,
                self.R_total[i] * (1 - w) + self.R_total[i + 1] * w)


def build_hull_table(V, E, m, u_max=45.0, du=0.02):
    n = int(round(u_max / du)) + 1
    SW, R = [], []
    H = None
    for i in range(n):
        H = hull_solve(i * du, V, E, m)
        SW.append(H["SW"])
        R.append(H["R_total"])
    return HullTable(du=du, umax=u_max, SW=tuple(SW), R_total=tuple(R),
                     SW_disp=H["SW_disp"], T_draft=H["T_draft"])


def hull_forces(u, v, r, P, SW=None, R_total=None):
    """Sway force, yaw moment and surge resistance at the current state.
    u, v are WATER-relative. Returns (X, Y, N, detail_dict)."""
    V, E = P.V, P.E
    if SW is None:
        SW, R_total = P.HT.lookup(u)
    U = max(abs(u), 0.05)

    # Clarke (1982), per demihull, doubled; geometry at the displacement condition
    L = 0.85 * V.LOA
    T = P.HT.T_draft
    Vol = P.m / E.rho_w / 2
    B = 2 * T / math.tan(V.deadrise)
    CB = Vol / (L * B * T)
    TL, BT, BL = T / L, B / T, B / L
    k = math.pi * TL * TL
    Yv_p = -k * (1 + 0.4 * CB * BT)
    Yr_p = -k * (-0.5 + 2.2 * BL - 0.08 * BT)
    Nv_p = -k * (0.5 + 2.4 * TL)
    Nr_p = -k * (0.25 + 0.039 * BT - 0.56 * BL)

    # Dimensionalise (linear in U) and scale by immersed area
    s_area = min(max(SW / P.HT.SW_disp, 0.02), 1.0)
    q2 = 0.5 * E.rho_w * U
    Yv = 2 * q2 * L ** 2 * Yv_p * s_area * P.fac_Yv
    Yr = 2 * q2 * L ** 3 * Yr_p * s_area * P.fac_Yr
    Nv = 2 * q2 * L ** 3 * Nv_p * s_area * P.fac_Nv
    Nr = 2 * q2 * L ** 4 * Nr_p * s_area * P.fac_Nr
    Y_lin = Yv * v + Yr * r
    N_lin = Nv * v + Nr * r

    # Quadratic cross-flow drag by strip theory: one Cd gives Y, N and coupling
    T_eff = 2 * T * s_area
    n_strip = 20
    dx = L / (n_strip - 1)
    c0 = -0.5 * E.rho_w * 1.0 * T_eff * dx
    Yq = Nq = 0.0
    for i in range(n_strip):
        x = -L / 2 + i * dx
        vl = v + r * x
        f = c0 * vl * abs(vl)
        Yq += f
        Nq += f * x
    Y_quad = Yq * P.fac_Yv
    N_quad = Nq * P.fac_Nr

    # Surge: resistance opposes WATER-relative motion, plus turn-induced drag
    # from an energy argument (lateral/yaw power must come out of surge)
    sgn = 1.0 if u > 0 else (-1.0 if u < 0 else 0.0)   # MATLAB sign(); numpy-safe
    X_res = -sgn * R_total
    Y_tot = Y_lin + Y_quad
    N_tot = N_lin + N_quad
    X_turn = -(abs(Y_tot * v) + abs(N_tot * r)) / U

    return (X_res + X_turn, Y_tot, N_tot,
            dict(Yv=Yv, Yr=Yr, Nv=Nv, Nr=Nr, Y_lin=Y_lin, Y_quad=Y_quad,
                 N_lin=N_lin, N_quad=N_quad, X_res=X_res, X_turn=X_turn,
                 s_area=s_area))
