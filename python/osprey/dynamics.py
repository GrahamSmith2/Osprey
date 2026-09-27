"""3-DOF manoeuvring equations of motion, body frame at the CG.

Ported from model/eom3dof.m. Full term-by-term derivation lives there; in short:

  (m - X_udot) udot  = (m - Y_vdot) v r        + X_hull + X_prop + X_rud + X_aero
  (m - Y_vdot) vdot  = -(m - X_udot) u r       + Y_hull + Y_rud  + Y_aero
  (Iz - N_rdot) rdot = (X_udot - Y_vdot) u v   + N_hull + N_rud + N_prop + N_aero

Coriolis/centripetal terms, the Munk moment, speed-squared rudder authority,
quadratic damping and turn-induced drag are all kept; nothing is linearised.

State x = [u, v, r, X, Y, psi, T_port, T_stbd, delta]
  u, v    [m/s]   body-frame surge, sway (GROUND-relative)
  r       [rad/s] yaw rate
  X, Y    [m]     north, east
  psi     [rad]   heading, clockwise from north
  T_*     [N]     delivered thrust (spool-up lag states)
  delta   [rad]   servo output angle (lag state; blade = this minus backlash)
"""
from __future__ import annotations

import math

from .environment import env_forces
from .hull import hull_forces
from .propulsion import prop_forces
from .rudder import rudder_forces


def eom(t, x, cmd, vent, P, delta_blade=None, diag=False):
    """cmd = (delta_cmd, T_cmd_port, T_cmd_stbd). vent is held fixed across RK4
    stages by the caller. Returns xdot (list), plus a diagnostics dict if diag."""
    u, v, r, _, _, psi, Tp, Ts, delta = x
    if delta_blade is None:
        delta_blade = delta

    # Current is a frame shift: hydrodynamics see WATER-relative velocity
    c, s = math.cos(psi), math.sin(psi)
    E = P.E
    cn, ce = E.V_current * math.cos(E.psi_current), E.V_current * math.sin(E.psi_current)
    u_w = u - (c * cn + s * ce)
    v_w = v - (-s * cn + c * ce)

    SW, R_tot = P.HT.lookup(u_w)
    Xh, Yh, Nh, Hd = hull_forces(u_w, v_w, r, P, SW, R_tot)
    Xr, Yr, Nr, Rd = rudder_forces(delta_blade, u_w, v_w, r, vent, P)
    Xp, Yp, Np, T_max = prop_forces(Tp, Ts, u_w, P)
    Xa, Ya, Na = env_forces(u, v, r, psi, t, P)

    m, Iz, Xu, Yv, Nrd = P.m, P.Iz, P.X_udot, P.Y_vdot, P.N_rdot
    cor_surge = (m - Yv) * v * r
    cor_sway = -(m - Xu) * u * r
    n_munk = (Xu - Yv) * u_w * v_w

    udot = (cor_surge + Xh + Xp + Xr + Xa) / (m - Xu)
    vdot = (cor_sway + Yh + Yr + Ya) / (m - Yv)
    rdot = (n_munk + Nh + Nr + Np + Na) / (Iz - Nrd)

    # Actuators: thrust spool-up, saturated at what the prop can deliver
    A = P.A
    T_max = max(T_max, 1e-6)
    d_cmd, Tp_cmd, Ts_cmd = cmd
    Tp_cmd = min(max(Tp_cmd, -0.3 * T_max), T_max)
    Ts_cmd = min(max(Ts_cmd, -0.3 * T_max), T_max)
    Tpdot = (Tp_cmd - Tp) / A.tau_thrust
    Tsdot = (Ts_cmd - Ts) / A.tau_thrust

    # Servo: rate-limited first-order lag with hard stops; the rate ceiling
    # drops as hinge moment eats into stall torque
    dmax = P.R.delta_max
    d_cmd = min(max(d_cmd, -dmax), dmax)
    ddot_raw = (d_cmd - delta) / A.tau_servo_lag
    rate_cap = A.rate_max
    if A.rate_load_knockdown:
        rate_cap *= 1 - min(Rd["tau_servo"] / A.tau_stall, 0.95)
    ddot = math.copysign(min(abs(ddot_raw), rate_cap), ddot_raw) if ddot_raw else 0.0
    if (delta >= dmax and ddot > 0) or (delta <= -dmax and ddot < 0):
        ddot = 0.0

    xdot = [udot, vdot, rdot, u * c - v * s, u * s + v * c, r, Tpdot, Tsdot, ddot]
    if not diag:
        return xdot
    return xdot, dict(
        u_w=u_w, v_w=v_w, beta=math.atan2(v_w, max(abs(u_w), 0.05)),
        chi=math.atan2(xdot[4], xdot[3]),
        N_munk=n_munk, N_rud=Nr, N_hull=Nh, N_prop=Np, N_aero=Na,
        Y_hull=Yh, Y_rud=Yr, Y_aero=Ya, cor_sway=cor_sway,
        vent_state=Rd["vent_state"], tau_servo=Rd["tau_servo"],
        sigma=Rd["sigma"], T_max=T_max, hull=Hd, rudder=Rd)
