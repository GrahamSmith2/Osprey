"""Propulsion: twin props as surge force plus a differential-thrust yaw moment.

Ported from model/propForces.m and propMaxThrust.m.

The controller commands THRUST; a first-order lag (a state in the dynamics)
represents ESC and rotor spool-up. The KT(J) map is used only for the available
thrust CEILING. KT0 is calibrated against the logged top-speed run so only J0,
the zero-thrust advance ratio, is assumed.
"""
from __future__ import annotations


def calibrate_kt0(V, E, A, legacy_table=None):
    """Solve KT0 so that, at the demonstrated top speed with NO payload and the
    logged peak shaft speed, thrust balances hull resistance.

    legacy_table: pass the loaded-mass hull table to reproduce the MATLAB bug
    where the calibration read the table instead of solving at dry mass."""
    from .hull import hull_solve
    if legacy_table is not None:
        R = legacy_table.lookup(V.U_demo)[1]
    else:
        R = hull_solve(V.U_demo, V, E, V.m_dry)["R_total"]
    T_req = R / V.n_motors
    n, D = A.n_max, V.D_prop
    J_demo = V.U_demo / (n * D)
    return T_req / (E.rho_w * n * n * D ** 4 * (1 - J_demo / A.J0_prop))


def size_kt0_for_top_speed(U_top, V, E, A, table):
    """KT0 such that the LOADED boat's thrust ceiling equals its resistance at
    U_top, i.e. propulsion upgraded so U_top is the top speed with payload.

    This is a stand-in for the ESC/prop/power upgrade, not a prop design: it
    scales the whole thrust curve (J0 unchanged), so static thrust and launch
    acceleration scale up with it and are optimistic."""
    n, D = A.n_max, V.D_prop
    J = U_top / (n * D)
    if J >= A.J0_prop:
        raise ValueError(f"top speed {U_top:.1f} m/s is past the prop's zero-thrust "
                         f"speed {A.J0_prop * n * D:.1f} m/s")
    T_req = table.lookup(U_top)[1] / V.n_motors
    return T_req / (E.rho_w * n * n * D ** 4 * (1 - J / A.J0_prop))


def prop_max_thrust(u, P):
    """Per-motor thrust ceiling [N] at forward speed u."""
    n, D = P.A.n_max, P.V.D_prop
    J = u / (n * D)
    KT = max(P.KT0 * (1 - J / P.A.J0_prop), 0.0)
    return P.E.rho_w * n * n * D ** 4 * KT


def prop_forces(T_port, T_stbd, u, P):
    """(X, Y, N, T_max). Positive N is a turn to starboard, so more port thrust
    than starboard gives positive N."""
    return (T_port + T_stbd, 0.0, (T_port - T_stbd) * P.V.y_p,
            prop_max_thrust(u, P))
