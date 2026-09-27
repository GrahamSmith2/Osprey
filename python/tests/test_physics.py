"""Physics sanity checks, ported from tests/run_tests.m. These must pass before
any result is trusted."""
import math
from dataclasses import replace

import pytest

from osprey import AS_BUILT, PROPOSED, DEG, KGCM, build_params, simulate, trim_state
from osprey.dynamics import eom
from osprey.rudder import peak_servo_torque, rudder_forces


@pytest.fixture(scope="module")
def P():
    return build_params()


def test_rudder_geometry_follows_span_and_chord():
    assert PROPOSED.chord == pytest.approx(1.5 * AS_BUILT.chord)
    assert PROPOSED.h_sub - AS_BUILT.h_sub == pytest.approx(2 * 0.0254)
    assert PROPOSED.A_r == pytest.approx(PROPOSED.h_sub * PROPOSED.chord)


def test_parameter_signs(P):                                        # T1
    assert P.X_udot < 0 and P.Y_vdot < 0 and P.N_rdot < 0          # SNAME
    assert P.x_r < 0                                                # rudder aft of CG


def test_lhs_respects_every_declared_bound():                       # T3
    from osprey.params import UNCERTAINTY, sample_uncertainty
    for d in sample_uncertainty("lhs", 200, 7):
        for k, u in UNCERTAINTY.items():
            assert u.lo - 1e-12 <= d[k] <= u.hi + 1e-12, k


def test_hull_terms_are_physical(P):                                # T4
    from osprey.hull import hull_solve
    H = [hull_solve(u, P.V, P.E, P.m) for u in (2, 8, 13.4, 20, 35.8)]
    assert all(math.isfinite(h["R_total"]) and h["R_total"] >= 0 for h in H)
    assert all(h["SW"] <= h["SW_disp"] + 1e-12 for h in H)
    assert all(a["SW"] > b["SW"] for a, b in zip(H[2:], H[3:]))    # dries out once planing
    assert all(h["lam"] * P.V.b_pad <= P.V.LOA + 1e-9 for h in H)


def test_sign_convention(P):
    assert rudder_forces(8 * DEG, 13.4, 0, 0, 0, P)[2] > 0          # +delta -> +N
    a = rudder_forces(8 * DEG, 13.4, 0, 0, 0, P)[2]
    b = rudder_forces(-8 * DEG, 13.4, 0, 0, 0, P)[2]
    assert a == pytest.approx(-b)
    assert rudder_forces(0, 13.4, 0, 0, 0, P)[2] == 0
    assert rudder_forces(8 * DEG, 13.4, 0, 0, 0, P)[0] <= 0        # drag always retards
    assert rudder_forces(-8 * DEG, 13.4, 0, 0, 0, P)[0] <= 0


def test_rudder_adds_yaw_damping(P):                                # T8
    X, Y, N, _ = rudder_forces(0.0, 13.4, 1.0, 0.0, 0, P)          # drifting, no rudder
    assert Y < 0 and abs(N) > 0


def test_authority_scales_with_u_squared(P):
    a = rudder_forces(5 * DEG, 10, 0, 0, 0, P)[2]
    b = rudder_forces(5 * DEG, 20, 0, 0, 0, P)[2]
    assert b / a == pytest.approx(4.0)


def test_ventilation_latches_and_authority_drops(P):
    _, _, N10, D = rudder_forces(10 * DEG, 13.4, 0, 0, 0, P)
    _, _, N12, D = rudder_forces(12 * DEG, 13.4, 0, 0, 0, P)
    assert D["vent_state"] == 1 and N12 < N10
    assert rudder_forces(8 * DEG, 13.4, 0, 0, 1, P)[3]["vent_state"] == 1   # stays latched
    assert rudder_forces(4 * DEG, 13.4, 0, 0, 1, P)[3]["vent_state"] == 0   # re-wets


def test_straight_line_stays_straight(P):
    x0, Tt = trim_state(13.4, P)
    S = simulate(x0, 8, dict(delta_cmd=0, T_cmd_port=Tt, T_cmd_stbd=Tt), P)
    assert abs(S.r).max() < 1e-12 and abs(S.v).max() < 1e-12


def test_steady_turn_closes(P):
    x0, Tt = trim_state(8, P)
    cmd = dict(delta_cmd=8 * DEG, T_cmd_port=Tt, T_cmd_stbd=Tt)
    S = simulate(x0, 25, cmd, P)
    xdot, D = eom(S.t[-1], list(S.x[-1]), (8 * DEG, Tt, Tt), int(S.vent[-1]), P,
                  S.delta_blade[-1], diag=True)
    assert max(abs(v) for v in xdot[:3]) < 1e-3
    # centripetal closure: -(m - X_udot) u r + sum(Y) = 0
    Ysum = D["Y_hull"] + D["Y_rud"] + D["Y_aero"]
    assert abs(D["cor_sway"] + Ysum) / abs(D["cor_sway"]) < 1e-3
    # yaw budget, Munk moment included, sums to (Iz - N_rdot) * rdot           T10c
    Nsum = D["N_rud"] + D["N_hull"] + D["N_prop"] + D["N_aero"] + D["N_munk"]
    assert Nsum == pytest.approx((P.Iz - P.N_rdot) * xdot[2], abs=1e-9)


def test_numerical_jacobian_is_converged(P):                        # T11
    """No analytic Jacobian exists (saturations, abs, a latched branch); the
    meaningful property is that the numerical one does not depend on step."""
    import numpy as np
    x0, Tt = trim_state(8, P)
    cmd = (8 * DEG, Tt, Tt)
    S = simulate(x0, 25, dict(delta_cmd=cmd[0], T_cmd_port=Tt, T_cmd_stbd=Tt), P, dt=0.002)
    x = np.asarray(S.x[-1], dtype=float)
    f = lambda z: np.asarray(eom(S.t[-1], list(z), cmd, int(S.vent[-1]), P, S.delta_blade[-1]))

    def jac(h):
        J = np.zeros((9, 9))
        for j in range(9):
            e = np.zeros(9); e[j] = h * max(1.0, abs(x[j]))
            J[:, j] = (f(x + e) - f(x - e)) / (2 * e[j])
        return J
    J1, J2 = jac(1e-5), jac(5e-6)
    assert np.linalg.norm(J1 - J2) / np.linalg.norm(J1) < 1e-6


def test_gain_schedule_scaling_exponents(P):                        # T14
    from osprey.nomoto import gain_schedule, identify_nomoto
    fit = identify_nomoto(P)
    g = [gain_schedule(U, fit, P) for U in (4, 8, 16)]
    base = g[0]["Kp_r"] * g[0]["U"] ** 2
    assert all(x["Kp_r"] * x["U"] ** 2 == pytest.approx(base, rel=1e-9) for x in g)   # 1/U^2
    assert all(x["wc_r"] <= 0.8 * x["wc_ceiling"] + 1e-9 for x in g)


def test_current_is_a_frame_shift(P):
    # dt = 0.002 as in MATLAB T12. The result is step-size sensitive because hull
    # resistance does not vanish at zero speed (a known model defect, PORTING.md),
    # so the relative velocity chatters about zero on its way to rest.
    Pc = P.with_env(V_current=1.5, psi_current=0.0)
    S = simulate([0] * 9, 20, dict(delta_cmd=0, T_cmd_port=0, T_cmd_stbd=0), Pc, dt=0.002)
    assert S.u[-1] == pytest.approx(1.5, abs=1e-3)
    assert abs(S.r).max() < 1e-9


def test_compute_delay_is_one_tick(P):
    """The MATLAB queue applied commands with NO delay; the port applies one tick."""
    class Step:
        def __init__(self): self.n = 0
        def step(self, t, x):
            self.n += 1
            return (5 * DEG if self.n >= 2 else 0.0, 0.0, 0.0)
    for legacy, first_move in ((False, 0.04), (True, 0.02)):
        S = simulate([0] * 9, 0.1, Step(), P, dt=0.005, f_ctrl=50, legacy_zero_delay=legacy)
        moved = [t for t, d in zip(S.t, S.delta) if abs(d) > 1e-9]
        assert moved[0] == pytest.approx(first_move + 0.005, abs=1e-9)


def test_servo_peak_is_not_at_ventilation_onset(P):
    """Ventilated lift keeps growing with angle and, at the nominal 50% loss,
    exceeds the onset value. The MATLAB 35 deg point was neither peak."""
    Pw = replace(P, R=replace(P.R, x_stock_frac=0.0))
    full, at = peak_servo_torque(Pw, 20.0)
    clamped, _ = peak_servo_torque(Pw, 20.0, Pw.R.delta_vent_on)
    at35 = rudder_forces(35 * DEG, 20.0, 0, 0, 1, Pw)[3]["tau_servo"]
    assert at > Pw.R.delta_vent_on and full > clamped > at35


def test_balanced_stock_carries_no_hinge_moment(P):
    assert peak_servo_torque(P, 30.0)[0] == pytest.approx(0.0, abs=1e-12)
