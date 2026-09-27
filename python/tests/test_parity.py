"""The Python port must reproduce the MATLAB model.

Reference values are MATLAB outputs recorded before the port (see PORTING.md).
They were all produced with a 0.075 m submerged span (50% of the 150 mm blade),
so that is what these tests use rather than the current 3 in baseline.
"""
from dataclasses import replace

import pytest

from osprey import AS_BUILT, DEG, build_params, simulate, trim_state
from osprey.hull import hull_solve
from osprey.nomoto import identify_nomoto
from osprey.propulsion import prop_max_thrust
from osprey.rudder import rudder_forces
from osprey.sizing import span_sweep

LEGACY = replace(AS_BUILT, h_sub=0.075)


@pytest.fixture(scope="module")
def P():
    return build_params(rudder=LEGACY)


@pytest.mark.parametrize("u,tau,SW,R", [
    (2, 3.34, 1.0600, 59.7), (5, 3.13, 1.0600, 74.0), (8, 2.70, 0.8372, 111.6),
    (10, 2.39, 0.6145, 119.7), (13.4, 1.92, 0.6145, 189.1), (20, 1.50, 0.5012, 324.1),
    (30, 1.50, 0.1663, 306.7), (35.8, 1.50, 0.0861, 281.3)])
def test_hull_steady_state(P, u, tau, SW, R):
    H = hull_solve(u, P.V, P.E, P.m)
    assert H["tau_run_deg"] == pytest.approx(tau, abs=0.006)
    assert H["SW"] == pytest.approx(SW, abs=6e-5)
    assert H["R_total"] == pytest.approx(R, abs=0.06)


def test_aero_lift_calibration(P):
    assert hull_solve(35.8, P.V, P.E, P.m)["f_aero"] == pytest.approx(0.36, abs=1e-9)


@pytest.mark.parametrize("deg,CL,F,N", [
    (2, .0987, 20.13, 12.89), (5, .2467, 50.33, 32.22),
    (8, .3947, 80.52, 51.55), (10, .4933, 100.65, 64.44)])
def test_rudder_forces(P, deg, CL, F, N):
    _, _, Nn, D = rudder_forces(deg * DEG, 13.4, 0, 0, 0, P)
    assert D["CL"] == pytest.approx(CL, abs=6e-5)
    assert D["F_N"] == pytest.approx(F, abs=6e-3)
    assert Nn == pytest.approx(N, abs=6e-3)


def test_ventilated_rudder(P):
    _, _, N, D = rudder_forces(12 * DEG, 13.4, 0, 0, 1, P)
    assert D["CL"] == pytest.approx(0.2960, abs=6e-5)
    assert N == pytest.approx(38.66, abs=6e-3)


def test_prop_calibration(P):
    assert P.KT0 == pytest.approx(0.0261, abs=6e-5)
    assert 2 * prop_max_thrust(13.4, P) == pytest.approx(173.5, abs=0.06)


def test_legacy_kt0_reproduces_matlab_bug():
    """MATLAB calibrated the prop against the loaded-mass table: +9% thrust."""
    good = build_params(rudder=LEGACY).KT0
    bad = build_params(rudder=LEGACY, legacy_kt0=True).KT0
    assert bad / good == pytest.approx(1.090, abs=0.002)


def test_steady_turn(P):
    x0, Tt = trim_state(8, P)
    S = simulate(x0, 25, dict(delta_cmd=8 * DEG, T_cmd_port=Tt, T_cmd_stbd=Tt), P, dt=0.002)
    assert S.u[-1] == pytest.approx(6.852, abs=6e-4)
    assert S.v[-1] == pytest.approx(-0.252, abs=6e-4)
    assert S.r[-1] == pytest.approx(0.2073, abs=6e-5)
    assert S.turn_radius() == pytest.approx(33.08, abs=0.006)


def test_nomoto(P):
    fit = identify_nomoto(P)
    assert fit.K_prime == pytest.approx(0.506, abs=6e-4)
    assert fit.T_prime == pytest.approx(1.084, abs=6e-4)
    assert fit.scaling_holds


def test_span_sweep(P):
    ref = {0.075: 9.53, 0.110: 6.09, 0.150: 4.76, 0.220: 3.79}
    for h, R5, _ in span_sweep(P, spans=tuple(ref)):
        assert R5 == pytest.approx(ref[h], abs=0.006)
