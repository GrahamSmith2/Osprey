function run_tests()
% RUN_TESTS  Sanity checks that must pass before any result is trusted.
%
% Plain asserts, no test framework, no toolbox. Run from anywhere:
%   addpath(genpath('C:\Osprey')); run_tests
%
% Tests covering the 3-DOF EOM (straight-line, steady-turn closure, Jacobian
% agreement, Nomoto fit) are added as those files land. The checks here cover
% the parameter and hull/rudder layers that currently exist.

fprintf('\n=== Osprey sanity checks ===\n');
n_pass = 0; n_fail = 0;

%% T1 -- parameter assembly is self-consistent
P = buildParams();
[n_pass, n_fail] = ck('T1a rudder area = span x chord', ...
    abs(P.R.A_r - P.R.h_sub*P.R.c) < 1e-12, n_pass, n_fail);
[n_pass, n_fail] = ck('T1b added masses are negative (SNAME sign)', ...
    P.X_udot < 0 && P.Y_vdot < 0 && P.N_rdot < 0, n_pass, n_fail);
[n_pass, n_fail] = ck('T1c rudder is aft of CG (x_r < 0)', ...
    P.x_r < 0, n_pass, n_fail);

%% T2 -- area override rescales chord and AR correctly
P3 = buildParams([], struct('R_A_r', 3*2.25e-3));
[n_pass, n_fail] = ck('T2a 3x area at fixed span -> 3x chord', ...
    abs(P3.R.c - 3*0.030) < 1e-9, n_pass, n_fail);
[n_pass, n_fail] = ck('T2b 3x area at fixed span -> AR/3', ...
    abs(P3.R.AR_geom - 2.5/3) < 1e-9, n_pass, n_fail);

%% T3 -- LHS sampler respects every declared bound
U = uncertainty(); f = fieldnames(U);
S = sampleUncertainty('lhs', 200, 7);
ok = true;
for k = 1:numel(f)
    vals = [S.(f{k})];
    ok = ok && all(vals >= U.(f{k}).lo - 1e-12) && all(vals <= U.(f{k}).hi + 1e-12);
end
[n_pass, n_fail] = ck('T3  LHS samples inside all declared bounds', ok, n_pass, n_fail);

%% T4 -- hull: units and dimensions of every force term
H = hullSteadyState([2 8 13.4 20 35.8], P);
[n_pass, n_fail] = ck('T4a all resistance terms finite and >= 0', ...
    all(isfinite(H.R_total)) && all(H.R_total >= 0), n_pass, n_fail);
[n_pass, n_fail] = ck('T4b wetted area never exceeds floating value', ...
    all(H.SW <= H.SW_disp + 1e-12), n_pass, n_fail);
[n_pass, n_fail] = ck('T4c wetted area decreases with speed once planing', ...
    all(diff(H.SW(3:end)) < 0), n_pass, n_fail);
[n_pass, n_fail] = ck('T4d wetted length never exceeds the hull', ...
    all(H.lambda * P.V.b_pad <= P.V.LOA + 1e-9), n_pass, n_fail);
[n_pass, n_fail] = ck('T4e aero lift fraction matches report at design speed', ...
    abs(H.f_aero(end) - P.V.aero_lift_frac_design) < 1e-6, n_pass, n_fail);

% Dimensional check: 0.5*rho*u^2*S*Cf must come out in newtons.
q_dim = 0.5 * P.E.rho_w * 13.4^2 * H.SW(3);   % kg/m^3 * m^2/s^2 * m^2 = kg*m/s^2
[n_pass, n_fail] = ck('T4f dynamic pressure x area has force magnitude', ...
    q_dim > 1e3 && q_dim < 1e6, n_pass, n_fail);

%% T5 -- rudder: sign convention and symmetry
Rp = rudderForces( deg2rad(8), 13.4, 0, 0, 0, P);
Rm = rudderForces(-deg2rad(8), 13.4, 0, 0, 0, P);
[n_pass, n_fail] = ck('T5a positive delta -> positive yaw moment', ...
    Rp.N > 0, n_pass, n_fail);
[n_pass, n_fail] = ck('T5b force is odd in delta', ...
    abs(Rp.N + Rm.N) < 1e-9, n_pass, n_fail);
[n_pass, n_fail] = ck('T5c drag always retards (X <= 0)', ...
    Rp.X <= 0 && Rm.X <= 0, n_pass, n_fail);
[n_pass, n_fail] = ck('T5d zero deflection, zero drift -> zero moment', ...
    abs(rudderForces(0, 13.4, 0, 0, 0, P).N) < 1e-12, n_pass, n_fail);

%% T6 -- rudder force scales as u^2
Ra = rudderForces(deg2rad(5), 10, 0, 0, 0, P);
Rb = rudderForces(deg2rad(5), 20, 0, 0, 0, P);
[n_pass, n_fail] = ck('T6  yaw moment scales as u^2 (4x for 2x speed)', ...
    abs(Rb.N/Ra.N - 4) < 1e-6, n_pass, n_fail);

%% T7 -- ventilation hysteresis actually latches
vs = 0;
r10 = rudderForces(deg2rad(11), 13.4, 0, 0, vs, P);   % should trip on
r08 = rudderForces(deg2rad(8),  13.4, 0, 0, r10.vent_state, P); % stays latched
r04 = rudderForces(deg2rad(4),  13.4, 0, 0, r08.vent_state, P); % re-wets
[n_pass, n_fail] = ck('T7a ventilation trips above onset', ...
    r10.vent_state == 1, n_pass, n_fail);
[n_pass, n_fail] = ck('T7b stays ventilated between off and on angles', ...
    r08.vent_state == 1, n_pass, n_fail);
[n_pass, n_fail] = ck('T7c re-wets below the hysteresis angle', ...
    r04.vent_state == 0, n_pass, n_fail);

% The headline pathology: authority is NON-MONOTONIC in deflection.
vs = 0; N = zeros(1,3); d = [10 12 14];
for i = 1:3
    Rf = rudderForces(deg2rad(d(i)), 13.4, 0, 0, vs, P); vs = Rf.vent_state;
    N(i) = Rf.N;
end
[n_pass, n_fail] = ck('T7d authority DROPS past ventilation onset (expected)', ...
    N(2) < N(1), n_pass, n_fail);

%% T8 -- rudder-induced yaw damping is present and has the right sign
% Boat drifting to starboard (v>0) with zero rudder must produce a restoring
% side force to port at the stern, i.e. a NEGATIVE Y and a bow-up-into-the-flow
% (weathercocking) yaw moment.
Rd = rudderForces(0, 13.4, 1.0, 0, 0, P);
[n_pass, n_fail] = ck('T8  sway drift alone generates rudder side force', ...
    Rd.Y < 0 && abs(Rd.N) > 0, n_pass, n_fail);

%% Summary
fprintf('\n%d passed, %d failed\n\n', n_pass, n_fail);
if n_fail > 0
    error('run_tests: %d sanity check(s) FAILED -- do not trust any result.', n_fail);
end
end

% -------------------------------------------------------------------------
function [p, f] = ck(name, cond, p, f)
if cond
    fprintf('  PASS  %s\n', name); p = p + 1;
else
    fprintf('  FAIL  %s\n', name); f = f + 1;
end
end
