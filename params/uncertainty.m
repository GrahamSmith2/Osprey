function U = uncertainty()
% UNCERTAINTY  Definition of every parameter that is NOT known, with its range.
%
% This is the most important file in the repo. The simulation's job is not to
% predict a trajectory -- it is to produce design decisions that survive every
% combination of the values defined here. If a number is uncertain it belongs in
% this file as a RANGE, never in a model file as a constant.
%
% Each field is a struct with:
%   .nom   nominal value used for single deterministic runs
%   .lo    lower bound
%   .hi    upper bound
%   .dist  'uniform' | 'logunif'   (logunif for anything spanning >1 decade,
%                                   so the sampler does not over-weight the top)
%   .why   one line on where the range came from
%
% Most entries are MULTIPLICATIVE FACTORS on a nominal computed elsewhere. That
% keeps a single source of truth for the nominal (params_*.m) and makes a
% "5x uncertainty" literally readable as lo=0.2, hi=5.
%
% Draw samples with params/sampleUncertainty.m.

%% ---- Mass / inertia ----------------------------------------------------
U.Iz_fac = mk(1.0, 0.5, 1.5, 'uniform', ...
    'Iz never measured; k_zz=0.25*LOA rule of thumb. Largest single unknown.');

U.Xudot_frac = mk(0.05, 0.02, 0.15, 'uniform', ...
    'Surge added mass / m. Small for a planing hull. Enters the Munk moment.');

U.Yvdot_frac = mk(0.15, 0.05, 0.40, 'uniform', ...
    'Sway added mass / m. Planing hull carries far less than a displacement hull.');

U.Nrdot_frac = mk(0.15, 0.05, 0.40, 'uniform', ...
    'Yaw added inertia / Iz. Same argument as Yvdot.');

%% ---- Hull manoeuvring derivatives --------------------------------------
% READ THE README BEFORE TRUSTING ANY OF THESE. The nominal values in
% model/hullForces.m come from slender-body / Clarke-type regressions fitted at
% Froude number < 0.3. This boat runs at Fn ~ 2.9 to 7.8. The regressions are
% out of their domain by an order of magnitude, so these are placeholders with
% a deliberately brutal +/-5x range, not estimates.
U.Yv_fac = mk(1.0, 0.2, 5.0, 'logunif', 'Clarke regression far outside its Fn domain.');
U.Yr_fac = mk(1.0, 0.2, 5.0, 'logunif', 'Clarke regression far outside its Fn domain.');
U.Nv_fac = mk(1.0, 0.2, 5.0, 'logunif', 'Clarke regression far outside its Fn domain.');
U.Nr_fac = mk(1.0, 0.2, 5.0, 'logunif', 'Clarke regression far outside its Fn domain.');

%% ---- Rudder ------------------------------------------------------------
U.CLa_fac = mk(1.0, 0.4, 1.2, 'uniform', ...
    'Whicker-Fehlner baseline; knocked down by ventilation and surface effects.');

U.k_surface = mk(1.0, 1.0, 2.0, 'uniform', ...
    'Free-surface image factor. 1.0 if ventilated (expected), 2.0 if it stays wetted.');

U.x_cp_frac = mk(0.25, 0.15, 0.35, 'uniform', ...
    'CoP chordwise position. Drives hinge moment sign as well as magnitude.');

U.delta_vent_on = mk(deg2rad(10), deg2rad(6), deg2rad(15), 'uniform', ...
    'Ventilation inception deflection for a surface-piercing blade.');

U.k_vent_loss = mk(0.50, 0.30, 0.70, 'uniform', ...
    'Residual CL fraction once the suction side is ventilated.');

%% ---- Configuration -----------------------------------------------------
U.x_cg_frac = mk(0.30, 0.25, 0.35, 'uniform', ...
    'Battery and payload slide on rails; fraction of LOA fwd of transom.');

%% ---- Actuation ---------------------------------------------------------
U.backlash = mk(deg2rad(0.5), 0.0, deg2rad(2.0), 'uniform', ...
    'Pull-pull cable stretch and horn slop, as free play referred to the rudder.');

U.r_rudder_arm = mk(0.015, 0.010, 0.025, 'uniform', ...
    'NOT SUPPLIED. Scales required servo torque linearly -- measure this first.');

U.tau_thrust = mk(0.15, 0.10, 0.30, 'uniform', ...
    'ESC + motor + prop inertia spool-up lag.');

end

% -------------------------------------------------------------------------
function s = mk(nom, lo, hi, dist, why)
% MK  Build one uncertainty entry and check it is self-consistent.
assert(lo <= nom && nom <= hi, 'uncertainty: nominal %g outside [%g %g]', nom, lo, hi);
if strcmp(dist, 'logunif')
    assert(lo > 0, 'uncertainty: logunif range must be strictly positive');
end
s = struct('nom', nom, 'lo', lo, 'hi', hi, 'dist', dist, 'why', why);
end
