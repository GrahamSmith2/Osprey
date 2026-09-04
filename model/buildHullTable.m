function HT = buildHullTable(P, u_max, du)
% BUILDHULLTABLE  Precompute hullSteadyState on a speed grid for interpolation.
%
%   HT = buildHullTable(P)
%
% WHY. hullSteadyState solves two nested bisections (wetted length / running
% trim, and the deadrise inversion) for every call. eom3dof calls it once per
% RK4 stage, i.e. four times per step, i.e. ~16,000 solves for a 20 s run. That
% is the single dominant cost in the whole simulation and it is pure waste:
% every quantity it returns is a smooth function of |u| alone, for a fixed
% parameter set.
%
% So it is evaluated once on a fine grid and interpolated thereafter. The grid
% is fine enough (default 0.02 m/s) that linear interpolation error is far below
% any physical significance -- verified in tests/run_tests.m T17.
%
% The table is attached to P by buildParams. hullSteadyState uses it when
% present and falls back to the full solve when it is not, so nothing else in
% the repo has to know this exists.

if nargin < 2, u_max = 45; end
if nargin < 3, du = 0.02; end

ug = 0:du:u_max;

% Strip any existing table so the reference call does the real solve.
Pref = P;
if isfield(Pref, 'HT'), Pref = rmfield(Pref, 'HT'); end

H = hullSteadyState(ug, Pref);

HT.u    = ug;
HT.du   = du;
HT.umax = u_max;
% Only the fields the dynamics actually consume are tabulated.
HT.SW          = H.SW;
HT.R_total     = H.R_total;
HT.L_aero      = H.L_aero;
HT.f_aero      = H.f_aero;
HT.lambda      = H.lambda;
HT.tau_run_deg = H.tau_run_deg;
HT.regime      = H.regime;
HT.Fn          = H.Fn;
HT.R_fric      = H.R_fric;
HT.R_induced   = H.R_induced;
HT.R_wave      = H.R_wave;
HT.D_aero      = H.D_aero;
HT.L_hydro     = H.L_hydro;
% Scalars, identical at every speed.
HT.SW_disp        = H.SW_disp;
HT.T_draft        = H.T_draft;
HT.CL_alpha_tunnel= H.CL_alpha_tunnel;
HT.lambda_max     = H.lambda_max;
HT.U_transition   = H.U_transition;
end
