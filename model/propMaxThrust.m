function [T_max, info] = propMaxThrust(u, P)
% PROPMAXTHRUST  Per-motor available thrust ceiling vs forward speed.
%
%   [T_max, info] = propMaxThrust(u, P)     u scalar or vector [m/s]
%
% Standard open-water form, at the logged peak shaft speed:
%       J  = u / (n*D)                      advance ratio        [-]
%       KT = KT0 * (1 - J/J0)               thrust coefficient   [-]
%       T  = rho * n^2 * D^4 * KT           thrust per prop      [N]
%
% CALIBRATION, NOT INVENTION. Open-water data for this prop is not available,
% so rather than pick KT0 from a generic table the value is SOLVED so that the
% model reproduces the one logged operating point:
%
%   at the demonstrated top speed (13.4 m/s, NO payload, m = 30.6 kg) and the
%   logged peak shaft speed (23,150 rpm), thrust must exactly balance the hull
%   resistance at that condition -- because at top speed, by definition, it does.
%
% That pins KT0. J0, the zero-thrust advance ratio (essentially a pitch ratio),
% is still assumed at 0.9, which is typical for a high-speed prop. So one
% invented number remains instead of two, and the map is anchored to real data.
%
% WHAT THIS AFFECTS. The differential-thrust authority curve and therefore the
% rudder/thrust allocator crossover speed -- treat that speed as +/-30%. It does
% NOT touch the rudder sizing band, which is set by rudder physics alone.

persistent KT0_cal
V = P.V;  E = P.E;

D  = V.D_prop;                                  % [m]
n  = P.A.n_max;                                 % [rev/s]
J0 = P.A.J0_prop;                               % [-] zero-thrust J (see params)

%% ---- One-time calibration against the logged top-speed run ------------
if isempty(KT0_cal)
    % Resistance at the DEMONSTRATED condition: no payload, so dry mass.
    P_demo = P;  P_demo.m = V.m_dry;
    H_demo = hullSteadyState(V.U_demo, P_demo);
    T_req_per_prop = H_demo.R_total / V.n_motors;               % [N]

    J_demo = V.U_demo / (n * D);
    KT_demo_shape = (1 - J_demo/J0);
    KT0_cal = T_req_per_prop / (E.rho_w * n^2 * D^4 * KT_demo_shape);
end

%% ---- Evaluate ---------------------------------------------------------
J  = u ./ (n * D);
KT = KT0_cal * (1 - J/J0);
KT = max(KT, 0);                                % no negative thrust at over-speed

T_max = E.rho_w * n^2 * D^4 * KT;               % [N] per prop

info.KT0   = KT0_cal;
info.J0    = J0;
info.J     = J;
info.KT    = KT;
info.u_zero_thrust = J0 * n * D;                % [m/s] speed where thrust runs out
end
