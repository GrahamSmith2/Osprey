function P = buildParams(S, overrides)
% BUILDPARAMS  Assemble the single parameter struct that every model file reads.
%
%   P = buildParams()                 % nominal everything
%   P = buildParams(S)                % S from sampleUncertainty()
%   P = buildParams(S, overrides)     % overrides is a struct of dotted paths,
%                                     %   e.g. struct('R_A_r', 4.5e-3)
%
% Model functions take P and nothing else. They never call params_*.m directly,
% so a Monte Carlo run only has to vary S and the whole model follows.
%
% Sub-structs:  P.V vessel   P.E environment   P.R rudder   P.A actuators
% Derived scalars are hoisted to the top level of P for readability in the EOM.

if nargin < 1 || isempty(S), S = sampleUncertainty('nominal'); end
if nargin < 2, overrides = struct(); end

P.V = params_vessel();
P.E = params_env();
P.R = params_rudder();
P.A = params_actuators();
P.S = S;                       % keep the draw attached for traceability

%% ---- Apply the uncertainty draw ---------------------------------------
% Mass / inertia
P.m   = P.V.m;
P.Iz  = P.V.Iz * S.Iz_fac;                        % [kg*m^2]

% Added mass. Signs follow SNAME: X_udot, Y_vdot, N_rdot are all NEGATIVE, so
% (m - X_udot) is m PLUS the added mass. Storing them negative keeps the EOM
% written exactly as in the textbook rather than with ad-hoc sign flips.
P.X_udot = -S.Xudot_frac * P.m;                   % [kg]
P.Y_vdot = -S.Yvdot_frac * P.m;                   % [kg]
P.N_rdot = -S.Nrdot_frac * P.Iz;                  % [kg*m^2]

% Configuration
P.V.x_cg_frac = S.x_cg_frac;
P.V.x_cg      = S.x_cg_frac * P.V.LOA;            % [m] fwd of transom
% Rudder stock is at the transom, so it sits x_cg AFT of the CG.
% x_r is positive-forward, hence negative here.
P.x_r         = -P.V.x_cg;                        % [m]

% Rudder
P.R.k_surface     = S.k_surface;
P.R.x_cp_frac     = S.x_cp_frac;
P.R.delta_vent_on = S.delta_vent_on;
P.R.k_vent_loss   = S.k_vent_loss;
P.R.CLa_fac       = S.CLa_fac;

% Hull derivative multipliers -- applied inside model/hullForces.m
P.fac.Yv = S.Yv_fac;
P.fac.Yr = S.Yr_fac;
P.fac.Nv = S.Nv_fac;
P.fac.Nr = S.Nr_fac;

% Actuation
P.A.backlash      = S.backlash;
P.A.r_rudder_arm  = S.r_rudder_arm;
P.A.tau_thrust    = S.tau_thrust;

%% ---- Overrides (used by the sizing sweep) ------------------------------
% Keys are underscore-joined paths: 'R_A_r' -> P.R.A_r, 'A_backlash' -> P.A.backlash.
ok = fieldnames(overrides);
for k = 1:numel(ok)
    key = ok{k};
    i = strfind(key, '_');
    if ~isempty(i) && isfield(P, key(1:i(1)-1))
        P.(key(1:i(1)-1)).(key(i(1)+1:end)) = overrides.(key);
    else
        P.(key) = overrides.(key);
    end
end

% Keep the rudder geometry self-consistent after a sweep override. The sizing
% sweep varies AREA and SUBMERGED SPAN independently; chord is whatever falls
% out (c = A_r/h_sub), and aspect ratio follows from that. Doing it in this
% order means "3x the area at the same span" correctly means a 3x fatter chord
% and a 3x LOWER aspect ratio -- which is exactly the trade the sweep must see.
P.R.c       = P.R.A_r / P.R.h_sub;
P.R.AR_geom = P.R.h_sub / P.R.c;

end
