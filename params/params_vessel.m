function V = params_vessel()
% PARAMS_VESSEL  Measured geometry and mass properties of the Osprey USV.
%
% Every value here is either measured, taken from the design report, or a
% CAD-derived number flagged VERIFY. Nothing in this file is a guess -- guesses
% live in params/uncertainty.m and are swept, not fixed.
%
% Frame convention (SNAME body-fixed, origin at CG):
%   x  forward (+ toward bow),  y  starboard (+),  z  down (+)
%   u  surge,  v  sway,  r  yaw rate,  psi  heading (+ clockwise from North)
% Lever arms: x_r is POSITIVE FORWARD, so an aft-mounted rudder has x_r < 0.
%
% All values SI. Conversions happen here at the boundary, never downstream.

%% ---- Hull geometry -----------------------------------------------------
V.LOA          = 2.134;      % [m]  length overall (7.0 ft)
V.beam_deck    = 0.775;      % [m]  deck beam (30.5 in)
V.beam_tunnel  = 0.457;      % [m]  tunnel width (18.0 in)
V.b_pad        = 0.141;      % [m]  sponson planing pad width (5.56 in)
V.chord_tunnel = 1.841;      % [m]  tunnel aero chord (6.04 ft)
V.deadrise     = deg2rad(21.5);  % [rad] deadrise at aft step
V.a_hydro      = deg2rad(1.45);  % [rad] step angle
V.a_trim       = deg2rad(1.5);   % [rad] design running trim
V.a_aero       = deg2rad(1.8);   % [rad] tunnel angle of attack

% CG height above the planing surface. NOT SUPPLIED -- VERIFY from CAD.
% This is the lever that converts a turn's lateral acceleration into roll load
% transfer, so it sets the whole blow-over / sponson-unloading envelope.
V.h_cg         = 0.12;       % [m] PLACEHOLDER -- FILL. See ASSUMPTIONS.md #A14.

% Lateral offset from centreline to each sponson's planing pad centre.
% = half tunnel width + half pad width. Independently equals prop_sep/2, which
% is a useful consistency check on the CAD numbers.
V.y_hull       = 0.457/2 + 0.141/2;   % [m] = 0.299

% Lateral area seen by a crosswind (tunnel side + deck + canopy).
% VERIFY from CAD -- this drives the wind disturbance magnitude directly.
V.A_lateral    = 0.30;       % [m^2] projected lateral area above waterline
V.h_cp_aero    = 0.18;       % [m]   height of aero centre of pressure above CG

%% ---- Mass properties ---------------------------------------------------
V.m_dry        = 30.6;       % [kg]  dry integrated, measured (67.4 lb)
V.m_payload    = 13.6;       % [kg]  30 lb payload
V.m            = 44.2;       % [kg]  NOMINAL loaded mass (97.4 lb)
V.g            = 9.80665;    % [m/s^2]

% Longitudinal CG, measured FORWARD OF THE TRANSOM. The battery/payload rails
% slide, so this is nominal only -- uncertainty.m sweeps 25%-35% LOA.
V.x_cg_frac    = 0.30;                  % [-] fraction of LOA fwd of transom
V.x_cg         = V.x_cg_frac * V.LOA;   % [m] = 0.640 m fwd of transom

% Yaw inertia. NOT MEASURED. Nominal uses radius of gyration k_zz = 0.25*LOA,
% a common slender-craft rule of thumb. This is the single largest unknown in
% the whole model; uncertainty.m sweeps it +/-50%. See ASSUMPTIONS.md #A1.
V.k_zz         = 0.25 * V.LOA;          % [m]  radius of gyration in yaw
V.Iz           = V.m * V.k_zz^2;        % [kg*m^2] ~= 12.6

%% ---- Propulsion layout -------------------------------------------------
V.prop_sep     = 0.598;                 % [m] lateral prop separation (VERIFY CAD)
V.y_p          = V.prop_sep / 2;        % [m] moment arm, +/- 0.299
V.D_prop       = 0.076;                 % [m] Graupner K-series 76 mm
V.n_motors     = 2;

%% ---- Reference speeds --------------------------------------------------
V.U_demo       = 13.4;       % [m/s] demonstrated planing run, no payload (30 mph)
V.U_design     = 35.8;       % [m/s] design top speed (80 mph)
V.U_plane_on   = 8.0;        % [m/s] nominal planing onset -- see hullSteadyState.m
V.aero_lift_frac_design = 0.36;  % [-] aero lift / weight at design point

end
