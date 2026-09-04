function R = params_rudder()
% PARAMS_RUDDER  Baseline (as-built) rudder geometry -- MHZ Mystic C5000.
%
% This describes the EXISTING rudder. The sizing sweep in analysis/ overrides
% A_r and h_sub; nothing else in this file changes during the sweep.
%
% Sign convention: x_r is the rudder stock position POSITIVE FORWARD of the CG.
% The rudder is at the transom, so x_r = -x_cg (negative, i.e. aft). It is
% computed in model/rudderForces.m from the CURRENT x_cg, because x_cg slides
% on the payload rails -- see uncertainty.m.
%
% Positive rudder deflection delta > 0 is defined to produce a POSITIVE yaw
% moment (turn to starboard). Sign bookkeeping is done once, in rudderForces.m.

%% ---- Baseline blade geometry ------------------------------------------
R.span_total = 0.150;        % [m]  full blade span
R.sub_frac   = 0.50;         % [-]  fraction submerged at planing trim (VERIFY)
R.h_sub      = R.span_total * R.sub_frac;  % [m] = 0.075 submerged span
R.c          = 0.030;        % [m]  chord
R.A_r        = R.h_sub * R.c;              % [m^2] = 2.25e-3 wetted area
R.AR_geom    = R.h_sub / R.c;              % [-]   = 2.5 geometric aspect ratio

%% ---- Hydrodynamic coefficients ----------------------------------------
% Free-surface image factor. A DEEPLY SUBMERGED foil with a solid end plate
% behaves like a wing of ~2x its geometric aspect ratio. A SURFACE-PIERCING
% rudder that ventilates gets NO image effect -- the free surface is a
% pressure-release boundary, not a reflecting plane. Nominal is therefore 1.0,
% NOT 2.0. Swept 1.0->2.0 in uncertainty.m. See ASSUMPTIONS.md #A4.
R.k_surface  = 1.0;          % [-]  AR_eff = k_surface * h_sub/c

R.CD0        = 0.0085;       % [-]  section profile drag at zero lift
R.e_oswald   = 0.85;         % [-]  span efficiency, low-AR plate

R.delta_max  = deg2rad(35);  % [rad] mechanical stop
R.x_cp_frac  = 0.25;         % [-]  centre of pressure, fraction of chord aft of LE
R.x_stock_frac = 0.25;       % [-]  stock axis location, fraction of chord aft of LE
                             %      (balanced at 25% -> near-zero nominal hinge
                             %      moment. VERIFY on the real blade: if the
                             %      stock is at the LE this number explodes.)

%% ---- Ventilation model ------------------------------------------------
% Surface-piercing rudders entrain air down the suction side past a critical
% angle. Lift does not recover until the angle drops well below inception
% (hysteresis), because the ventilated cavity is self-sustaining.
R.delta_vent_on  = deg2rad(10);  % [rad] inception angle (swept 6-15 deg)
R.vent_hysteresis = deg2rad(4);  % [rad] re-wetting happens this far below onset
R.k_vent_loss    = 0.50;         % [-]   CL multiplier once ventilated
R.vent_blend     = deg2rad(1.5); % [rad] smoothing width, keeps the ODE integrable

end
