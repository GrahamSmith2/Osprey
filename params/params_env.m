function E = params_env()
% PARAMS_ENV  Fluid properties and environmental disturbance defaults.
%
% Water density is brackish per the design report. Vapour pressure and
% atmospheric pressure are needed for the cavitation number in the rudder model.
% All SI.

%% ---- Water -------------------------------------------------------------
E.rho_w   = 1010;        % [kg/m^3] brackish water (design report)
E.nu_w    = 1.05e-6;     % [m^2/s]  kinematic viscosity, ~20 C brackish
E.p_atm   = 101325;       % [Pa]     atmospheric pressure at surface
E.p_vap   = 2339;         % [Pa]     water vapour pressure at 20 C
E.g       = 9.80665;      % [m/s^2]

%% ---- Air ---------------------------------------------------------------
E.rho_a   = 1.225;        % [kg/m^3] sea-level standard

%% ---- Disturbance defaults (overridden per-run by the analysis scripts) --
E.V_current     = 0.0;              % [m/s] steady current magnitude
E.psi_current   = 0.0;              % [rad] current SET (direction flow goes TO)
E.V_wind        = 0.0;              % [m/s] true wind speed
E.psi_wind      = 0.0;              % [rad] wind direction FROM
E.wave_N_amp    = 0.0;              % [N*m] band-limited yaw moment amplitude
E.wave_f_lo     = 0.2;              % [Hz]  wave yaw-moment band, low edge
E.wave_f_hi     = 1.5;              % [Hz]  wave yaw-moment band, high edge

% Aero side-force coefficient slope for the hull+canopy in sideslip. Placeholder
% flat-plate-ish value; the sensitivity of the cross-track result to this is
% reported rather than the value being trusted. See ASSUMPTIONS.md #A7.
E.CY_beta   = 1.2;      % [1/rad] dY_aero/dbeta_wind, linear region
E.x_cp_aero = 0.25;     % [-] aero CoP as fraction of LOA FWD of CG (destabilising)

end
