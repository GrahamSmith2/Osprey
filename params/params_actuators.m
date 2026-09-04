function A = params_actuators()
% PARAMS_ACTUATORS  Servo, steering linkage, and ESC/motor/prop dynamics.
%
% The servo is NOT modelled as ideal. The chain implemented in
% model/servoDynamics.m is:
%   delta_cmd -> rate limit -> 1st-order lag -> backlash -> saturation -> delta
% Rate limit and backlash are what will actually cap the achievable Kd.

%% ---- Servo: AGFRC A81FHM HV -------------------------------------------
A.tau_stall_kgcm = 74;                       % [kg*cm] datasheet, 8.4 V
A.tau_stall      = A.tau_stall_kgcm * 0.0980665;  % [N*m] = 7.257

% <<FILL>> No-load speed. Datasheet value not supplied. Nominal below is a
% placeholder for a high-torque HV digital servo; analysis/ sweeps it and
% reports the required value rather than assuming this one is right.
A.t60_noload   = 0.10;                       % [s/60deg] PLACEHOLDER -- FILL
A.rate_max     = deg2rad(60) / A.t60_noload; % [rad/s] ~10.5 rad/s = 600 deg/s

% Under load the servo slows roughly in proportion to remaining torque margin.
A.rate_load_knockdown = true;   % scale rate by (1 - M_h/tau_stall)

A.tau_servo_lag = 0.040;        % [s] closed-loop electrical/mech lag, 30-60 ms

% <<FILL>> Deadband / resolution and PWM command rate not supplied.
A.deadband     = deg2rad(0.3);  % [rad] PLACEHOLDER -- FILL from datasheet
A.f_pwm        = 333;           % [Hz]  PLACEHOLDER -- FILL (HV digital typ. 333)

%% ---- Steering linkage: pull-pull cable --------------------------------
A.r_servo_horn = 0.015;         % [m] servo horn radius (given)

% <<FILL>> Rudder tiller arm radius NOT supplied. If it equals the servo horn
% the linkage is 1:1 and there is no torque advantage. This ratio scales the
% required servo torque LINEARLY, so it is a first-order sizing input, not a
% detail. VERIFY from the boat. Swept in uncertainty.m.
A.r_rudder_arm = 0.015;         % [m] PLACEHOLDER -- assumed 1:1 -- FILL
A.eta_cable    = 0.85;          % [-] pull-pull cable efficiency (friction)
A.backlash     = deg2rad(0.5);  % [rad] free play at the rudder (swept 0-2 deg)

%% ---- ESC / motor / prop ------------------------------------------------
A.tau_thrust   = 0.15;          % [s] first-order spool-up lag (swept 0.1-0.3)
A.V_batt       = 44.4;          % [V] nominal 12S
A.Kv           = 680;           % [rpm/V] Castle 2535
A.n_max        = 23150/60;      % [rev/s] logged planing peak RPM
A.I_peak       = 45;            % [A] logged peak, per motor

% KT(J) map: KT = KT0*(1 - J/J0). KT0 is CALIBRATED in propMaxThrust against
% the logged top-speed run, so only J0 (the zero-thrust advance ratio, which is
% essentially a pitch/diameter proxy) is assumed.
%
% <<FILL>> Graupner K-series 76 mm open-water data not supplied.
%
% J0 IS LOAD-BEARING, so it is stated rather than buried. At the logged
% 23,150 rpm the boat cannot exceed u = J0*n*D. A generic J0 = 0.9 caps the
% boat at 26.4 m/s, which would make the 35.8 m/s design speed unreachable --
% so a generic value from the wrong class of propeller is not admissible here.
% Reaching 35.8 m/s requires J0 >= 1.22 (P/D ~ 1.4), which is ordinary for a
% high-speed surface-piercing race prop. Nominal 1.35 leaves ~10% margin.
% If the real prop is lower-pitched, the top speed claim goes with it.
A.J0_prop = 1.35;               % [-] zero-thrust advance ratio -- FILL from data

end
