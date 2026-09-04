function Sn = params_sensors()
% PARAMS_SENSORS  GPS, IMU and loop-timing characteristics.
%
% The simulation is worthless without these. A heading controller designed
% against perfect full-rate state feedback will be tuned far too aggressively:
% GPS latency alone is a 100-200 ms transport delay, which at a 2 rad/s
% crossover costs 20-25 degrees of phase margin on its own.
%
% <<FILL>> GPS model/rate and IMU noise density were not supplied. Values below
% are typical for the class of hardware this boat would use and are flagged so
% they are not mistaken for datasheet numbers.

%% ---- GPS ---------------------------------------------------------------
Sn.gps_rate    = 5;          % [Hz]  PLACEHOLDER -- FILL (u-blox default 5-10)
Sn.gps_sigma   = 1.0;        % [m]   horizontal position 1-sigma
Sn.gps_latency = 0.15;       % [s]   fix age at the time it reaches the loop
                             %       (swept 0.10-0.20 in the Monte Carlo)
Sn.gps_vel_sigma = 0.10;     % [m/s] velocity/COG solution noise

% Course over ground is only meaningful when actually moving. Below this speed
% the COG solution is dominated by position noise and must be ignored.
Sn.cog_min_speed = 1.0;      % [m/s]

%% ---- IMU ---------------------------------------------------------------
Sn.imu_rate      = 200;      % [Hz]
Sn.gyro_sigma    = 0.004;    % [rad/s] PLACEHOLDER -- FILL, white noise 1-sigma
Sn.gyro_bias_rw  = 2e-4;     % [rad/s/sqrt(s)] bias random-walk intensity
Sn.gyro_bias_0   = 0.002;    % [rad/s] initial turn-on bias

% Heading from a fused IMU/magnetometer solution.
Sn.head_sigma    = deg2rad(1.5);   % [rad] noise
Sn.head_bias     = deg2rad(2.0);   % [rad] magnetic/installation bias

%% ---- Loop timing -------------------------------------------------------
Sn.f_ctrl     = 50;          % [Hz] controller rate (ZOH)
Sn.delay_n    = 1;           % [-]  compute delay, in controller ticks

end
