function [meas, sens] = sensorModel(t, x, sens, P)
% SENSORMODEL  GPS + IMU measurements from the true state.
%
%   [meas, sens] = sensorModel(t, x, sens, P)
%   sens is the sensor memory; initialise with sensorModel('init', seed, [], P).
%
% THE ONE THAT BITES: HEADING IS NOT COURSE OVER GROUND
% The IMU/magnetometer measures HEADING psi -- where the bow points.
% GPS measures COURSE OVER GROUND chi -- where the boat is actually going.
% In any current or crosswind these differ by the crab angle:
%
%       chi = psi + crab
%
% A heading controller must close on psi. A CROSS-TRACK controller must close
% on chi, because the boat's ground track is what has to lie on the line. Feed
% psi to a cross-track loop in a 1.5 m/s current and it will hold the bow on
% the bearing while the boat sails steadily downstream of the track, with the
% integrator winding up against a disturbance it cannot see. That failure is
% silent -- the heading error reads zero the whole time.
%
% Both are returned separately here so the controller has to choose explicitly.

%% ---- Initialisation ----------------------------------------------------
if ischar(t) && strcmp(t, 'init')
    seed = x;
    sens = struct();
    sens.rs        = RandStream('mt19937ar', 'Seed', seed);
    sens.Sn        = params_sensors();
    sens.gyro_bias = sens.Sn.gyro_bias_0;
    sens.t_last_gps= -inf;
    sens.gps_buf   = [];        % [t, X, Y, chi, speed] history for latency
    sens.gps_hold  = [0 0 0 0]; % last delivered fix: X, Y, chi, speed
    sens.t_prev    = 0;
    meas = struct();
    return
end

Sn = sens.Sn;
dt = max(t - sens.t_prev, 0);
sens.t_prev = t;

X = x(4); Y = x(5); psi = x(6);
u = x(1); v = x(2); r = x(3);

%% ---- IMU ---------------------------------------------------------------
% Gyro bias is a random walk: variance grows linearly in time, so the increment
% scales as sqrt(dt). Using a fixed per-step increment instead would make the
% drift rate depend on the step size, which is a classic way to get a sensor
% model that behaves differently at every integrator setting.
sens.gyro_bias = sens.gyro_bias + Sn.gyro_bias_rw * sqrt(dt) * randn(sens.rs);

meas.r   = r + sens.gyro_bias + Sn.gyro_sigma * randn(sens.rs);
meas.psi = wrapPi(psi + Sn.head_bias + Sn.head_sigma * randn(sens.rs));

%% ---- GPS ---------------------------------------------------------------
% True ground track, used to build the buffer.
c = cos(psi); s = sin(psi);
Xdot = u*c - v*s;  Ydot = u*s + v*c;
spd  = hypot(Xdot, Ydot);
chi  = atan2(Ydot, Xdot);

sens.gps_buf = [sens.gps_buf; t, X, Y, chi, spd];
if size(sens.gps_buf,1) > 2000, sens.gps_buf(1,:) = []; end

meas.gps_new = false;
if t - sens.t_last_gps >= 1/Sn.gps_rate - 1e-9
    sens.t_last_gps = t;

    % LATENCY: the fix that arrives NOW describes where the boat WAS.
    t_fix = t - Sn.gps_latency;
    b = sens.gps_buf;
    if t_fix <= b(1,1)
        row = b(1,:);
    else
        row = [interp1(b(:,1), b(:,2), t_fix), ...
               interp1(b(:,1), b(:,3), t_fix), ...
               interp1(b(:,1), unwrap(b(:,4)), t_fix), ...
               interp1(b(:,1), b(:,5), t_fix)];
        row = [t_fix, row];
    end

    Xg   = row(2) + Sn.gps_sigma * randn(sens.rs);
    Yg   = row(3) + Sn.gps_sigma * randn(sens.rs);
    spdg = max(row(5) + Sn.gps_vel_sigma * randn(sens.rs), 0);

    % COG is only trustworthy above a minimum speed. Below it, the solution is
    % position noise divided by a small displacement and points anywhere.
    if spdg > Sn.cog_min_speed
        chig = wrapPi(row(4) + Sn.gps_vel_sigma/max(spdg,0.1) * randn(sens.rs));
        cog_valid = true;
    else
        chig = sens.gps_hold(3);
        cog_valid = false;
    end

    sens.gps_hold = [Xg, Yg, chig, spdg];
    meas.gps_new  = true;
    meas.cog_valid= cog_valid;
end

% Zero-order hold between fixes -- the controller sees a staircase, not a curve.
meas.X    = sens.gps_hold(1);
meas.Y    = sens.gps_hold(2);
meas.chi  = sens.gps_hold(3);          % course over ground
meas.spd  = sens.gps_hold(4);          % speed over ground
if ~isfield(meas,'cog_valid'), meas.cog_valid = sens.gps_hold(4) > Sn.cog_min_speed; end

% Speed used for GAIN SCHEDULING. Scheduling on a noisy 5 Hz staircase would
% inject that noise straight into the gains, so it is low-pass filtered.
if ~isfield(sens, 'spd_f'), sens.spd_f = meas.spd; end
a = min(dt / 0.5, 1);                   % 0.5 s time constant
sens.spd_f = sens.spd_f + a * (meas.spd - sens.spd_f);
meas.spd_sched = max(sens.spd_f, 0.5);  % floor keeps 1/U gains finite

meas.crab = wrapPi(meas.chi - meas.psi);
meas.t = t;
end

% -------------------------------------------------------------------------
function a = wrapPi(a)
a = mod(a + pi, 2*pi) - pi;
end
