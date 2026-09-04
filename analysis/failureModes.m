function FM = failureModes(P, NM, outdir)
% FAILUREMODES  Does the boat survive the four faults the brief asks about?
%
%   FM = failureModes(P, NM, outdir)
%
% Each run is the same two-leg course, same gains, same disturbances, with one
% fault injected at t = 12 s -- mid-course, on the straight leg, so the boat is
% settled and tracking when it happens rather than still capturing the line.
%
% PLANT faults are injected in simOsprey and the controller is NOT told about
% them; the SENSOR fault is injected in makeAutopilot. That separation is the
% point of the exercise -- a fault the controller is warned about in advance is
% not a fault, it is a mode change.
%
% Faults:
%   1. Rudder jam at 10 deg     -- the linkage fails with the blade deflected
%   2. One motor out            -- can differential thrust from the REMAINING
%                                  motor plus rudder still hold heading?
%   3. GPS dropout for 5 s      -- IMU survives, so heading holds; cross-track
%                                  is what degrades
%   4. Ventilation mid-turn     -- latched 50% authority loss that does not
%                                  recover

if nargin < 3
    outdir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
end
if ~exist(outdir,'dir'), mkdir(outdir); end

wpts  = [0 0; 100 0; 180 80];
U_ref = 8;
t_sim = 32;
% Fault at 7 s puts it mid-way along the FIRST STRAIGHT LEG (X ~ 55 m), with the
% boat settled and tracking. Injecting at 12 s -- as a first attempt did -- lands
% it exactly on the corner at (100,0), so the corner transient and the fault
% response are superimposed and neither can be read.
t_f   = 7;

% Run the faults against the RECOMMENDED rudder -- there is no point asking
% whether a blade already known to be undersized survives a fault.
Pk = buildParams(P.S, struct('R_A_r', 0.150*0.030, 'R_h_sub', 0.150));
Pk.E.V_wind    = 7.7;  Pk.E.psi_wind    = deg2rad(90);
Pk.E.V_current = 0.5;  Pk.E.psi_current = pi;

cases = { ...
 'baseline (no fault)', struct('type','none'),                              [];
 'rudder jam @10 deg',  struct('type','rudder_jam','t0',t_f,'value',deg2rad(10)), [];
 'port motor out',      struct('type','motor_out','t0',t_f,'value',1),      [];
 'GPS dropout 5 s',     struct('type','none'),                              [t_f t_f+5];
 'ventilation latched', struct('type','ventilation','t0',t_f),              [] };

n = size(cases,1);
FM.name = cases(:,1);
FM.S = cell(n,1);
[FM.e_max_after, FM.psi_drift, FM.recovered, FM.u_end, FM.finished] = deal(nan(n,1));

for i = 1:n
    cfg = struct('wpts', wpts, 'U_ref', U_ref, 'seed', 42, 'scheduled', true);
    if ~isempty(cases{i,3}), cfg.gps_dropout = cases{i,3}; end

    Tt = hullSteadyState(U_ref, Pk).R_total/2;
    x0 = [U_ref;0;0;0;0;0;Tt;Tt;0];
    S  = simOsprey(x0, t_sim, makeAutopilot(Pk, NM, cfg), Pk, ...
                   struct('dt', 0.02, 'fault', cases{i,2}));
    FM.S{i} = S;

    % Score only while the boat is still ON the course. A 32 s run at 8 m/s
    % covers 256 m of a 213 m course, so a boat that simply finishes would
    % otherwise be charged tens of metres of "cross-track error" for sailing
    % past the last mark.
    e  = crossTrackSimple(S.X, S.Y, wpts);
    on = onCourse(S.X, S.Y, wpts);
    after = (S.t >= t_f) & on;
    if ~any(after), after = on; end
    FM.e_max_after(i) = max(abs(e(after)));
    FM.u_end(i)       = S.u(end);
    FM.finished(i)    = S.X(end) > 150;

    % Heading drift over the 5 s following the fault -- the direct measure of
    % whether the boat held its course or departed.
    j0 = find(S.t >= t_f, 1);  j1 = find(S.t >= t_f+5, 1);
    if isempty(j1), j1 = numel(S.t); end
    FM.psi_drift(i) = rad2deg(abs(wrapPiL(S.psi(j1) - S.psi(j0))));

    % "Recovered" = back inside 5 m of track, and still moving, over the last
    % on-course stretch before the finish.
    tail = on & (S.t >= t_f + 8);
    if any(tail)
        FM.recovered(i) = max(abs(e(tail))) < 5 && S.u(end) > 1;
    else
        FM.recovered(i) = false;
    end
end

%% ---- Rudder-jam recovery: the actionable part --------------------------
% A jammed rudder produces a moment that grows as u^2. Differential thrust
% produces one that is nearly independent of speed (it falls only slowly, with
% advance ratio). So the two cross, and SLOWING DOWN is the recovery action --
% it is the only lever that shifts authority back toward the working actuator.
FM.jam_u    = [8 6 5 4 3];
FM.jam_emax = nan(size(FM.jam_u));
FM.jam_N_rud = nan(size(FM.jam_u));
FM.jam_N_thr = nan(size(FM.jam_u));
jam = struct('type','rudder_jam','t0',t_f,'value',deg2rad(10));
for i = 1:numel(FM.jam_u)
    Ur = FM.jam_u(i);
    FM.jam_N_rud(i) = rudderForces(deg2rad(10), Ur, 0, 0, 0, Pk).N;
    FM.jam_N_thr(i) = 2 * propMaxThrust(Ur, Pk) * Pk.V.y_p;
    Tt = hullSteadyState(Ur, Pk).R_total/2;
    cfg = struct('wpts',wpts,'U_ref',Ur,'seed',42,'scheduled',true);
    S = simOsprey([Ur;0;0;0;0;0;Tt;Tt;0], 90, makeAutopilot(Pk,NM,cfg), Pk, ...
                  struct('dt',0.02,'fault',jam));
    e = crossTrackSimple(S.X, S.Y, wpts);
    on = onCourse(S.X, S.Y, wpts) & (S.t > t_f);
    if any(on), FM.jam_emax(i) = max(abs(e(on))); end
end
% Speed below which differential thrust keeps a 1.5x margin over the jam.
g  = @(u) 2*propMaxThrust(u,Pk)*Pk.V.y_p - 1.5*rudderForces(deg2rad(10),u,0,0,0,Pk).N;
us = linspace(1,12,300); gv = arrayfun(g, us);
j  = find(gv(1:end-1) > 0 & gv(2:end) <= 0, 1);
if isempty(j), FM.jam_u_safe = NaN; else, FM.jam_u_safe = us(j); end

%% ---- Figure ------------------------------------------------------------
fig = figure('Position',[80 80 1180 460],'Color','w');
subplot(1,2,1); hold on; grid on;
plot(wpts(:,2), wpts(:,1), 'k--o', 'LineWidth',1.5, 'DisplayName','course');
for i = 1:n
    S = FM.S{i};
    plot(S.Y, S.X, 'LineWidth',1.4, 'DisplayName', FM.name{i});
end
xlabel('East  [m]'); ylabel('North  [m]'); axis equal;
title(sprintf('Failure modes, fault at t = %g s', t_f)); legend('Location','northwest');

subplot(1,2,2); hold on; grid on;
for i = 1:n
    S = FM.S{i};
    plot(S.t, rad2deg(S.delta_blade), 'LineWidth',1.3, 'DisplayName', FM.name{i});
end
xline(t_f,'k:','fault');
yline( rad2deg(Pk.R.delta_vent_on),'r--');
yline(-rad2deg(Pk.R.delta_vent_on),'r--');
xlabel('Time  [s]'); ylabel('Blade angle  [deg]');
title('Actuator response (red = ventilation clamp)');

exportgraphics(fig, fullfile(outdir,'failure_modes.png'), 'Resolution', 150);
FM.figure = fullfile(outdir,'failure_modes.png');
end

% -------------------------------------------------------------------------
function e = crossTrackSimple(X, Y, wpts)
n = numel(X); e = zeros(n,1);
for k = 1:n
    dmin = inf;
    for j = 1:size(wpts,1)-1
        p0 = wpts(j,:); p1 = wpts(j+1,:);
        d = p1-p0; L = hypot(d(1),d(2)); a = atan2(d(2),d(1));
        dp = [X(k)-p0(1), Y(k)-p0(2)];
        s  = dp(1)*cos(a)+dp(2)*sin(a);
        ec = -dp(1)*sin(a)+dp(2)*cos(a);
        if s < 0, dd = hypot(dp(1),dp(2));
        elseif s > L, dd = hypot(X(k)-p1(1), Y(k)-p1(2));
        else, dd = abs(ec); end
        if dd < dmin, dmin = dd; e(k) = sign(ec)*dd; end
    end
end
end

function m = onCourse(X, Y, wpts)
% ONCOURSE  True between the start and the finish, measured along track.
p0 = wpts(1,:);     d0 = wpts(2,:)-p0;     a0 = atan2(d0(2),d0(1));
pE = wpts(end-1,:); dE = wpts(end,:)-pE;   aE = atan2(dE(2),dE(1));
LE = hypot(dE(1),dE(2));
s0 = (X-p0(1))*cos(a0) + (Y-p0(2))*sin(a0);
sE = (X-pE(1))*cos(aE) + (Y-pE(2))*sin(aE);
m  = (s0 >= 0) & (sE <= LE);
end

function a = wrapPiL(a)
a = mod(a + pi, 2*pi) - pi;
end
