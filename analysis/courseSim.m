function CS = courseSim(P, NM, wpts, cases, outdir)
% COURSESIM  Full 2-mile course run: cross-track, distance, average speed.
%
%   CS = courseSim(P, NM, wpts, cases, outdir)
%
% cases is a cell array {name, rudder_override, env_override}. The default set
% compares the as-built and recommended rudders, calm and disturbed.
%
% REPORTED SEPARATELY, AND FOR THE SAME REASON AS EVERYWHERE ELSE IN THIS REPO:
%   e_straight -- cross-track on the straight legs. This is what the CONTROLLER
%                 governs, and it is the number to tune against.
%   e_corner   -- cross-track within one turn radius of a mark. This is set by
%                 the boat's own turn radius against the course's, i.e. by
%                 RUDDER SIZE, and no gain change touches it.
% A single "max cross-track error" figure conflates the two and is dominated by
% whichever mark is tightest.

if nargin < 3 || isempty(wpts), wpts = courseGeometry('oval'); end
if nargin < 5
    outdir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
end
if ~exist(outdir,'dir'), mkdir(outdir); end

ov_built = struct('R_A_r', 0.075*0.030, 'R_h_sub', 0.075);
ov_rec   = struct('R_A_r', 0.150*0.030, 'R_h_sub', 0.150);
calm     = struct('V_wind',0,   'psi_wind',0,          'V_current',0,   'psi_current',0);
rough    = struct('V_wind',7.7, 'psi_wind',deg2rad(90),'V_current',0.5,'psi_current',pi);

if nargin < 4 || isempty(cases)
    cases = { 'as-built, calm',      ov_built, calm;
              'as-built, 15 kt + current', ov_built, rough;
              'recommended, calm',   ov_rec,   calm;
              'recommended, 15 kt + current', ov_rec, rough };
end

U_ref = 8;                                   % [m/s] commanded speed
d = hypot(diff(wpts(:,1)), diff(wpts(:,2)));
L_course = sum(d);
t_sim = 1.6 * L_course / U_ref;              % generous; the run is cut at finish

n = size(cases,1);
CS.name = cases(:,1);
CS.wpts = wpts;
CS.L_course = L_course;
[CS.e_straight, CS.e_corner, CS.e_rms, CS.dist, CS.t_finish, ...
 CS.u_avg, CS.finished, CS.sat_frac, CS.vent_ct] = deal(nan(n,1));
CS.S = cell(n,1);

for i = 1:n
    Pk = buildParams(P.S, cases{i,2});
    e  = cases{i,3};
    Pk.E.V_wind = e.V_wind;  Pk.E.psi_wind = e.psi_wind;
    Pk.E.V_current = e.V_current;  Pk.E.psi_current = e.psi_current;

    cfg = struct('wpts', wpts, 'U_ref', U_ref, 'seed', 11, 'scheduled', true);
    Tt  = hullSteadyState(U_ref, Pk).R_total/2;
    S   = simOsprey([U_ref;0;0;0;0;0;Tt;Tt;0], t_sim, ...
                    makeAutopilot(Pk, NM, cfg), Pk, struct('dt', 0.02));
    CS.S{i} = S;

    % Along-track progress, to find the finish and to mask the post-finish tail.
    [prog, ecross, near] = trackProgress(S.X, S.Y, wpts, courseTurnRadius(wpts));
    % Tolerance, not an exact >=. Accumulated progress is a sum of leg lengths
    % and cannot be expected to land exactly on their total.
    kf = find(prog >= L_course - 1e-6, 1);
    if isempty(kf)
        kf = numel(S.t);  CS.finished(i) = false;
    else
        CS.finished(i) = true;
    end
    on = (1:numel(S.t)).' <= kf;

    es = abs(ecross(on & ~near));  es = es(~isnan(es));
    ec = abs(ecross(on &  near));  ec = ec(~isnan(ec));
    if isempty(es), es = NaN; end
    if isempty(ec), ec = NaN; end

    CS.e_straight(i) = max(es);
    CS.e_rms(i)      = sqrt(mean(es.^2));
    CS.e_corner(i)   = max(ec);
    CS.t_finish(i)   = S.t(kf);
    % Distance actually travelled over the ground, not course length.
    CS.dist(i)       = sum(hypot(diff(S.X(on)), diff(S.Y(on))));
    CS.u_avg(i)      = CS.dist(i) / CS.t_finish(i);
    CS.sat_frac(i)   = mean(abs(S.delta_cmd(on)) >= 0.98*min(Pk.R.delta_vent_on, Pk.R.delta_max));
    CS.vent_ct(i)    = sum(diff(S.vent(on)) > 0);
end

%% ---- Figure ------------------------------------------------------------
% 'Visible','off' so the export cannot be disturbed by desktop window activity
% (an on-screen figure was being invalidated mid-export).
fig = figure('Position',[60 60 1200 500],'Color','w','Visible','off');
subplot(1,2,1); hold on; grid on; axis equal;
plot(wpts(:,2), wpts(:,1), 'k--', 'LineWidth',1.2, 'DisplayName','course');
for i = 1:n
    S = CS.S{i};
    plot(S.Y, S.X, 'LineWidth',1.2, 'DisplayName', CS.name{i});
end
xlabel('East  [m]'); ylabel('North  [m]');
title(sprintf('2-mile course (%.0f m) -- PLACEHOLDER GEOMETRY', L_course));
legend('Location','best');

subplot(1,2,2);
b = [CS.e_straight, CS.e_corner];
bar(b); grid on;
set(gca,'XTick',1:n,'XTickLabel',CS.name,'XTickLabelRotation',20);
ylabel('Cross-track error  [m]');
% A single-case bar() makes one series, not two, so the two-entry legend would
% warn about extra entries.
if n > 1
    legend('straight legs (controller)','corners (rudder size)','Location','northwest');
end
title('Where the error comes from');

% A failed plot must never destroy the numerical results that took minutes to
% produce, so the export is non-fatal.
CS.figure = fullfile(outdir,'course_sim.png');
try
    exportgraphics(fig, CS.figure, 'Resolution', 150);
catch err
    warning('courseSim:export', 'figure export failed: %s', err.message);
    CS.figure = '';
end
close(fig);
end

% -------------------------------------------------------------------------
function R = courseTurnRadius(wpts)
% COURSETURNRADIUS  Radius around a mark inside which a sample counts as
% "in the corner" rather than "on the straight".
%
% A SMOOTH course has no corners at all. The real PEP circle is discretised
% into 48 waypoints per lap, 16.8 m apart, so a fixed 30 m mark radius would
% flag EVERY sample as "in a corner" and return NaN straight-leg error. Detect
% a smooth course from its leg-to-leg heading changes and disable the mask.
a  = atan2(diff(wpts(:,2)), diff(wpts(:,1)));
da = abs(mod(diff(a) + pi, 2*pi) - pi);
if isempty(da) || max(da) < deg2rad(20)
    R = 0;      % smooth course: no corners, everything is "straight"
    return
end
%
% This must key off the BOAT's turn radius, not the leg length. A first version
% used 2*median(leg length), which is fine on the many-short-legs oval but
% evaluates to 800 m on a triangle with 400 m legs -- masking the entire course
% as corner and returning NaN for the straight-leg error.
%
% 30 m is about 1.5x the as-built boat's ~20 m steady turn radius, so it spans
% the region where the boat is genuinely unable to follow the corner, for either
% rudder size.
R = 30;
end

% -------------------------------------------------------------------------
function [prog, e, near] = trackProgress(X, Y, wpts, R_mark)
% TRACKPROGRESS  Cumulative along-course distance, signed cross-track to the
% nearest leg, and a mask for samples close to a mark.
n = numel(X);
nl = size(wpts,1) - 1;
L  = hypot(diff(wpts(:,1)), diff(wpts(:,2)));
Lc = [0; cumsum(L)];

% SEQUENTIAL leg tracking, not nearest-leg.
%
% Nearest-leg progress is wrong on a CLOSED course and fails silently. When the
% boat comes back around to leg 1, the nearest leg is leg 1 again, so progress
% collapses to ~0; cummax then freezes it below the course length and the finish
% is never detected. The run continues past the last mark, the boat sails
% straight off the end with nothing to steer to, and the distance-to-course
% grows without bound -- which is exactly how a boat that COMPLETED the course
% came to be scored at 353 m of "cross-track error".
%
% Tracking the leg index forward, the way the guidance itself does, is immune to
% this: progress can only advance, and the finish is unambiguous.
prog = zeros(n,1);  e = zeros(n,1);  near = false(n,1);
j = 1;
for k = 1:n
    % Advance the leg index while the boat is past the end of the current leg.
    while j < nl
        p0 = wpts(j,:);  dv = wpts(j+1,:) - p0;  a = atan2(dv(2), dv(1));
        s  = (X(k)-p0(1))*cos(a) + (Y(k)-p0(2))*sin(a);
        if s > L(j), j = j + 1; else, break; end
    end
    p0 = wpts(j,:);  dv = wpts(j+1,:) - p0;  a = atan2(dv(2), dv(1));
    dp = [X(k)-p0(1), Y(k)-p0(2)];
    s  =  dp(1)*cos(a) + dp(2)*sin(a);
    ec = -dp(1)*sin(a) + dp(2)*cos(a);
    sc = min(max(s, 0), L(j));
    e(k)    = ec;
    prog(k) = Lc(j) + sc;

    % Near a mark? (interior waypoints only)
    for jj = 2:size(wpts,1)-1
        if hypot(X(k)-wpts(jj,1), Y(k)-wpts(jj,2)) < R_mark, near(k) = true; break; end
    end
end
prog = cummax(prog);
end
