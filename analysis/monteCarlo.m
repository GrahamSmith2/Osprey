function MC = monteCarlo(NM, spec, idx, MC, opts)
% MONTECARLO  Robustness of ONE fixed gain set over the full uncertainty space.
%
%   MC = monteCarlo(NM, spec, idx, MC, opts)
%   idx  vector of run indices to execute this call (so a 500-run study can be
%        done in chunks without blocking for 20 minutes)
%   MC   accumulator from a previous call, or [] to start
%
% THE POINT OF THIS STUDY
% The gains are derived from a Nomoto fit whose coefficients rest on hull
% derivatives that are uncertain by +/-5x. This asks the only question that
% matters: does ONE fixed gain set, designed at nominal, still fly the course
% when every unknown is drawn from its full declared range simultaneously?
%
% If fewer than 95% of runs meet the cross-track spec, the GAINS are wrong --
% not the boat. That is the acceptance criterion the brief set and it is used
% verbatim here.
%
% Sampling is Latin hypercube over all 15 declared uncertainties
% (params/uncertainty.m), so 500 runs actually cover the space; plain i.i.d.
% Monte Carlo leaves large voids at that count in 15 dimensions.
%
% IMPORTANT: the GAINS ARE HELD FIXED at the nominal-parameter design. Each run
% re-samples the PLANT, not the controller. Re-deriving gains per run would
% measure nothing -- it would just confirm that the design procedure works when
% you already know the answer.

if nargin < 5, opts = struct(); end
if nargin < 2 || isempty(spec)
    spec = struct('e_cross_max', 5.0, 'U_ref', 8, 't_sim', 130, ...
                  'V_wind', 7.7, 'psi_wind', deg2rad(90), ...
                  'V_current', 0.5, 'psi_current', deg2rad(180));
end
dt   = getf(opts, 'dt', 0.02);
seed = getf(opts, 'seed', 12345);
n_total = getf(opts, 'n_total', 500);

% Course. Default is ONE LAP of the real PEP circle (0.5 mile, R = 128 m).
% One lap is sufficient and four is wasteful: the circle is homogeneous, so a
% single lap already sweeps every relative wind bearing and holds the same
% steady turn throughout. Running four laps would quadruple the cost to
% re-measure the same thing.
wpts = getf(opts, 'wpts', courseGeometry('circle', 0.5*1609.344));

%% ---- Initialise the accumulator ---------------------------------------
if nargin < 4 || isempty(MC)
    MC.n_total   = n_total;
    MC.spec      = spec;
    MC.wpts      = wpts;
    MC.done      = false(1, n_total);
    MC.e_max     = nan(1, n_total);     % straight legs
    MC.e_rms     = nan(1, n_total);
    MC.e_corner  = nan(1, n_total);     % corner overshoot (rudder-size limited)
    MC.e_max_all = nan(1, n_total);
    MC.frac_course = nan(1, n_total);   % fraction of the run spent on course
    MC.pass      = false(1, n_total);
    MC.unstable  = false(1, n_total);
    MC.u_min     = nan(1, n_total);
    MC.vent_ct   = nan(1, n_total);
    MC.sat_frac  = nan(1, n_total);
    MC.tau_peak  = nan(1, n_total);
    % Optional rudder geometry override, so the SAME uncertainty draws can be
    % run against the as-built blade and against the recommended one. That
    % paired comparison is the whole point: it isolates the effect of rudder
    % size from the effect of the parameter uncertainty.
    MC.rud_override = getf(opts, 'rud_override', struct());
    MC.wc_r         = getf(opts, 'wc_r', 4.0);   % inner-loop crossover, rad/s
    % One LHS design for the whole study, drawn once so chunks are consistent.
    MC.S = sampleUncertainty('lhs', n_total, seed);
    % Gains are frozen at the NOMINAL design.
    MC.NM = NM;
end

%% ---- Run the requested indices ----------------------------------------
for i = idx(:).'
    Pk = buildParams(MC.S(i), MC.rud_override);
    Pk.E.V_wind      = spec.V_wind;
    Pk.E.psi_wind    = spec.psi_wind;
    Pk.E.V_current   = spec.V_current;
    Pk.E.psi_current = spec.psi_current;

    cfg = struct('wpts', wpts, 'U_ref', spec.U_ref, 'seed', 1000+i, ...
                 'scheduled', true, 'wc_r', MC.wc_r);
    fh  = makeAutopilot(Pk, MC.NM, cfg);       % NM fixed => gains fixed

    Tt = hullSteadyState(spec.U_ref, Pk).R_total/2;
    x0 = [spec.U_ref;0;0;0;0;0;Tt;Tt;0];

    bad = false;
    try
        S = simOsprey(x0, spec.t_sim, fh, Pk, struct('dt', dt));
    catch
        bad = true;
    end

    if bad || any(~isfinite(S.x(:)))
        MC.unstable(i) = true;  MC.pass(i) = false;  MC.done(i) = true;
        continue
    end

    % Cross-track error, recomputed from the TRUE track against the legs.
    %
    % SPLIT AT THE CORNER, deliberately. A boat with a 20 m turn radius
    % physically cannot stay within 5 m of a 39 deg corner -- the overshoot
    % there is set by turn geometry, not by the gains. Lumping it into one
    % "max cross-track" number produces a metric that fails every run for a
    % reason no gain change can fix, and hides whether the controller is
    % actually tracking. So straight-leg error (what the CONTROLLER governs)
    % and corner overshoot (what the RUDDER SIZE governs) are reported apart.
    e = crossTrackTrue(S.X, S.Y, wpts);

    % Only score samples that are actually ON the course. The run is a fixed
    % 30 s, which at 8 m/s covers 240 m against a 213 m course, so the boat
    % sails past the final waypoint and keeps going -- and distance-to-endpoint
    % then reads as tens of metres of "cross-track error" for a boat that has
    % simply finished. Scoring that was inflating the median from ~4 m to ~19 m
    % and made every configuration look equally bad.
    % Sequential leg progress, the same scheme courseSim uses. The earlier
    % first-leg/last-leg mask assumed an OPEN path and returns an empty mask on
    % a CLOSED circle, where the last leg points back at the start.
    on_course = seqOnCourse(S.X, S.Y, wpts);
    e(~on_course) = NaN;

    % A smooth course has no corners. The real circle is discretised at 16.8 m,
    % so a 25 m "near a mark" radius would flag every single sample.
    if isSmooth(wpts)
        near = false(size(e));
    else
        near = falseNearWaypoints(S.X, S.Y, wpts, 25) & on_course & ~isnan(e);
    end
    % Guard both masks: a run that diverges early may never reach the corner,
    % and one that stalls may produce no straight-leg samples at all. An empty
    % max() would otherwise assign [] and abort the whole study.
    e_str = abs(e(on_course & ~near));  e_str = e_str(~isnan(e_str));
    e_cor = abs(e(near));               e_cor = e_cor(~isnan(e_cor));
    if isempty(e_str), e_str = NaN; end
    if isempty(e_cor), e_cor = NaN; end
    MC.e_max(i)      = max(e_str);              % straight legs only
    MC.e_rms(i)      = sqrt(mean(e_str.^2));
    MC.e_corner(i)   = max(e_cor);
    e_all = abs(e(on_course));  e_all = e_all(~isnan(e_all));
    if isempty(e_all), e_all = NaN; end
    MC.e_max_all(i)  = max(e_all);
    MC.frac_course(i)= mean(on_course);
    MC.u_min(i) = min(S.u);
    MC.vent_ct(i)  = sum(diff(S.vent) > 0);
    MC.tau_peak(i) = max(S.tau_servo);

    % Fraction of the run with the rudder command pinned at its useful limit.
    % This is the discriminator between a GAIN problem and an AUTHORITY problem:
    % a badly tuned loop oscillates without saturating, whereas a boat that
    % simply cannot generate the demanded moment sits on the stop.
    d_use = min(Pk.R.delta_vent_on, Pk.R.delta_max);
    MC.sat_frac(i) = mean(abs(S.delta_cmd) >= 0.98*d_use);

    % Instability: divergence, an unrecoverable excursion, or the boat stopped.
    RE = rollEnvelope(max(mean(S.u),0.5), Pk);
    MC.unstable(i) = MC.e_max(i) > 50 || max(abs(S.r)) > 3*RE.r_max_safe || ...
                     min(S.u) < 0.5;
    MC.pass(i) = ~MC.unstable(i) && MC.e_max(i) <= spec.e_cross_max;
    MC.done(i) = true;
end

%% ---- Roll-up -----------------------------------------------------------
d = MC.done;
MC.n_done      = sum(d);
MC.frac_pass   = sum(MC.pass(d)) / max(sum(d),1);
MC.frac_unstable = sum(MC.unstable(d)) / max(sum(d),1);
MC.e_max_median  = median(MC.e_max(d & ~MC.unstable));
MC.e_max_p95     = prctileLocal(MC.e_max(d & ~MC.unstable), 95);
end

% -------------------------------------------------------------------------
function e = crossTrackTrue(X, Y, wpts)
% CROSSTRACKTRUE  Signed distance to the nearest leg, from the TRUE track.
% Measured against the geometry, not against what the guidance THOUGHT the
% error was -- the two differ by exactly the navigation error, which is part of
% what this study is meant to expose.
n = numel(X);  e = zeros(n,1);
for k = 1:n
    dmin = inf;
    for j = 1:size(wpts,1)-1
        p0 = wpts(j,:);  p1 = wpts(j+1,:);
        d  = p1 - p0;    L = hypot(d(1), d(2));
        a  = atan2(d(2), d(1));
        dp = [X(k)-p0(1), Y(k)-p0(2)];
        s  = ( dp(1)*cos(a) + dp(2)*sin(a));
        ec = (-dp(1)*sin(a) + dp(2)*cos(a));
        if s < 0,      dd = hypot(dp(1), dp(2));
        elseif s > L,  dd = hypot(X(k)-p1(1), Y(k)-p1(2));
        else,          dd = abs(ec);
        end
        if dd < dmin, dmin = dd; e(k) = sign(ec)*dd; end
    end
end
end

% -------------------------------------------------------------------------
function tf = isSmooth(wpts)
% ISSMOOTH  True if no leg-to-leg heading change exceeds 20 deg.
a  = atan2(diff(wpts(:,2)), diff(wpts(:,1)));
da = abs(mod(diff(a) + pi, 2*pi) - pi);
tf = isempty(da) || max(da) < deg2rad(20);
end

% -------------------------------------------------------------------------
function m = seqOnCourse(X, Y, wpts)
% SEQONCOURSE  True until cumulative along-course progress reaches the finish.
% Advances the leg index forward only, so it is correct on a closed circuit.
n = numel(X);  nl = size(wpts,1) - 1;
L = hypot(diff(wpts(:,1)), diff(wpts(:,2)));
Lc = [0; cumsum(L)];  Ltot = Lc(end);
prog = zeros(n,1);  j = 1;
for k = 1:n
    while j < nl
        p0 = wpts(j,:);  dv = wpts(j+1,:) - p0;  a = atan2(dv(2), dv(1));
        s = (X(k)-p0(1))*cos(a) + (Y(k)-p0(2))*sin(a);
        if s > L(j), j = j + 1; else, break; end
    end
    p0 = wpts(j,:);  dv = wpts(j+1,:) - p0;  a = atan2(dv(2), dv(1));
    s = (X(k)-p0(1))*cos(a) + (Y(k)-p0(2))*sin(a);
    prog(k) = Lc(j) + min(max(s,0), L(j));
end
prog = cummax(prog);
kf = find(prog >= Ltot - 1e-6, 1);
if isempty(kf), kf = n; end
m = (1:n).' <= kf;
end

% -------------------------------------------------------------------------
function m = onCourseMask(X, Y, wpts) %#ok<DEFNU>
% Retained for the open-path placeholder courses.
% ONCOURSEMASK  True while the boat is between the first and last waypoints,
% measured ALONG TRACK on the first and last legs. Everything after the finish
% is a boat that has completed the course, not a tracking error.
p0 = wpts(1,:);   d0 = wpts(2,:)   - p0;   a0 = atan2(d0(2), d0(1));
pE = wpts(end-1,:); dE = wpts(end,:) - pE; aE = atan2(dE(2), dE(1));
LE = hypot(dE(1), dE(2));

s_start = (X - p0(1))*cos(a0) + (Y - p0(2))*sin(a0);
s_end   = (X - pE(1))*cos(aE) + (Y - pE(2))*sin(aE);
m = (s_start >= 0) & (s_end <= LE);
end

% -------------------------------------------------------------------------
function m = falseNearWaypoints(X, Y, wpts, R)
% FALSENEARWAYPOINTS  Mask of samples within R of an INTERIOR waypoint.
m = false(numel(X),1);
for j = 2:size(wpts,1)-1
    m = m | (hypot(X - wpts(j,1), Y - wpts(j,2)) < R);
end
end

% -------------------------------------------------------------------------
function v = prctileLocal(x, p)
% PRCTILELOCAL  Percentile without the Statistics Toolbox.
x = sort(x(~isnan(x)));
if isempty(x), v = NaN; return; end
r = p/100 * numel(x);
v = x(max(min(ceil(r), numel(x)), 1));
end

function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
