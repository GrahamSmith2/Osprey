function LA = lapAccumulation(NM, idx, LA, opts)
% LAPACCUMULATION  Does track error grow from lap 1 to lap 4?
%
%   LA = lapAccumulation(NM, idx, LA, opts)
%
% WHY THIS IS A SEPARATE STUDY
% The Monte Carlo runs ONE lap of the circle, on the argument that the course is
% homogeneous so one lap sweeps every relative wind bearing. That argument is
% sound for anything quasi-steady, but it is blind to anything that ACCUMULATES:
%
%   - heading-integrator drift against a standing disturbance,
%   - gyro bias random walk (variance grows linearly in time),
%   - a slow radial bias from the crab-angle correction being imperfect,
%   - GPS position noise integrating into the guidance solution.
%
% All of those would be invisible on lap 1 and obvious on lap 4. The race is
% four laps, so the question has to be asked directly.
%
% Per-lap error statistics are recorded within each run, so the trend is visible
% from relatively few draws -- accumulation is a within-run effect, not a
% between-run one.

if nargin < 4, opts = struct(); end
n_total = getf(opts, 'n_total', 12);
seed    = getf(opts, 'seed', 12345);
dt      = getf(opts, 'dt', 0.02);
rud     = getf(opts, 'rud_override', struct());

wpts = courseGeometry('circle');                 % full 4 laps
ci   = struct(); [~, ci] = courseGeometry('circle');
Rc   = ci.radius;
Cc   = [0, Rc];                                  % circle centre
n_lap = ci.n_laps;
U_ref = 8;

if nargin < 3 || isempty(LA)
    LA.n_total = n_total;
    LA.S = sampleUncertainty('lhs', n_total, seed);
    LA.rud = rud;
    LA.done     = false(1, n_total);
    LA.lap_max  = nan(n_total, n_lap);
    LA.lap_rms  = nan(n_total, n_lap);
    LA.lap_bias = nan(n_total, n_lap);   % mean SIGNED radial error per lap
    LA.finished = false(1, n_total);
    LA.n_lap    = n_lap;
end

for i = idx(:).'
    Pk = buildParams(LA.S(i), LA.rud);
    Pk.E.V_wind = 7.7;  Pk.E.psi_wind = deg2rad(90);
    Pk.E.V_current = 0.5;  Pk.E.psi_current = pi;

    cfg = struct('wpts', wpts, 'U_ref', U_ref, 'seed', 1000+i, 'scheduled', true);
    Tt  = hullSteadyState(U_ref, Pk).R_total/2;
    try
        S = simOsprey([U_ref;0;0;0;0;0;Tt;Tt;0], 1.4*ci.length/U_ref, ...
                      makeAutopilot(Pk, NM, cfg), Pk, struct('dt', dt));
    catch
        LA.done(i) = true;  continue
    end
    if any(~isfinite(S.x(:))), LA.done(i) = true; continue, end

    % Radial error against the TRUE circle, not the inscribed polygon -- the
    % polygon is 0.14 m inside the circle and that would bias every lap alike.
    rad = hypot(S.X - Cc(1), S.Y - Cc(2));
    e   = rad - Rc;                               % signed: + is outside

    % Assign samples to laps by unwrapped angle about the centre.
    th  = unwrap(atan2(S.Y - Cc(2), S.X - Cc(1)));
    th  = th - th(1);
    lap = floor(abs(th) / (2*pi)) + 1;

    for L = 1:n_lap
        m = (lap == L);
        if any(m)
            LA.lap_max(i,L)  = max(abs(e(m)));
            LA.lap_rms(i,L)  = sqrt(mean(e(m).^2));
            LA.lap_bias(i,L) = mean(e(m));
        end
    end
    LA.finished(i) = max(abs(th)) >= 2*pi*n_lap - 0.1;
    LA.done(i) = true;
end

d = LA.done;
LA.n_done = sum(d);
LA.med_max  = median(LA.lap_max(d,:), 1, 'omitnan');
LA.med_rms  = median(LA.lap_rms(d,:), 1, 'omitnan');
LA.med_bias = median(LA.lap_bias(d,:), 1, 'omitnan');
% Growth from lap 1 to lap 4, as a ratio of the medians.
LA.growth_max = LA.med_max(end) / LA.med_max(1);
LA.growth_rms = LA.med_rms(end) / LA.med_rms(1);
end

function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
