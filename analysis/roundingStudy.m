function RS = roundingStudy(P, NM, radii, rudders, outdir)
% ROUNDINGSTUDY  Optimal mark-rounding radius, and whether rudder size moves it.
%
%   RS = roundingStudy(P, NM, radii, rudders, outdir)
%
% THE QUESTION
% On a two-mark oval the rounding radius is a STRATEGY CHOICE, not a course
% constraint. It trades two things against each other:
%
%   round WIDE  -> longer path per lap, but the boat holds its speed
%   round TIGHT -> shorter path, but the rudder saturates, the blade
%                  ventilates, and the boat bleeds up to ~21% of its speed
%
% Both effects hit TIME TO COVER TWO MILES, which is what the 20-point
% placement score depends on. (The 40-point distance score only cares that the
% distance is completed at all.) So there is an optimum, and the whole point of
% this study is whether a bigger rudder moves it -- and whether it is worth
% anything in lap time, as opposed to the disturbance-rejection margin argument
% made elsewhere in this repo.
%
% METRIC: time to cover the full 2-mile race distance, simulated, including the
% speed lost in every rounding. NOT time per lap -- a tighter-rounding boat runs
% a shorter lap and therefore needs MORE laps for the same distance, and that
% has to be counted or the comparison is rigged.

if nargin < 3 || isempty(radii),   radii = [12 16 20 25 30 40 60]; end
if nargin < 4 || isempty(rudders)
    rudders = { 'as-built 75mm',      struct('R_A_r',0.075*0.030,'R_h_sub',0.075);
                'recommended 150mm',  struct('R_A_r',0.150*0.030,'R_h_sub',0.150) };
end
if nargin < 5
    outdir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
end
if ~exist(outdir,'dir'), mkdir(outdir); end

C     = params_competition();
D     = 0.25 * 1609.344;            % [m] mark separation, 0.25 mile
U_ref = 8;                          % [m/s] commanded speed
race  = C.race_distance_m;          % [m] 2 miles

nr = numel(radii);  nk = size(rudders,1);
z  = nan(nk, nr);
RS = struct('radii',radii,'rudders',{rudders(:,1)},'separation',D, ...
            't_race',z,'u_avg',z,'lap_len',z,'n_laps',z, ...
            'u_min_turn',z,'sat_frac',z,'vent_ct',z,'e_track',z,'feasible',z);

for k = 1:nk
    Pk = buildParams(P.S, rudders{k,2});
    Pk.E.V_wind    = 7.7;  Pk.E.psi_wind    = deg2rad(90);
    Pk.E.V_current = 0.5;  Pk.E.psi_current = pi;

    for j = 1:nr
        Rt = radii(j);
        [wpts, ci] = courseGeometry('marks', D, Rt);

        % Enough laps to cover the full race distance.
        nlap = ceil(race / ci.lap_len);
        W = wpts;
        for L = 2:nlap, W = [W; wpts]; end          %#ok<AGROW>

        cfg = struct('wpts', W, 'U_ref', U_ref, 'seed', 11, 'scheduled', true);
        Tt  = hullSteadyState(U_ref, Pk).R_total/2;
        t_max = 2.2 * race / U_ref;
        S = simOsprey([U_ref;0;0;0;0;0;Tt;Tt;0], t_max, ...
                      makeAutopilot(Pk, NM, cfg), Pk, struct('dt', 0.02));

        % Distance over ground, and the time at which 2 miles is covered.
        dist = [0; cumsum(hypot(diff(S.X), diff(S.Y)))];
        kf = find(dist >= race, 1);
        if isempty(kf)
            RS.feasible(k,j) = 0;                    % never covered the distance
            continue
        end
        RS.feasible(k,j) = 1;
        on = (1:numel(S.t)).' <= kf;

        RS.t_race(k,j)    = S.t(kf);
        RS.u_avg(k,j)     = race / S.t(kf);
        RS.lap_len(k,j)   = ci.lap_len;
        RS.n_laps(k,j)    = race / ci.lap_len;
        RS.u_min_turn(k,j)= min(S.u(on));
        d_use = min(Pk.R.delta_vent_on, Pk.R.delta_max);
        RS.sat_frac(k,j)  = mean(abs(S.delta_cmd(on)) >= 0.98*d_use);
        RS.vent_ct(k,j)   = sum(diff(S.vent(on)) > 0);

        % Track error against the commanded path.
        RS.e_track(k,j)   = maxTrackError(S.X(on), S.Y(on), wpts);
    end
end

% Best radius per rudder, by race time.
RS.best_radius = nan(nk,1);  RS.best_time = nan(nk,1);
for k = 1:nk
    [RS.best_time(k), j] = min(RS.t_race(k,:));
    if isfinite(RS.best_time(k)), RS.best_radius(k) = radii(j); end
end
end

% -------------------------------------------------------------------------
function em = maxTrackError(X, Y, wpts)
n = numel(X);  e = zeros(n,1);
for i = 1:n
    dmin = inf;
    for j = 1:size(wpts,1)-1
        p0 = wpts(j,:); p1 = wpts(j+1,:); dv = p1-p0;
        L = hypot(dv(1),dv(2));  if L < 1e-9, continue; end
        a = atan2(dv(2),dv(1));  dp = [X(i)-p0(1), Y(i)-p0(2)];
        s = min(max(dp(1)*cos(a)+dp(2)*sin(a), 0), L);
        dd = hypot(X(i)-(p0(1)+s*cos(a)), Y(i)-(p0(2)+s*sin(a)));
        if dd < dmin, dmin = dd; end
    end
    e(i) = dmin;
end
em = max(e);
end
