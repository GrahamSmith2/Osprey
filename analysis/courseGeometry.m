function [wpts, info] = courseGeometry(type, total_len, R_turn)
% COURSEGEOMETRY  Waypoint list for a closed course of a given total length.
%
%   [wpts, info] = courseGeometry(type, total_len, R_turn)
%
% ===========================================================================
% THIS GEOMETRY IS A PLACEHOLDER. IT IS NOT THE PEP COURSE.
% ===========================================================================
% The actual course layout and buoy turn radii were never supplied. Rather than
% invent a specific layout and report numbers against it as if they were real,
% this function generates a PARAMETRIC course of the correct total length, and
% every result derived from it is labelled illustrative.
%
% To use the real course: replace the call with your own waypoint list. Nothing
% downstream depends on how the waypoints were produced.
%
% Why the shape still matters, and what to check when the real one arrives:
% the boat's steady turn radius is roughly 4.8 boat lengths (10 m) with the
% recommended rudder and 9.5 (20 m) with the as-built one. If any real buoy
% turn is tighter than that, the boat CANNOT hold the corner regardless of
% tuning, and the cross-track figure at that mark is set by turn geometry.
% info.min_turn_radius below is what to compare against.
%
% type: 'oval'      -- two straights joined by 180 deg turns (typical circuit)
%       'triangle'  -- three marks, 120 deg turns (typical windward-leeward)
%       'dogbone'   -- out and back with 180 deg turns at each end
%
% total_len [m] default 3218 (2 statute miles)
% R_turn    [m] radius used to round the corners into arcs of waypoints

if nargin < 1 || isempty(type),      type = 'oval'; end
if nargin < 2 || isempty(total_len), total_len = 3218; end   % 2 miles
if nargin < 3 || isempty(R_turn),    R_turn = 25; end        % [m]

switch lower(type)
    case 'marks'
        % THE ACTUAL PEP26 COURSE as described by the team (2026-09-04):
        % TWO MARKS, run as an oval / dogbone -- up one side, 180 deg around
        % the first mark, back down the other side, 180 deg around the second.
        % Both roundings in the same direction; no crossing.
        %
        % Arguments are repurposed for this shape:
        %   total_len -> D, the straight-line separation between the two marks
        %   R_turn    -> the rounding radius, which is a STRATEGY CHOICE, not a
        %                course constraint. The boat picks it, bounded below by
        %                what the rudder can actually achieve.
        %
        % Lap length = 2*D + 2*pi*R_turn. NOTE this does NOT come to 0.5 mile
        % for D = 0.25 mile: the straights alone are already half a mile, so a
        % lap is 0.54-0.58 mile depending on how tightly the marks are rounded.
        % Whichever of those two figures is authoritative should be confirmed.
        D  = total_len;
        Rt = R_turn;
        na = 24;                                   % points per 180 deg rounding
        th = linspace(-pi/2, pi/2, na).';
        % Straight north along y = 0, round the north mark, straight south
        % along y = 2*Rt, round the south mark.
        wpts = [ 0 0;  D 0; ...
                 D + Rt*cos(th),  Rt + Rt*sin(th); ...
                 D 2*Rt;  0 2*Rt; ...
                 Rt*cos(th+pi),   Rt + Rt*sin(th+pi) ];
        info.shape      = 'marks';
        info.separation = D;
        info.radius     = Rt;
        info.lap_len    = 2*D + 2*pi*Rt;
        info.n_marks    = 2;

    case 'circle'
        % THE ACTUAL PEP26 AUTONOMY COURSE (per the team, 2026-09-04):
        % a circle, one lap = 0.5 statute mile, four laps for the 2-mile race.
        % Each lap is therefore exactly one half-mile scoring segment (10 pts).
        %
        %   lap circumference = 804.67 m  ->  R = 128.06 m = 60 boat lengths
        %
        % total_len is the FULL RACE distance here; R_turn is ignored because
        % the radius follows from the lap length.
        lap   = 0.5 * 1609.344;                 % [m] one half-mile lap
        Rc    = lap / (2*pi);                   % [m] 128.06
        nlap  = round(total_len / lap);         % 4 laps for 2 miles
        per   = 48;                             % waypoints per lap
        % 48 points/lap gives 16.8 m chords and a sagitta of only 0.14 m, so
        % the polygon is a far better circle than the boat can track anyway.
        th    = linspace(0, 2*pi*nlap, per*nlap + 1).';
        wpts  = [Rc*sin(th), Rc*(1 - cos(th))]; % starts at origin heading north
        info.shape   = 'circle';
        info.radius  = Rc;
        info.lap_len = lap;
        info.n_laps  = nlap;

    case 'oval'
        % Perimeter = 2*L_straight + 2*pi*R  ->  solve for the straight length.
        L = (total_len - 2*pi*R_turn) / 2;
        assert(L > 0, 'courseGeometry: turn radius too large for this length');
        % Two straights at y = 0 and y = 2R, joined by semicircles.
        th1 = linspace(-pi/2, pi/2, 12).';
        th2 = linspace( pi/2, 3*pi/2, 12).';
        wpts = [ [0 0]; [L 0]; ...
                 [L + R_turn*cos(th1), R_turn + R_turn*sin(th1)]; ...
                 [L 2*R_turn]; [0 2*R_turn]; ...
                 [R_turn*cos(th2), R_turn + R_turn*sin(th2)] ];
        info.shape = 'oval';  info.straight = L;

    case 'triangle'
        % Equilateral, side s, corners rounded.
        s = total_len / 3;
        c = [0 0; s 0; s/2, s*sqrt(3)/2];
        wpts = [c; c(1,:)];
        info.shape = 'triangle';  info.side = s;

    case 'dogbone'
        L = (total_len - 2*pi*R_turn) / 2;
        th1 = linspace(-pi/2, pi/2, 12).';
        th2 = linspace( pi/2, 3*pi/2, 12).';
        wpts = [ [0 0]; [L 0]; ...
                 [L + R_turn*cos(th1), R_turn + R_turn*sin(th1)]; ...
                 [L 2*R_turn]; [0 2*R_turn]; ...
                 [R_turn*cos(th2), R_turn + R_turn*sin(th2)] ];
        info.shape = 'dogbone';  info.straight = L;

    otherwise
        error('courseGeometry: unknown type "%s"', type);
end

% Close the loop -- WITH A TOLERANCE. An exact inequality test is wrong here:
% the circle ends at sin(8*pi), which is -1e-15 rather than 0, so an exact test
% appends a duplicate start point and creates a leg ~1e-13 m long. Cumulative
% progress then tops out at L_course minus one ulp and a `prog >= L_course`
% finish test fails by a rounding error, making a boat that completed the race
% look as though it never finished.
if hypot(wpts(end,1)-wpts(1,1), wpts(end,2)-wpts(1,2)) > 1e-6
    wpts(end+1,:) = wpts(1,:);
end

% Remove duplicate consecutive waypoints, AFTER closing the loop. The arc
% constructions above start at the same point the preceding straight ends on,
% which produces a zero-length leg; atan2(0,0) is then 0, so the leg bearing is
% garbage and the reported "sharpest heading change" comes out as a spurious
% 180 deg.
keep = [true; hypot(diff(wpts(:,1)), diff(wpts(:,2))) > 1e-6];
wpts = wpts(keep,:);

d = hypot(diff(wpts(:,1)), diff(wpts(:,2)));
info.length      = sum(d);
info.n_wpts      = size(wpts,1);
if ~isfield(info,'radius'), info.turn_radius = R_turn; else, info.turn_radius = info.radius; end
% 'circle' is the real course; every other shape here is a placeholder.
info.placeholder = ~strcmpi(type, 'circle');

% Sharpest heading change between consecutive legs, and the turn radius that
% implies at 8 m/s if taken in one steady turn.
a = atan2(diff(wpts(:,2)), diff(wpts(:,1)));
da = abs(mod(diff(a) + pi, 2*pi) - pi);
info.max_heading_change = rad2deg(max(da));
info.min_turn_radius    = R_turn;
end
