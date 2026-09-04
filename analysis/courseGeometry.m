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

% Remove duplicate consecutive waypoints. The arc constructions above start at
% the same point the preceding straight ends on, which produces a zero-length
% leg. atan2(0,0) is then 0, so the leg bearing is garbage and the reported
% "sharpest heading change" comes out as a spurious 180 deg.
keep = [true; hypot(diff(wpts(:,1)), diff(wpts(:,2))) > 1e-9];
wpts = wpts(keep,:);

% Close the loop.
if any(wpts(end,:) ~= wpts(1,:)), wpts(end+1,:) = wpts(1,:); end

d = hypot(diff(wpts(:,1)), diff(wpts(:,2)));
info.length      = sum(d);
info.n_wpts      = size(wpts,1);
info.turn_radius = R_turn;
info.placeholder = true;      % <-- every consumer must report this

% Sharpest heading change between consecutive legs, and the turn radius that
% implies at 8 m/s if taken in one steady turn.
a = atan2(diff(wpts(:,2)), diff(wpts(:,1)));
da = abs(mod(diff(a) + pi, 2*pi) - pi);
info.max_heading_change = rad2deg(max(da));
info.min_turn_radius    = R_turn;
end
