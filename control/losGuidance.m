function [psi_cmd, gi] = losGuidance(X, Y, chi, wpts, gi, P)
% LOSGUIDANCE  Lookahead line-of-sight guidance along a waypoint list.
%
%   [psi_cmd, gi] = losGuidance(X, Y, chi, wpts, gi, P)
%   wpts  n-by-2 [north, east] waypoints
%   gi    guidance memory: gi.k (current leg index), gi.L_a (lookahead)
%
% LOS steers toward a point a fixed LOOKAHEAD DISTANCE L_a ahead of the boat's
% projection onto the current leg. The resulting course command is
%
%       chi_cmd = alpha_leg + atan( -e / L_a )
%
% where e is signed cross-track error and alpha_leg is the leg bearing. Small
% L_a turns hard onto the line and can oscillate; large L_a converges gently
% and cuts corners. L_a ~ 2-4 boat lengths is the usual band.
%
% ---------------------------------------------------------------------------
% THE COURSE-VS-HEADING TRAP
% ---------------------------------------------------------------------------
% LOS produces a COURSE command chi_cmd -- the direction the boat must TRAVEL.
% The inner loop controls HEADING psi -- the direction the bow POINTS. In a
% current or crosswind the two differ by the crab angle:
%
%       psi_cmd = chi_cmd - crab,     crab = chi - psi
%
% Omitting that correction is the single most common cross-track bug. The boat
% holds the commanded bearing with the bow, sails steadily downstream of the
% track, and the heading error reads ZERO the entire time, so the fault is
% invisible in the heading channel. The heading integrator eventually removes
% it, but slowly and only up to its own limits.
%
% Here the crab correction is applied explicitly, and only when the COG
% solution is trustworthy (GPS course is meaningless at low speed).

if ~isfield(gi, 'k'),    gi.k = 1;              end
if ~isfield(gi, 'L_a'),  gi.L_a = 3 * P.V.LOA;  end   % [m] ~6.4 m
if ~isfield(gi, 'R_accept'), gi.R_accept = 2 * P.V.LOA; end
if ~isfield(gi, 'crab_valid'), gi.crab_valid = false; end
if ~isfield(gi, 'crab'), gi.crab = 0; end

n = size(wpts, 1);
k = min(gi.k, n-1);

p0 = wpts(k, :);
p1 = wpts(k+1, :);

d      = p1 - p0;
alpha  = atan2(d(2), d(1));                 % [rad] leg bearing
Lleg   = hypot(d(1), d(2));

% Along-track and cross-track error, in the leg frame.
dp     = [X - p0(1), Y - p0(2)];
s_along= ( dp(1)*cos(alpha) + dp(2)*sin(alpha));
e_cross= (-dp(1)*sin(alpha) + dp(2)*cos(alpha));

%% ---- Waypoint switching -------------------------------------------------
% Switch on either a capture radius OR having passed the waypoint along-track.
% The along-track test matters: with a capture radius alone, a boat that
% overshoots wide never gets inside the circle and will orbit the waypoint
% forever trying to reach it.
if (hypot(X - p1(1), Y - p1(2)) < gi.R_accept || s_along > Lleg) && k < n-1
    gi.k = k + 1;
    [psi_cmd, gi] = losGuidance(X, Y, chi, wpts, gi, P);
    return
end

%% ---- LOS course command -------------------------------------------------
chi_cmd = alpha + atan2(-e_cross, gi.L_a);

%% ---- Convert course command to a HEADING command -----------------------
if gi.crab_valid
    psi_cmd = wrapPi(chi_cmd - gi.crab);
else
    psi_cmd = wrapPi(chi_cmd);
end

gi.e_cross = e_cross;
gi.s_along = s_along;
gi.alpha   = alpha;
gi.chi_cmd = chi_cmd;
gi.leg     = k;
gi.dist_to_wp = hypot(X - p1(1), Y - p1(2));
gi.finished = (k == n-1) && (s_along > Lleg);
end

% -------------------------------------------------------------------------
function a = wrapPi(a)
a = mod(a + pi, 2*pi) - pi;
end
