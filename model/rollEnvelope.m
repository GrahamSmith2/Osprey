function RE = rollEnvelope(u, P, beta)
% ROLLENVELOPE  Quasi-static roll / blow-over check and the safe yaw-rate limit.
%
%   RE = rollEnvelope(u, P, beta)     u vector [m/s], beta sideslip [rad] (opt.)
%
% WHY THIS EXISTS AND WHY IT IS NOT IN THE 3-DOF MODEL
% A tunnel cat at speed has two failure modes a PLANAR model is structurally
% incapable of seeing:
%   HOOKING     -- the outer sponson digs in during a drifting turn, trips, and
%                  the boat spins out or rolls.
%   BLOW-OVER   -- the bow lifts, aero lift under the tunnel takes over from
%                  hydrodynamic lift, and the boat goes over backwards.
% A 3-DOF surge/sway/yaw model has no roll or pitch degree of freedom, so it
% will happily command a manoeuvre that would put the boat on its roof and
% report a perfectly good trajectory.
%
% This function does NOT fix that. It is a quasi-static envelope evaluated
% ALONGSIDE the sim, producing a hard yaw-rate ceiling r_max_safe(u) that the
% guidance loop is required to respect. A slow safe boat beats a fast one
% upside down.
%
% THREE CHECKS
%   1. Inner sponson unloading (centripetal load transfer)
%   2. Aero lift fraction approaching the reported blow-over threshold
%   3. Sideslip-driven hooking
% The binding one is reported, per speed.

if nargin < 3, beta = deg2rad(5); end
V = P.V;  E = P.E;
u = abs(u(:).');

H = hullSteadyState(u, P);
W = P.m * E.g;

%% ---- Check 1: inner sponson unloading ---------------------------------
% In a steady turn the lateral acceleration is a_y = u*r, acting at the CG,
% height h_cg above the planing surfaces. The reaction acts at the pads. That
% couple transfers load outboard:
%
%       dN = m * a_y * h_cg / (2 * y_hull)
%
% The inner sponson goes light when dN reaches half the weight, i.e. when
%
%       a_y_crit = g * y_hull / h_cg
%
% Note this is INDEPENDENT of mass -- a wider or lower boat is more stable,
% a heavier one is not. Once an inner sponson unloads on a cat, the remaining
% hull is a single narrow planing surface and directional behaviour changes
% completely, so this is treated as the hard limit rather than actual capsize.
a_y_crit = E.g * V.y_hull / V.h_cg;                       % [m/s^2]

% A safety factor, because this is quasi-static: it ignores roll inertia, the
% transient overshoot when the rudder is slammed, and wave-induced roll.
SF = 0.6;
r_max_roll = SF * a_y_crit ./ max(u, 0.1);                % [rad/s]

%% ---- Check 2: blow-over / aero lift fraction --------------------------
% The report targets 30-40% of weight in aero lift specifically to manage
% blow-over. Past ~40% the tunnel is carrying enough that a pitch-up excursion
% can run away, because tunnel lift grows with angle of attack while the
% hydrodynamic lift that would restore it is disappearing as the hull leaves
% the water.
f_aero_limit = 0.40;
blowover_margin = f_aero_limit - H.f_aero;                % [-] +ve is safe
u_blowover = interpSafe(H.f_aero, u, f_aero_limit);       % [m/s] speed at limit

% In a turn the boat carries sideslip, which makes the tunnel see asymmetric
% flow: the windward sponson side gets more ram pressure. Treat this as an
% effective INCREASE in the lift fraction proportional to sideslip, which
% tightens the limit at high drift angles.
f_aero_eff = H.f_aero * (1 + 2*abs(sin(beta)));
blowover_ok = f_aero_eff < f_aero_limit;

%% ---- Check 3: hooking --------------------------------------------------
% The outer sponson trips when the drift angle gets large enough that it is
% presenting its side to the flow rather than its planing pad. There is no
% clean theory for this at these Froude numbers; the threshold below is a
% judgement call from planing-craft practice, not a derived quantity.
% See ASSUMPTIONS.md #A15.
beta_hook = deg2rad(12);                                   % [rad]
% Convert to a yaw-rate ceiling: in a steady turn sideslip grows roughly with
% r*x_cg/u (the drift the stern must generate to sustain the turn).
r_max_hook = beta_hook * max(u,0.1) / max(V.x_cg, 1e-3);   % [rad/s]

%% ---- Combine -----------------------------------------------------------
r_max_safe = min(r_max_roll, r_max_hook);

% Which check binds, per speed: 1 = roll/unloading, 3 = hooking.
binding = ones(size(u));
binding(r_max_hook < r_max_roll) = 3;
% Where the aero fraction is already past the limit, nothing else matters.
binding(~blowover_ok) = 2;
r_max_safe(~blowover_ok) = 0;

RE.u              = u;
RE.r_max_safe     = r_max_safe;
RE.r_max_roll     = r_max_roll;
RE.r_max_hook     = r_max_hook;
RE.a_y_crit       = a_y_crit;
RE.f_aero         = H.f_aero;
RE.f_aero_eff     = f_aero_eff;
RE.blowover_margin= blowover_margin;
RE.u_blowover     = u_blowover;
RE.binding        = binding;
RE.beta_hook      = beta_hook;
RE.SF             = SF;
end

% -------------------------------------------------------------------------
function xq = interpSafe(y, x, y0)
% INTERPSAFE  First crossing of y through y0, linear. NaN if never crossed.
i = find(y(1:end-1) < y0 & y(2:end) >= y0, 1);
if isempty(i), xq = NaN; return; end
xq = x(i) + (y0 - y(i)) * (x(i+1)-x(i)) / (y(i+1)-y(i));
end
