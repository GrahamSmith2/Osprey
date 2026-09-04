function A = allocate(N_cmd, T_base, u, P, opts)
% ALLOCATE  Split a commanded yaw moment between rudder and differential thrust.
%
%   A = allocate(N_cmd, T_base, u, P, opts)
%   N_cmd  [N*m] yaw moment demanded by the rate loop
%   T_base [N]   per-motor thrust demanded by the speed loop
%
% TWO ACTUATORS WITH OPPOSITE SPEED DEPENDENCE
%   Rudder moment      ~ u^2  -> ZERO at the start line, dominant when planing
%   Differential thrust ~ u^0  -> available at rest, fades as thrust falls off
%                                 with advance ratio at high speed
%
% PRIORITY ALLOCATION, NOT A SCHEDULED BLEND
% The rudder is used first, up to its useful limit, and only the shortfall goes
% to differential thrust. That ordering is deliberate: rudder costs a little
% drag, whereas differential thrust costs FORWARD SPEED directly (one motor
% must come down, and the pair cannot both sit at the ceiling). Spending speed
% to turn is the more expensive option, so it is the fallback.
%
% The crossover speed then EMERGES from the physics rather than being a tuned
% schedule: as u falls, the rudder's u^2 authority collapses, the shortfall
% grows, and thrust takes over automatically. analysis/ reports where that
% happens; nothing here has to be told the number.
%
% USEFUL DEFLECTION LIMIT
% Deflection is clamped at ventilation onset by default, NOT at the 35 deg
% mechanical stop. Past onset the yaw moment actually FALLS with increasing
% deflection (see ASSUMPTIONS.md #A5), so commanding into that region gives the
% controller a negative plant gain and hands it a limit cycle. The extra
% authority beyond onset is not real.

if nargin < 5, opts = struct(); end
clamp_at_vent = getf(opts, 'clamp_at_vent', true);

R = P.R;  E = P.E;

if clamp_at_vent
    delta_useful = min(R.delta_vent_on, R.delta_max);
else
    delta_useful = R.delta_max;
end

%% ---- Rudder authority at this speed ------------------------------------
AR_eff   = R.k_surface * (R.h_sub / R.c);
CL_alpha = 1.8*pi*AR_eff / (1.8 + sqrt(AR_eff^2 + 4)) * R.CLa_fac;
% u_alloc is the speed used for the moment -> angle conversion. Normally the
% MEASURED speed, which makes the conversion itself a form of gain scheduling.
% Setting opts.u_alloc_fix freezes it, modelling an autopilot whose steering
% path has no speed knowledge at all (ArduPilot Rover's steering controller
% commands an angle directly). That is the configuration in which the classic
% "fixed gains fail at one end of the speed range" result actually appears.
u_alloc  = getf(opts, 'u_alloc_fix', max(abs(u), 0.05));
q        = 0.5 * E.rho_w * u_alloc^2;

dN_ddelta = q * R.A_r * CL_alpha * abs(P.x_r);        % [N*m/rad]
N_rud_max = dN_ddelta * delta_useful;                 % [N*m]

% opts.rudder_failed models a fault-detection system having concluded the
% rudder is not responding, so the allocator stops crediting it with any
% authority and routes the whole demand to differential thrust.
if getf(opts, 'rudder_failed', false)
    N_rud_max = 0;
end

%% ---- Rudder first ------------------------------------------------------
N_rud_cmd = max(min(N_cmd, N_rud_max), -N_rud_max);
if dN_ddelta > 1e-9
    delta_cmd = N_rud_cmd / dN_ddelta;
else
    delta_cmd = 0;
end
delta_cmd = max(min(delta_cmd, delta_useful), -delta_useful);

%% ---- Differential thrust picks up the shortfall ------------------------
N_short = N_cmd - N_rud_cmd;                          % [N*m]
dT_want = N_short / P.V.y_p;                          % [N] port minus stbd

% Headroom: neither motor may exceed what the prop can deliver at this speed,
% nor go below the modest astern limit the ESC allows.
T_max  = propMaxThrust(abs(u), P);
T_base = max(min(T_base, T_max), -0.3*T_max);
head_up = T_max - T_base;
head_dn = T_base + 0.3*T_max;
dT_lim  = 2 * max(min(head_up, head_dn), 0);          % symmetric split

dT = max(min(dT_want, dT_lim), -dT_lim);

A.T_port = T_base + dT/2;
A.T_stbd = T_base - dT/2;
A.T_port = max(min(A.T_port, T_max), -0.3*T_max);
A.T_stbd = max(min(A.T_stbd, T_max), -0.3*T_max);

%% ---- Report ------------------------------------------------------------
A.delta_cmd  = delta_cmd;
A.N_rud_cmd  = N_rud_cmd;
A.N_thr_cmd  = (A.T_port - A.T_stbd) * P.V.y_p;
A.N_achieved = A.N_rud_cmd + A.N_thr_cmd;
A.N_deficit  = N_cmd - A.N_achieved;                  % >0 => cannot comply
A.saturated  = abs(A.N_deficit) > 1e-6 * max(abs(N_cmd), 1);
A.N_rud_max  = N_rud_max;
A.N_thr_max  = dT_lim * P.V.y_p;
A.dN_ddelta  = dN_ddelta;
A.delta_useful = delta_useful;
A.T_max      = T_max;
end

% -------------------------------------------------------------------------
function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
