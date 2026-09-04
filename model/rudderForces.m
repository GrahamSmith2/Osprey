function Rf = rudderForces(delta, u, v, r, vent_state, P)
% RUDDERFORCES  Side force, drag, yaw moment, hinge moment and servo demand.
%
%   Rf = rudderForces(delta, u, v, r, vent_state, P)
%
% delta       [rad] ACTUAL blade angle (post-servo-dynamics, not the command)
% u, v, r     [m/s, m/s, rad/s] body-frame surge, sway, yaw rate
% vent_state  [0 or 1] ventilation latch from the previous step (hysteresis)
% P           parameter struct from buildParams
%
% SIGN CONVENTION (fixed here once, used everywhere)
%   delta > 0  ->  positive yaw moment N  ->  turn to starboard.
%   The rudder sits AFT of the CG, so P.x_r < 0. Setting Y_rud negative for
%   positive delta then makes N_rud = x_r*Y_rud positive. Every sign in the
%   rest of the model follows from that one choice.
%
% THE THREE NON-LINEARITIES THAT DOMINATE TUNING
%   1. Ventilation  -- plant gain can HALVE mid-turn and not come back until the
%                      blade is unloaded well past the inception angle. Latched.
%   2. Cavitation   -- reported, not modelled. Past sigma ~ 0.5 this whole
%                      formulation is void; the model prints where that happens.
%   3. Stall        -- low-AR surfaces stall late (>25 deg) but ventilation
%                      bounds the useful range long before stall does.

R = P.R;  E = P.E;

%% ---- 1. Effective aspect ratio ----------------------------------------
% A surface-piercing rudder gets NO free-surface image doubling: the free
% surface is a pressure-release boundary, and once ventilated it is literally
% open to atmosphere. A deeply submerged foil with an end plate would get ~2x.
% k_surface is swept 1.0-2.0 rather than assumed. See ASSUMPTIONS.md #A4.
AR_eff = R.k_surface * (R.h_sub / R.c);                    % [-]

%% ---- 2. Lift-curve slope: Whicker-Fehlner ------------------------------
% Standard low-aspect-ratio control-surface correlation:
%     dCL/dalpha = 1.8*pi*AR / (1.8 + sqrt(AR^2 + 4))       [1/rad]
% At AR_eff = 2.5 this gives ~2.34 /rad, well below the 2*pi of a 2-D section --
% which is the whole point: this blade is small AND stubby.
CL_alpha = 1.8*pi*AR_eff / (1.8 + sqrt(AR_eff^2 + 4)) * R.CLa_fac;   % [1/rad]

%% ---- 3. Effective angle of attack --------------------------------------
% The blade does not see the commanded angle. It sees the commanded angle PLUS
% the local flow angle produced by the boat's own sway and yaw. The lateral
% velocity at the rudder is v + r*x_r.
%
% THIS TERM IS THE RUDDER'S CONTRIBUTION TO YAW DAMPING and it is not optional:
% delete it and the boat becomes far less directionally stable than it is, and
% the identified Nomoto T comes out wrong. It is also a genuine non-linear
% coupling -- the actuator's effectiveness depends on the states it is driving.
u_r   = max(abs(u), 0.05);                                  % [m/s] avoid /0
v_r   = v + r * P.x_r;                                      % [m/s] at the stock
beta_r = atan2(v_r, u_r);                                   % [rad] inflow angle
alpha  = delta + beta_r;                                    % [rad] effective AoA

%% ---- 4. Ventilation, with hysteresis ----------------------------------
% Inception at |alpha| > delta_vent_on. Re-wetting only once |alpha| falls
% BELOW (delta_vent_on - hysteresis), because a ventilated cavity is
% self-sustaining: the air path is already open.
a_on  = R.delta_vent_on;
a_off = max(R.delta_vent_on - R.vent_hysteresis, deg2rad(1));

vent_new = vent_state;
if abs(alpha) > a_on,  vent_new = 1; end
if abs(alpha) < a_off, vent_new = 0; end

% Blend rather than step, so the state derivative stays integrable. The latch
% above decides WHICH curve we are on; this decides how sharply we are on it.
if vent_new
    w = smoothsat((abs(alpha) - a_off) / R.vent_blend);
else
    w = 0;
end
k_vent = 1 - w * (1 - R.k_vent_loss);                       % [-] 1 -> k_vent_loss

%% ---- 5. Lift and drag --------------------------------------------------
% Late stall for a low-AR surface: hold linear to 25 deg, then fade.
a_stall = deg2rad(25);
CL_lin  = CL_alpha * alpha;
f_stall = 1 - smoothsat((abs(alpha) - a_stall) / deg2rad(10)) * 0.6;
CL      = CL_lin * f_stall * k_vent;                        % [-]

CD = R.CD0 + CL^2 / (pi * AR_eff * R.e_oswald);             % [-]

q      = 0.5 * E.rho_w * u_r^2;                             % [Pa]
F_N    = q * R.A_r * CL;                                    % [N] normal force
F_D    = q * R.A_r * CD;                                    % [N] drag

% Body-frame components. Y negative for positive delta (see sign convention).
Y_rud = -F_N;                                               % [N]
X_rud = -F_D;                                               % [N] always retarding
N_rud = P.x_r * Y_rud;                                      % [N*m] positive for
                                                            % positive delta

%% ---- 6. Hinge moment and servo demand ---------------------------------
% Hinge moment is force x the distance from the CENTRE OF PRESSURE to the
% STOCK AXIS -- NOT force x servo horn radius. Getting this wrong is the single
% most common way to mis-size a steering servo, in either direction:
%   - stock aft of CoP  -> negative (overbalanced) hinge moment, servo fights
%     an unstable blade that wants to slam to the stop;
%   - stock at the LE   -> lever = 0.25c, hinge moment ~4x a 25%-balanced blade.
e_arm = (R.x_cp_frac - R.x_stock_frac) * R.c;               % [m] signed lever
M_h   = F_N * e_arm;                                        % [N*m] at the stock

% Cable linkage: rudder tiller arm -> cable tension -> servo horn.
%   T_cable   = M_h / r_rudder_arm            [N]
%   tau_servo = T_cable * r_servo_horn / eta  [N*m]
T_cable   = M_h / P.A.r_rudder_arm;                         % [N]
tau_servo = abs(T_cable) * P.A.r_servo_horn / P.A.eta_cable;% [N*m]

%% ---- 7. Cavitation number (reported, not modelled) --------------------
% sigma = (p_atm + rho*g*h - p_v) / (0.5*rho*U^2)
% The blade is shallow, so the hydrostatic term is nearly nil and sigma is
% essentially set by speed alone. Below sigma ~ 0.5 the attached-flow lift
% model above is meaningless -- analysis/ prints the speed where that happens.
h_mid = R.h_sub / 2;                                        % [m] mid-span depth
sigma = (E.p_atm + E.rho_w*E.g*h_mid - E.p_vap) / max(q, 1);

%% ---- 8. Pack ----------------------------------------------------------
Rf.X = X_rud;      Rf.Y = Y_rud;      Rf.N = N_rud;
Rf.F_N = F_N;      Rf.F_D = F_D;
Rf.CL = CL;        Rf.CD = CD;        Rf.CL_alpha = CL_alpha;
Rf.AR_eff = AR_eff;
Rf.alpha = alpha;  Rf.beta_r = beta_r;
Rf.vent_state = vent_new;   Rf.k_vent = k_vent;
Rf.M_h = M_h;      Rf.tau_servo = tau_servo;   Rf.e_arm = e_arm;
Rf.sigma = sigma;

end

% -------------------------------------------------------------------------
function y = smoothsat(t)
% SMOOTHSAT  C1-continuous 0->1 saturating ramp (same shape as smoothstep).
t = min(max(t, 0), 1);
y = t.^2 .* (3 - 2*t);
end
