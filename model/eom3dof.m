function [xdot, D] = eom3dof(t, x, ctrl, vent_state, P, delta_blade)
% EOM3DOF  3-DOF manoeuvring equations of motion, body-fixed frame at the CG.
%
%   [xdot, D] = eom3dof(t, x, ctrl, vent_state, P, delta_blade)
%
% delta_blade [rad] optional. The BLADE angle after cable backlash, which the
%   stepper maintains because backlash is a non-differentiable hysteresis and
%   cannot live inside a state derivative. Defaults to x(9), i.e. no backlash.
%
% STATE (9)
%   x(1) u      [m/s]    surge velocity, body frame
%   x(2) v      [m/s]    sway velocity, body frame
%   x(3) r      [rad/s]  yaw rate
%   x(4) X      [m]      north position, earth frame
%   x(5) Y      [m]      east position, earth frame
%   x(6) psi    [rad]    heading, +clockwise from north
%   x(7) T_p    [N]      delivered port thrust   (ESC/rotor spool-up state)
%   x(8) T_s    [N]      delivered stbd thrust
%   x(9) delta  [rad]    actual rudder angle     (servo lag state)
%
% CTRL (commands, held by a ZOH between controller ticks)
%   ctrl.delta_cmd, ctrl.T_cmd_port, ctrl.T_cmd_stbd
%
% VENT_STATE
%   The ventilation latch is a DISCRETE state and is deliberately held FIXED
%   across all four RK4 stages, then updated once per step by the stepper.
%   Letting it flip mid-stage would make the four stages disagree about which
%   physical branch they are on and destroy the integrator's order.
%
% ===========================================================================
% THE EQUATIONS, TERM BY TERM
% ===========================================================================
%
%   (m - X_udot)*udot  = (m - Y_vdot)*v*r  + X_hull + X_prop + X_rud + X_aero
%   (m - Y_vdot)*vdot  = -(m - X_udot)*u*r + Y_hull + Y_rud  + Y_aero + Y_dist
%   (Iz - N_rdot)*rdot = (X_udot - Y_vdot)*u*v + N_hull + N_rud + N_prop
%                                              + N_aero + N_dist
%
% LEFT-HAND SIDES. X_udot, Y_vdot, N_rdot are added mass/inertia and are stored
% NEGATIVE (SNAME convention), so (m - X_udot) reads as "mass plus added mass".
% Writing it this way keeps the equations identical to the textbook rather than
% littered with ad-hoc sign flips.
%
% TERM 1 -- CORIOLIS / CENTRIPETAL: (m - Y_vdot)*v*r and -(m - X_udot)*u*r
%   Pure kinematics. These are not forces; they are what appears when you write
%   Newton's second law in a frame that is itself rotating. The sway term
%   -m*u*r is the centripetal acceleration of a turn: a boat going u forward at
%   yaw rate r is being accelerated sideways at u*r whether or not anything is
%   pushing it. Always present, never linearised away.
%
% TERM 2 -- MUNK MOMENT: (X_udot - Y_vdot)*u*v
%   A body with more added mass in sway than in surge, moving at an angle to
%   the flow, feels a couple that tries to turn it BROADSIDE. Destabilising in
%   yaw, and it grows with sideslip times speed.
%   The prompt expected this to be small for a planing hull. IT IS NOT
%   NECESSARILY SMALL. It is proportional to (X_udot - Y_vdot), and the sweep
%   ranges over Xudot_frac 0.02-0.15 and Yvdot_frac 0.05-0.40. At the low end
%   the two added masses cancel and the Munk moment vanishes; at the high end
%   it exceeds full rudder authority at a few degrees of sideslip. D.N_munk is
%   logged every step so its magnitude is reported, not assumed.
%
% TERM 3 -- SPEED-SQUARED ACTUATOR AUTHORITY
%   N_rud goes as u^2 (dynamic pressure on the blade). The hull damping N_r
%   goes as u^1. The plant input gain is therefore quadratic in speed while the
%   damping is linear. That single mismatch is why one fixed PID cannot work
%   from 4 to 30 mph, and it is the origin of the Kp ~ 1/U schedule.
%
% TERM 4 -- QUADRATIC DAMPING
%   In hullForces, via strip-theory cross-flow drag alongside the linear terms.
%
% TERM 5 -- TURN-INDUCED DRAG -> SPEED LOSS -> GAIN CHANGE
%   In hullForces (energy argument) plus rudder drag here. u is a logged output
%   precisely so a >15% sag in a hard turn can be detected: the gain schedule
%   is scheduled ON u, so if u moves a lot mid-manoeuvre the linearisation the
%   gains were derived from is no longer the plant being controlled.
% ===========================================================================

u   = x(1);  v = x(2);  r = x(3);
psi = x(6);
T_p = x(7);  T_s = x(8);  delta = x(9);

% The servo state x(9) drives the linkage; the BLADE may lag it by the cable
% free play. Hydrodynamics see the blade; the servo lag equation sees x(9).
if nargin < 6 || isempty(delta_blade), delta_blade = delta; end

%% ---- 0. Water-relative velocity ---------------------------------------
% A current is not a force. It is a change of frame: every HYDRODYNAMIC force
% depends on motion relative to the WATER, while the kinematics that integrate
% position depend on motion relative to the GROUND. Applying a current as a
% force instead is the classic way to get a cross-track controller that biases.
% It is also what makes heading differ from course over ground (the crab angle).
c = cos(psi);  s = sin(psi);
Vc_earth = P.E.V_current * [cos(P.E.psi_current); sin(P.E.psi_current)];
Vc_body  = [ c, s; -s, c] * Vc_earth;                       % [m/s]

u_w = u - Vc_body(1);                                       % water-relative surge
v_w = v - Vc_body(2);                                       % water-relative sway

%% ---- 1. Force contributions -------------------------------------------
H  = hullSteadyState(u_w, P, 'lite');   % dynamics only need SW and R_total
Hf = hullForces(u_w, v_w, r, P, H);                         % hull
Rf = rudderForces(delta_blade, u_w, v_w, r, vent_state, P); % rudder (blade angle)
Pf = propForces(T_p, T_s, u_w, P);                          % props
Ef = envForces(u, v, r, psi, t, P);                         % wind + waves

%% ---- 2. Rigid-body terms ----------------------------------------------
m  = P.m;   Iz = P.Iz;
Xu = P.X_udot;  Yv = P.Y_vdot;  Nr = P.N_rdot;

M_surge = m  - Xu;                                          % [kg]
M_sway  = m  - Yv;                                          % [kg]
M_yaw   = Iz - Nr;                                          % [kg*m^2]

Cor_surge =  (m - Yv) * v * r;                              % [N]
Cor_sway  = -(m - Xu) * u * r;                              % [N]
N_munk    =  (Xu - Yv) * u_w * v_w;                         % [N*m]

%% ---- 3. Accelerations --------------------------------------------------
X_sum = Hf.X + Pf.X + Rf.X + Ef.X;
Y_sum = Hf.Y + Rf.Y + Ef.Y;
N_sum = Hf.N + Rf.N + Pf.N + Ef.N;

udot = (Cor_surge + X_sum) / M_surge;
vdot = (Cor_sway  + Y_sum) / M_sway;
rdot = (N_munk    + N_sum) / M_yaw;

%% ---- 4. Kinematics -----------------------------------------------------
% Earth-frame velocity is the rotation of the body-frame velocity, plus the
% current, which carries the boat bodily over the ground.
psidot = r;
Xdot = u*c - v*s;
Ydot = u*s + v*c;

%% ---- 5. Actuator states ------------------------------------------------
% ESC + motor + prop spool-up: first-order lag on commanded thrust, saturated
% at what the prop can actually deliver at this speed.
T_max = max(Pf.T_max, 1e-6);
Tp_cmd = min(max(ctrl.T_cmd_port, -0.3*T_max), T_max);      % limited astern
Ts_cmd = min(max(ctrl.T_cmd_stbd, -0.3*T_max), T_max);
Tpdot = (Tp_cmd - T_p) / P.A.tau_thrust;
Tsdot = (Ts_cmd - T_s) / P.A.tau_thrust;

% Servo: rate-limited first-order lag with hard position stops. Backlash is
% NOT here -- it is a non-differentiable hysteresis and belongs in the stepper,
% applied between the servo output and the blade.
d_cmd = min(max(ctrl.delta_cmd, -P.R.delta_max), P.R.delta_max);
ddot_raw = (d_cmd - delta) / P.A.tau_servo_lag;

% Rate ceiling drops as the hinge moment eats into the stall torque -- a servo
% at 80% of stall barely moves. This is what actually caps achievable Kd.
rate_cap = P.A.rate_max;
if P.A.rate_load_knockdown
    load_frac = min(Rf.tau_servo / P.A.tau_stall, 0.95);
    rate_cap  = rate_cap * (1 - load_frac);
end
ddot = sign(ddot_raw) * min(abs(ddot_raw), rate_cap);

% Hard stops: do not integrate past the mechanical limit.
if (delta >=  P.R.delta_max && ddot > 0) || (delta <= -P.R.delta_max && ddot < 0)
    ddot = 0;
end

xdot = [udot; vdot; rdot; Xdot; Ydot; psidot; Tpdot; Tsdot; ddot];

%% ---- 6. Diagnostics ----------------------------------------------------
if nargout > 1
    D.H = H;  D.Hf = Hf;  D.Rf = Rf;  D.Pf = Pf;  D.Ef = Ef;
    D.Cor_surge = Cor_surge;  D.Cor_sway = Cor_sway;
    D.N_munk = N_munk;
    D.N_rud = Rf.N;  D.N_hull = Hf.N;  D.N_prop = Pf.N;  D.N_aero = Ef.N;
    D.u_w = u_w;  D.v_w = v_w;
    D.beta = atan2(v_w, max(abs(u_w), 0.05));               % [rad] sideslip
    D.chi  = atan2(Ydot, Xdot);                             % [rad] course over ground
    D.crab = wrapToPiLocal(D.chi - psi);                    % [rad] crab angle
    D.vent_state = Rf.vent_state;
    D.T_max = T_max;
    D.rate_cap = rate_cap;
    D.sigma = Rf.sigma;
end
end

% -------------------------------------------------------------------------
function a = wrapToPiLocal(a)
% WRAPTOPILOCAL  Wrap to (-pi, pi]. Local so no Mapping Toolbox is needed.
a = mod(a + pi, 2*pi) - pi;
end
