function Ef = envForces(u, v, r, psi, t, P)
% ENVFORCES  Wind side force and wave-induced yaw moment.
%
% Current is NOT handled here -- it is handled in eom3dof by taking every
% hydrodynamic force off the WATER-RELATIVE velocity, which is the physically
% correct place for it. A current is not a force on the boat; it is a shift in
% the frame the water forces are computed in. Getting that wrong is what makes
% a cross-track controller quietly bias.
%
% Wind, by contrast, IS a force: the boat is moving through air at a different
% relative velocity than it is through water.

E = P.E;  V = P.V;

%% ---- Wind ---------------------------------------------------------------
% Wind direction is stored as the direction it comes FROM (meteorological).
% Convert to the vector it blows TO, in earth axes, then into body axes.
psi_to = E.psi_wind + pi;
Vw_earth = E.V_wind * [cos(psi_to); sin(psi_to)];           % [m/s] N, E

c = cos(psi); s = sin(psi);
R_eb = [ c,  s;                                              % earth -> body
        -s,  c];
Vw_body = R_eb * Vw_earth;                                   % [m/s]

% Air-relative velocity of the hull.
u_a = u - Vw_body(1);
v_a = v - Vw_body(2);
V_rel = hypot(u_a, v_a);
beta_w = atan2(-v_a, max(abs(u_a), 0.05));                   % [rad] aero sideslip

% Side force on the lateral projected area. Linear in sideslip to ~30 deg, then
% flat -- a bluff body does not stall in the way a foil does.
CY = E.CY_beta * max(min(beta_w, deg2rad(30)), -deg2rad(30));
q_a = 0.5 * E.rho_a * V_rel^2;                               % [Pa]

Y_aero = q_a * V.A_lateral * CY;                             % [N]

% THE AERO CENTRE OF PRESSURE IS FORWARD OF THE CG, so the wind side force
% makes a moment that turns the bow FURTHER off the wind. This is destabilising
% -- a weathervane in reverse. A 44 kg boat with a tunnel, deck and canopy has a
% lot of lateral area for its mass, so at 15 kt of crosswind this is not a
% trim nuisance, it is a disturbance the controller has to actively fight.
x_cp = E.x_cp_aero * V.LOA;                                  % [m] fwd of CG
N_aero = Y_aero * x_cp;                                      % [N*m]

% Aero drag along the hull axis.
X_aero = -q_a * V.A_lateral * 0.1 * sign(u_a) * abs(cos(beta_w));

%% ---- Waves --------------------------------------------------------------
% Band-limited yaw moment. Deterministic in t (sum of fixed-phase sinusoids
% across the band) rather than randn, so that an RK4 stage evaluated twice at
% the same t returns the SAME value. Calling randn inside a derivative function
% corrupts the integrator -- the four stages would see four different worlds.
N_wave = 0;
if E.wave_N_amp > 0
    f = linspace(E.wave_f_lo, E.wave_f_hi, 8);
    ph = 2*pi*(0:7)/8 * 3.7;                                 % fixed scrambled phases
    N_wave = E.wave_N_amp * sum(sin(2*pi*f*t + ph)) / sqrt(8);
end

%% ---- Pack ---------------------------------------------------------------
Ef.X = X_aero;
Ef.Y = Y_aero;
Ef.N = N_aero + N_wave;
Ef.N_wave = N_wave;
Ef.beta_wind = beta_w;
Ef.V_rel_air = V_rel;
end
