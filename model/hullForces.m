function Hf = hullForces(u, v, r, P, H)
% HULLFORCES  Hull sway force, yaw moment and resistance at the current state.
%
%   Hf = hullForces(u, v, r, P, H)
%   H is an optional pre-computed hullSteadyState(u,P) result. Pass it in from
%   the EOM so the Savitsky solve is not repeated inside every RK4 stage.
%
% ---------------------------------------------------------------------------
% WHERE THESE NUMBERS COME FROM -- AND WHY YOU SHOULD NOT BELIEVE THEM
% ---------------------------------------------------------------------------
% The linear derivatives use the Clarke (1982) regression. That regression is
% fitted to slender DISPLACEMENT hulls at Fn < 0.3. Osprey runs at Fn 2.9-7.8
% on planing pads with a ram-air tunnel. The regression is being used roughly
% one and a half orders of magnitude outside its domain, on the wrong hull form.
%
% It is used anyway because the alternative is inventing numbers with no
% pedigree at all, and because the STRUCTURE it gives (Y_v linear in U, N_r
% linear in U, both proportional to immersed area) is right even where the
% MAGNITUDE is not. The magnitudes are swept +/-5x in uncertainty.m.
%
% Every conclusion in this repo is required to survive that 5x sweep. If one
% does not, it gets reported as not surviving it.
% See ASSUMPTIONS.md #A11.
% ---------------------------------------------------------------------------
%
% TWO DAMPING MECHANISMS, AND WHERE THEY CROSS OVER
%   LINEAR (lifting-body):  force ~ rho*L^2*U*v      -- grows as U^1 * v^1
%   QUADRATIC (cross-flow): force ~ rho*A*v*|v|      -- grows as v^2, no U
% At small drift the linear term dominates; at large drift the quadratic one
% does. analysis/ reports the crossover drift angle. This matters because a
% controller linearised about straight-line running sees only the linear term,
% and a hard turn lives in the quadratic one.

E = P.E;  V = P.V;
if nargin < 5 || isempty(H), H = hullSteadyState(u, P, 'lite'); end

U = max(abs(u), 0.05);                    % [m/s] guard against divide-by-zero

%% ---- 1. Geometry at the displacement (fully immersed) condition -------
% Clarke wants L, B, T and block coefficient. Osprey is a catamaran, so the
% regression is applied PER DEMIHULL and the result doubled. That is correct
% for both sway and yaw: in pure sway both hulls translate identically, and in
% pure yaw each hull sees the same local velocity v + r*x as a monohull would.
% Lateral hull separation does NOT add yaw damping here -- it would only matter
% for fore-aft force differences between the hulls, which sway/yaw do not make.
L   = 0.85 * V.LOA;                       % [m] waterline length
T   = H.T_draft;                          % [m] draft at rest
Vol = P.m / E.rho_w / 2;                  % [m^3] displaced volume per demihull
B   = 2 * T / tan(V.deadrise);            % [m] waterline beam of the V-section
CB  = Vol / (L * B * T);                  % [-] block coefficient

TL = T/L;  BT = B/T;  BL = B/L;
k  = pi * TL^2;

% Clarke (1982) linear manoeuvring derivatives, prime (bis) system.
Yv_p = -k * (1 + 0.4*CB*BT);
Yr_p = -k * (-0.5 + 2.2*BL - 0.08*BT);
Nv_p = -k * (0.5 + 2.4*TL);
Nr_p = -k * (0.25 + 0.039*BT - 0.56*BL);

%% ---- 2. Dimensionalise, and scale by how much hull is still in the water
% Prime system:  Y_v = 0.5*rho*L^2*U * Y'_v      [N/(m/s)]
%                Y_r = 0.5*rho*L^3*U * Y'_r      [N/(rad/s)]
%                N_v = 0.5*rho*L^3*U * N'_v      [N*m/(m/s)]
%                N_r = 0.5*rho*L^4*U * N'_r      [N*m/(rad/s)]
% Note every one is LINEAR IN U. Hold that thought: the rudder moment is
% QUADRATIC in u. That mismatch -- input gain ~ u^2 against plant damping ~ u --
% is precisely why one fixed set of PID gains cannot span 4 to 30 mph, and it
% is where the 1/U gain schedule comes from.
%
% The area scale factor is the one honest way this model knows the hull is
% climbing out of the water: as SW collapses 12x from displacement to top
% speed, the lateral force the hull can generate collapses with it.
s_area = min(max(H.SW ./ H.SW_disp, 0.02), 1.0);            % [-]
q2 = 0.5 * E.rho_w * U;

Yv = 2 * q2 * L^2 * Yv_p * s_area * P.fac.Yv;               % [N/(m/s)]
Yr = 2 * q2 * L^3 * Yr_p * s_area * P.fac.Yr;               % [N/(rad/s)]
Nv = 2 * q2 * L^3 * Nv_p * s_area * P.fac.Nv;               % [N*m/(m/s)]
Nr = 2 * q2 * L^4 * Nr_p * s_area * P.fac.Nr;               % [N*m/(rad/s)]

Y_lin = Yv*v + Yr*r;                                        % [N]
N_lin = Nv*v + Nr*r;                                        % [N*m]

%% ---- 3. Quadratic cross-flow drag, by strip theory --------------------
% Each transverse strip of hull at station x sees a local lateral velocity
%       v_local(x) = v + r*x
% and drags like a bluff body normal to the flow:
%       dY = -0.5*rho*Cd*T_eff * v_local*|v_local| dx
%       dN = -0.5*rho*Cd*T_eff * x * v_local*|v_local| dx
%
% Doing the integral numerically rather than lumping it is worth the 20 flops:
% it gives the sway force, the yaw moment AND their cross-coupling from a
% SINGLE coefficient Cd, with no extra invented constants. It also captures
% the case that matters -- a hard turn where r*x dominates v at the bow and
% stern but not amidships, so the two ends drag in opposite directions.
Cd_cross = 1.0;                          % [-] V-section bluff-body cross-flow
T_eff = 2 * T * s_area;                  % [m] both demihulls, area-scaled

n_strip = 20;
xs = linspace(-L/2, L/2, n_strip);       % [m] station, +ve forward of CG
dx = L / (n_strip - 1);
vl = v + r * xs;                         % [m/s] local lateral velocity
f  = -0.5 * E.rho_w * Cd_cross * T_eff .* vl .* abs(vl) * dx;   % [N] per strip

Y_quad = sum(f)      * P.fac.Yv;                            % [N]
N_quad = sum(f .* xs) * P.fac.Nr;                           % [N*m]

%% ---- 4. Surge: resistance plus the cost of turning --------------------
% Straight-line resistance from the Savitsky/displacement model.
% RESISTANCE OPPOSES MOTION THROUGH THE WATER, so it must carry the sign of the
% water-relative surge velocity. Writing it as a bare -R_total is wrong the
% moment the boat is moving astern relative to the water -- which happens with
% any following current at low speed, and would push the boat the wrong way.
X_res = -sign(u) * H.R_total;                                % [N]

% Turn-induced drag, from an energy argument rather than an invented X_vv:
% the lateral force does work on the water at rate |Y*v| and the yaw moment at
% rate |N*r|. That power has to come from somewhere, and the only source is
% forward motion, so it appears as a surge force P_loss/u.
%
% This is the term that makes speed sag in a hard turn. If u drops more than
% ~15% during a manoeuvre, the gain schedule -- which is scheduled ON u -- is
% chasing a moving operating point and the linearisation is no longer valid
% mid-turn. eom3dof logs u(t) so that can be checked directly.
Y_tot = Y_lin + Y_quad;
N_tot = N_lin + N_quad;
X_turn = -(abs(Y_tot * v) + abs(N_tot * r)) / U;             % [N]

%% ---- 5. Pack ----------------------------------------------------------
Hf.X = X_res + X_turn;
Hf.Y = Y_tot;
Hf.N = N_tot;

Hf.Yv = Yv;  Hf.Yr = Yr;  Hf.Nv = Nv;  Hf.Nr = Nr;
Hf.Y_lin = Y_lin;   Hf.Y_quad = Y_quad;
Hf.N_lin = N_lin;   Hf.N_quad = N_quad;
Hf.X_res = X_res;   Hf.X_turn = X_turn;
Hf.s_area = s_area; Hf.CB = CB;  Hf.T_draft = T;  Hf.L_wl = L;

end
