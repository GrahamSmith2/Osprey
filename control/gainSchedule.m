function G = gainSchedule(U, NM, P, opts)
% GAINSCHEDULE  Speed-scheduled cascade gains, DERIVED from the Nomoto fit.
%
%   G = gainSchedule(U, NM, P, opts)   U [m/s] scalar or vector
%
% Not grid-searched. Every gain below comes out of the Nomoto model identified
% in analysis/identifyNomoto.m, using pole cancellation and a target crossover.
%
% ===========================================================================
% THE ALGEBRA
% ===========================================================================
% Nomoto:   T*rdot + r = K*delta,   K = K'*U/L,   T = T'*L/U
%
% INNER LOOP (yaw rate).  Plant  r/delta = K/(1 + T*s).
%   PI controller  C(s) = Kp_r * (1 + 1/(Ti*s)).
%   Choose Ti = T so the PI zero cancels the plant pole. Open loop becomes
%
%       L(s) = Kp_r*K / (T*s)          -> a pure integrator
%
%   which crosses over at  wc_r = Kp_r*K/T  with 90 deg phase margin before
%   the actuator and computation delays are counted. Therefore
%
%       Kp_r = wc_r * T / K            Ki_r = Kp_r / T = wc_r / K
%
% OUTER LOOP (heading).  With the inner loop closed it is first order,
% r/r_cmd = 1/(1 + s/wc_r), and psi_dot = r, so
%
%       psi/r_cmd = 1 / ( s*(1 + s/wc_r) )
%
%   A proportional heading gain Kp_psi crosses over at ~Kp_psi provided
%   Kp_psi << wc_r. Taking Kp_psi = wc_r/4 gives about 76 deg phase margin
%   and clean timescale separation between the loops.
%
% ---------------------------------------------------------------------------
% SCALING -- AND A CORRECTION TO THE EXPECTED 1/U LAW
% ---------------------------------------------------------------------------
% Substituting K = K'U/L and T = T'L/U:
%
%       Kp_r = wc_r * (T'L/U) / (K'U/L) = wc_r * T' * L^2 / (K' * U^2)
%                                                            ~~~~~~~~
%       Ki_r = wc_r / (K'U/L)           = wc_r * L / (K' * U)
%
%       Kp_psi = wc_r/4                 -- INDEPENDENT of speed
%
% So in THIS cascade architecture the inner proportional gain scales as 1/U^2,
% not 1/U, and the outer heading gain does not scale at all.
%
% The 1/U law quoted in the brief is correct, but for a DIFFERENT architecture:
% a single-loop heading-to-rudder PID, psi/delta = K/(s(1+Ts)). There
% Kp = wc/K ~ 1/U and Td ~ T ~ 1/U, exactly as stated. Both results are right;
% they belong to different loop structures. The cascade picks up the extra
% power of U because the inner loop is regulating a rate whose plant gain is
% itself proportional to U, on top of the u^2 rudder authority.
%
% What survives the +/-5x hull-derivative uncertainty is the EXPONENT, not the
% coefficient: K' and T' set the constant multiplying the schedule, and they
% are the uncertain part. The shape is trustworthy; the magnitude is not.
% ===========================================================================

if nargin < 4, opts = struct(); end
L = P.V.LOA;
U = max(U(:).', 0.5);

%% ---- Target inner-loop crossover --------------------------------------
% Bounded by everything between the command and the water:
%   servo lag           tau_servo_lag        (nominal 40 ms)
%   ZOH at f_ctrl       ~ 0.5/f_ctrl          (10 ms at 50 Hz)
%   one-sample delay    1/f_ctrl              (20 ms at 50 Hz)
% Total transport delay ~70 ms. A pure delay costs wc*tau radians of phase, so
% budgeting 30 deg (0.52 rad) of the phase margin to delay caps wc_r at
% about 7.5 rad/s. Backlash and the servo rate limit tighten it further, so the
% default target is deliberately below that ceiling.
Sn = params_sensors();
tau_delay = P.A.tau_servo_lag + 0.5/Sn.f_ctrl + Sn.delay_n/Sn.f_ctrl;
wc_r_ceiling = 0.52 / tau_delay;                       % [rad/s]

wc_r = getf(opts, 'wc_r', 4.0);                        % [rad/s] design target
wc_r = min(wc_r, 0.8 * wc_r_ceiling);

%% ---- Nomoto values at each speed --------------------------------------
K = NM.K_prime * U / L;                                % [1/s]
T = NM.T_prime * L ./ U;                               % [s]

%% ---- Inner loop: yaw rate ----------------------------------------------
% The angle-domain gain, i.e. rate error -> RUDDER ANGLE:
G.Kp_r = wc_r * T ./ K;                                % [rad / (rad/s)]
G.Ti_r = T;                                            % [s]
G.Ki_r = G.Kp_r ./ G.Ti_r;
%
% BUT the rate loop does not command an angle -- it commands a YAW MOMENT,
% which the allocator then splits between rudder and differential thrust. The
% conversion is not optional bookkeeping: the allocator divides by dN/ddelta,
% which carries a full factor of u^2. Handing it an angle-domain gain and
% letting Iz do the unit conversion double-counts that u^2 and makes the loop
% progressively under-damped as speed rises (30 deg step: 9% overshoot at
% 4 m/s degrading to 60% at 13 m/s).
%
% So the moment-domain gains are derived explicitly here:
%
%   dN/ddelta = 0.5*rho*u^2 * A_r * CL_alpha * |x_r|      [N*m/rad]
%   Kp_N      = Kp_r * dN/ddelta                          [N*m/(rad/s)]
%
% Note the scaling that falls out: Kp_r ~ 1/U^2 and dN/ddelta ~ U^2, so
%
%       Kp_N is INDEPENDENT OF SPEED.
%
% That is physically right, and a good check on the algebra: the yaw moment
% needed to produce a given yaw acceleration is a property of the boat's
% inertia, not of how fast it happens to be going. All of the speed dependence
% belongs downstream, in converting that moment into a rudder angle.
AR_eff_g   = P.R.k_surface * (P.R.h_sub / P.R.c);
CLa_g      = 1.8*pi*AR_eff_g / (1.8 + sqrt(AR_eff_g^2 + 4)) * P.R.CLa_fac;
G.dN_ddelta = 0.5 * P.E.rho_w * U.^2 * P.R.A_r * CLa_g * abs(P.x_r);

G.Kp_N = G.Kp_r .* G.dN_ddelta;                        % [N*m/(rad/s)]
G.Ki_N = G.Kp_N ./ G.Ti_r;                             % [N*m/rad]

%% ---- Outer loop: heading ----------------------------------------------
G.Kp_psi = (wc_r/4) * ones(size(U));                   % [(rad/s)/rad]

% Integral action on heading exists only to trim out a steady disturbance
% (current, crosswind, rudder mis-zero, magnetometer bias).
%
% The textbook Ki = Kp^2/10 (integral corner at wc/10) is TOO FAST here and
% produces a lightly damped ~20 s mode: a 30 deg step overshoots to 41 deg and
% then rings for half a minute. The cause is that the plant is not the clean
% Nomoto model the gains were derived from -- during the step the Munk moment
% reaches 135% of the rudder moment and acts as negative damping in sideslip,
% eroding the margin the design assumed. Backing the integral corner off to
% wc/30 restores it. See ASSUMPTIONS.md #A16.
G.Ki_psi = 0.033 * G.Kp_psi.^2;

% DERIVATIVE ON HEADING IS SET TO ZERO, DELIBERATELY.
% Placing a useful lead zero at wc_psi/3 would need Kd_psi ~ 3. The fused
% IMU/mag heading carries 1.5 deg of noise; differentiated at the 50 Hz loop
% rate that is 1.85 rad/s of noise, so Kd_psi = 3 would inject 5.6 rad/s of
% command noise against a useful r_cmd of about 0.5 rad/s. Unusable.
%
% This is exactly why the architecture is a CASCADE. The damping comes from the
% inner loop, which closes on the IMU RATE signal -- 0.004 rad/s noise, some
% 460x cleaner than differentiated heading. Trying to recover damping in the
% outer loop instead would mean filtering the derivative, and the filter lag
% would cost more phase than the lead gained.
G.Kd_psi = zeros(size(U));

%% ---- Equivalent single-loop PID (for comparison and for autopilots that
%%      only expose one loop, e.g. a heading-to-steering PID) -------------
% psi/delta = K/(s(1+Ts)); PD with Td = T cancels the pole, giving wc = Kp*K.
G.single_Kp = wc_r/4 ./ K;                             % ~ 1/U
G.single_Td = T;                                       % ~ 1/U
G.single_Ki = 0.1 * (wc_r/4) ./ K;

%% ---- Bookkeeping -------------------------------------------------------
G.U            = U;
G.K            = K;
G.T            = T;
G.wc_r         = wc_r;
G.wc_r_ceiling = wc_r_ceiling;
G.wc_psi       = wc_r/4;
G.tau_delay    = tau_delay;
G.K_prime      = NM.K_prime;
G.T_prime      = NM.T_prime;
end

% -------------------------------------------------------------------------
function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
