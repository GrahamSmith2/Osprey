function NM = identifyNomoto(P, speeds, delta_step, opts)
% IDENTIFYNOMOTO  Fit the first-order Nomoto model to the nonlinear simulation.
%
%   NM = identifyNomoto(P, speeds, delta_step, opts)
%
% This is system identification performed ON MY OWN MODEL. That is worth being
% honest about: it does not validate the model against reality, it extracts the
% model's own low-order behaviour so a controller can be designed against it.
% The value is that the SCALING LAW it confirms (below) is robust even when the
% hull derivatives feeding it are wrong by 5x.
%
% ===========================================================================
% THE DERIVATION
% ===========================================================================
% Linearise the sway/yaw subsystem about straight-line running at speed U,
% i.e. about (u,v,r) = (U,0,0), with small delta. Surge decouples to first
% order because it is even in v and r. What is left is:
%
%   (m - Y_vdot) vdot = Y_v v + (Y_r - m*U) r + Y_d delta            ... (1)
%   (Iz - N_rdot) rdot = N_v v + (N_r - m*x_G*U) r + N_d delta       ... (2)
%
% (x_G = 0 here because the body frame origin IS the CG, so that term drops.)
%
% Write as a 2-state system in (v, r) and eliminate v. Taking Laplace
% transforms of (1) and (2) with a = m - Y_vdot, b = Iz - N_rdot:
%
%   (a*s - Y_v) v = (Y_r - m*U) r + Y_d delta
%   (b*s - N_r) r = N_v v + N_d delta
%
% Solve the first for v and substitute into the second:
%
%   (b*s - N_r) r = N_v * [ (Y_r - m*U) r + Y_d delta ] / (a*s - Y_v) + N_d delta
%
% Multiply through by (a*s - Y_v) and collect:
%
%   [ (b*s - N_r)(a*s - Y_v) - N_v(Y_r - m*U) ] r
%                                  = [ N_d(a*s - Y_v) + N_v*Y_d ] delta
%
% which is the SECOND-order Nomoto form
%
%   T1*T2*sdd(r) + (T1+T2)*sd(r) + r = K*( delta + T3*sd(delta) )
%
% The first-order Nomoto model drops the fast sway mode by setting
% T = T1 + T2 - T3, giving
%
%       T * rdot + r = K * delta                                     ... (3)
%
% ---------------------------------------------------------------------------
% THE SCALING LAW -- THIS IS THE ACTUAL DELIVERABLE
% ---------------------------------------------------------------------------
% Non-dimensionalise with K' = K*L/U and T' = T*U/L. Then
%
%       K = K' * U / L          T = T' * L / U
%
% Why those scalings hold: every hull derivative in the coefficients above is
% LINEAR in U (rudder-free hull damping ~ rho*L^2*U), while the rudder terms
% N_d and Y_d are QUADRATIC in U. The ratio that forms K therefore carries one
% net power of U; the ratio that forms T carries one inverse power.
%
% Consequence for the controller. The open-loop heading transfer function is
%
%       psi/delta = K / ( s*(1 + T*s) )
%
% A proportional heading loop has loop gain Kp*K/(s(1+Ts)). Holding the
% crossover frequency and phase margin CONSTANT across speed requires
%
%       Kp * K = const   ->   Kp ~ 1/U
%       Td     ~ T       ->   Td ~ 1/U
%
% and the integral gain, which must scale with crossover squared, follows
% Ki ~ 1/U^2 if the bandwidth is held fixed in absolute terms.
%
% THAT RESULT SURVIVES BEING WRONG BY 5x ON THE HULL DERIVATIVES, because the
% derivatives set the VALUES of K' and T' but not their U-dependence. The shape
% of the schedule is trustworthy; the numbers multiplying it are not.
% ===========================================================================
%
% RETURNS
%   NM.speeds, NM.K, NM.T          dimensional fits at each speed  [1/s, s]
%   NM.Kp_nd, NM.Tp_nd             non-dimensional K', T' at each speed
%   NM.K_prime, NM.T_prime         mean K', T' across speeds
%   NM.fit_rms                     relative RMS of the first-order fit

if nargin < 2 || isempty(speeds),     speeds = [4 6 8 10 13]; end
if nargin < 3 || isempty(delta_step), delta_step = deg2rad(5); end
if nargin < 4, opts = struct(); end
t_win = getf(opts, 't_win', 6);       % [s] fit window
dt    = getf(opts, 'dt', 0.002);

% A SMALL step is essential. delta_step must stay below ventilation onset
% (nominally 10 deg) or the "linear model" is being fitted to a latched,
% non-monotonic plant and the result is meaningless.
assert(delta_step < P.R.delta_vent_on, ...
    'identifyNomoto: step of %.1f deg is at or past ventilation onset (%.1f deg)', ...
    rad2deg(delta_step), rad2deg(P.R.delta_vent_on));

L = P.V.LOA;
n = numel(speeds);
NM.speeds = speeds(:).';
NM.K = zeros(1,n);  NM.T = zeros(1,n);  NM.fit_rms = zeros(1,n);
NM.r_ss = zeros(1,n);

for i = 1:n
    U  = speeds(i);
    Tt = hullSteadyState(U, P).R_total / 2;
    x0 = [U;0;0;0;0;0;Tt;Tt;0];
    ctrl = struct('delta_cmd', delta_step, 'T_cmd_port', Tt, 'T_cmd_stbd', Tt);
    S = simOsprey(x0, t_win, ctrl, P, struct('dt', dt));

    t = S.t;  r = S.r;

    % Steady state from the tail of the window.
    tail  = t > 0.7*t_win;
    r_ss  = mean(r(tail));
    NM.r_ss(i) = r_ss;
    NM.K(i)    = r_ss / delta_step;                 % [1/s]

    % Fit T by 1-D search on the first-order step response
    %       r(t) = r_ss * (1 - exp(-t/T))
    % Golden-section on log(T) -- no toolbox, and log-space keeps the search
    % well conditioned across the decade of T values seen from 4 to 13 m/s.
    obj = @(TT) sum((r - r_ss*(1 - exp(-t/TT))).^2);
    NM.T(i) = goldenMin(obj, 0.02, 20);

    pred = r_ss * (1 - exp(-t/NM.T(i)));
    NM.fit_rms(i) = sqrt(mean((r - pred).^2)) / max(abs(r_ss), eps);
end

%% ---- Non-dimensionalise ------------------------------------------------
NM.Kp_nd = NM.K .* L ./ NM.speeds;      % K' = K*L/U   [-]
NM.Tp_nd = NM.T .* NM.speeds ./ L;      % T' = T*U/L   [-]

NM.K_prime = mean(NM.Kp_nd);
NM.T_prime = mean(NM.Tp_nd);
NM.K_prime_spread = (max(NM.Kp_nd) - min(NM.Kp_nd)) / abs(NM.K_prime);
NM.T_prime_spread = (max(NM.Tp_nd) - min(NM.Tp_nd)) / abs(NM.T_prime);
NM.L = L;
NM.delta_step = delta_step;

% If K' and T' are NOT roughly constant across speed, the assumed scaling is
% not holding in the nonlinear model and a 1/U schedule will not linearise the
% loop. That is a result worth knowing, so it is returned rather than hidden.
NM.scaling_holds = (NM.K_prime_spread < 0.35) && (NM.T_prime_spread < 0.35);
end

% -------------------------------------------------------------------------
function x = goldenMin(f, lo, hi)
% GOLDENMIN  Golden-section minimisation in log space. Plain MATLAB.
a = log(lo); b = log(hi);
gr = (sqrt(5)-1)/2;
c = b - gr*(b-a);  d = a + gr*(b-a);
for k = 1:80
    if f(exp(c)) < f(exp(d)), b = d; else, a = c; end
    c = b - gr*(b-a);  d = a + gr*(b-a);
end
x = exp((a+b)/2);
end

% -------------------------------------------------------------------------
function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
