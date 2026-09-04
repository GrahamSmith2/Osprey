function H = hullSteadyState(u, P)
% HULLSTEADYSTATE  Wetted area, lift split and resistance vs forward speed.
%
%   H = hullSteadyState(u, P)   u may be scalar or vector [m/s]
%
% WHY THIS FILE EXISTS
% The manoeuvring derivatives Yv, Nr, ... are not constants: they scale with the
% WETTED AREA, and a planing hull's wetted area collapses by roughly an order of
% magnitude between displacement speed and full plane. That collapse is the
% physical reason the boat's yaw damping changes character across the speed
% range, and it is why a single set of PID gains cannot work from 4 to 30 mph.
% So SW(u) is computed here once and every derivative downstream is scaled by it.
%
% PROVENANCE -- READ THIS
% The prompt asked for the design report's Appendix A.1 routine to be ported.
% That appendix was not available, so this is an INDEPENDENT implementation of
% the standard Savitsky (1964) prismatic planing method, not a port. Where the
% report supplies a checkable number (aero lift = 36% of weight at 35.8 m/s)
% this file is calibrated to reproduce it. If Appendix A.1 differs, replace the
% body of this function; the returned struct is the only interface.
% See ASSUMPTIONS.md #A2.
%
% RETURNS (fields are the same size as u)
%   H.SW          [m^2] total hydrodynamic wetted area (both sponsons)
%   H.lambda      [-]   mean wetted length / pad beam, per sponson
%   H.L_aero      [N]   tunnel aerodynamic lift
%   H.f_aero      [-]   aero lift as a fraction of weight
%   H.R_total     [N]   total resistance
%   H.regime      [-]   0 = displacement, 1 = fully planing, in-between = blend
%   H.U_transition[m/s] blend midpoint (scalar)

V = P.V;  E = P.E;
% Speed MAGNITUDE. Everything here (wetted area, trim, resistance magnitude) is
% even in u -- a boat dragged backwards through the water has the same wetted
% area. The DIRECTION of the resistance force is applied by the caller
% (hullForces), which knows the sign of the water-relative velocity.
u = abs(u(:).');                  % row, so all outputs are rows
W = P.m * E.g;                    % [N] weight

%% ---- 1. Tunnel aerodynamic lift ---------------------------------------
% Ram-wing tunnel between the sponsons. Planform is tunnel width x chord.
% CL_alpha is CALIBRATED so that at the design speed the aero lift equals the
% reported 36% of weight -- this is the one aero anchor the report provides.
S_tunnel = V.beam_tunnel * V.chord_tunnel;                 % [m^2] = 0.841
q_design = 0.5 * E.rho_a * V.U_design^2;                   % [Pa]
CL_needed = V.aero_lift_frac_design * W / (q_design * S_tunnel);
CL_alpha_tunnel = CL_needed / V.a_aero;                    % [1/rad] ~7.5

q_a   = 0.5 * E.rho_a * u.^2;                              % [Pa]
L_aero = q_a * S_tunnel * CL_alpha_tunnel * V.a_aero;      % [N]
L_aero = min(L_aero, W);                                   % cannot exceed weight
                                                           % (past that it flies)

% Tunnel induced + profile drag. AR of the tunnel planform.
AR_t   = V.beam_tunnel / V.chord_tunnel;
CL_t   = CL_alpha_tunnel * V.a_aero;
CD_t   = 0.02 + CL_t^2 / (pi * AR_t * 0.7);
D_aero = q_a * S_tunnel * CD_t;                            % [N]

%% ---- 2. Planing branch: Savitsky (1964) -------------------------------
% Hydrodynamic lift must carry whatever the air does not.
L_hydro = max(W - L_aero, 0.02*W);                         % [N]

% TWO sponsons share the load, each a prismatic planing surface of pad beam b.
b    = V.b_pad;                                            % [m] per sponson
L_per = L_hydro / 2;                                       % [N]

tau_deg  = rad2deg(V.a_trim);                              % [deg] running trim
beta_deg = rad2deg(V.deadrise);                            % [deg] deadrise

Cv = u ./ sqrt(E.g * b);                                   % [-] beam Froude no.
CL_req = L_per ./ (0.5 * E.rho_w * u.^2 * b^2);            % [-] required CL_beta

% Savitsky, deadrise correction:  CL_beta = CL0 - 0.0065*beta*CL0^0.60
% and                             CL0     = tau^1.1 * (0.0120*lam^0.5
%                                                    + 0.0055*lam^2.5/Cv^2)
%
% TRIM IS NOT CONSTANT WITH SPEED. Solving at a fixed 1.5 deg trim gives
% lambda ~ 17-20 at 13.4 m/s, i.e. a wetted length of 2.4-2.9 m on a 2.13 m
% boat -- physically impossible. The 1.5 deg figure is the DESIGN-POINT trim at
% 35.8 m/s, where the tunnel already carries 36% of the weight. At lower speed
% the hull must trim BOW-UP to make the same lift on the same pads.
%
% So the solve is two-branch, with lambda capped at the physical hull length:
%   (a) if the required lift is achievable at design trim within lambda_max,
%       solve for lambda at fixed trim  -- the boat is comfortably on the pads;
%   (b) otherwise pin lambda = lambda_max and solve for the TRIM required.
% Branch (b) is what produces the resistance hump, because induced drag is
% L_hydro*tan(tau) and tau climbs steeply as speed drops.
% See ASSUMPTIONS.md #A3.
lambda_max = 0.95 * V.LOA / b;                              % [-] ~14.4
tau_max    = 12;                                            % [deg] past this it
                                                            % is not planing at all
lambda   = nan(size(u));
tau_run  = nan(size(u));
for k = 1:numel(u)
    if u(k) < 0.5 || ~isfinite(CL_req(k))
        continue                                            % handled by blend
    end
    % Invert the deadrise correction to get the equivalent zero-deadrise CL0.
    CL0 = invertDeadrise(CL_req(k), beta_deg);

    % Savitsky CL0 as a function of (lambda, tau), for this speed.
    sav = @(lam, td) td^1.1 * (0.0120*sqrt(lam) + 0.0055*lam.^2.5 / Cv(k)^2);

    if sav(lambda_max, tau_deg) >= CL0
        % (a) enough lift is available at design trim -- solve for lambda.
        lambda(k)  = bisect(@(lam) sav(lam, tau_deg) - CL0, 1e-4, lambda_max);
        tau_run(k) = tau_deg;
    else
        % (b) pinned at full hull length -- solve for the trim required.
        lambda(k)  = lambda_max;
        tau_run(k) = bisect(@(td) sav(lambda_max, td) - CL0, tau_deg, 60);
        if ~isfinite(tau_run(k)) || tau_run(k) > tau_max
            tau_run(k) = tau_max;      % beyond this the displacement branch owns it
        end
    end
end

% Wetted area of a prismatic surface: mean wetted length x beam, divided by
% cos(deadrise) because the V-bottom is longer than its projection.
SW_plane = 2 * lambda * b^2 / cos(V.deadrise);             % [m^2] both sponsons

%% ---- 3. Displacement branch -------------------------------------------
% Below planing the boat floats on buoyancy. Draft from a prismatic V-section
% carrying half the displaced volume per demihull; wetted surface from the
% Denny-Mumford approximation S ~ 1.7*L*T + Vol/T.
Vol_total = P.m / E.rho_w;                                 % [m^3]
Vol_half  = Vol_total / 2;
L_wl      = 0.85 * V.LOA;                                  % [m] waterline length
A_sect    = Vol_half / L_wl;                               % [m^2] midship section
T_draft   = sqrt(A_sect * tan(V.deadrise));                % [m] ~0.07
SW_disp   = 2 * (1.7 * L_wl * T_draft + Vol_half / T_draft);  % [m^2]

%% ---- 4. Blend the two regimes -----------------------------------------
% A hard switch would put a discontinuity in the derivatives and wreck the
% integrator. Smoothstep over a transition band centred on planing onset.
U_lo = 0.75 * V.U_plane_on;                                % [m/s] 6.0
U_hi = 1.25 * V.U_plane_on;                                % [m/s] 10.0
s = smoothstep((u - U_lo) / (U_hi - U_lo));                % 0 -> 1

SW_plane(~isfinite(SW_plane)) = SW_disp;                   % low-speed guard
SW = (1 - s) .* SW_disp + s .* SW_plane;                   % [m^2]
SW = min(SW, SW_disp);                                     % never exceed floating

%% ---- 5. Resistance -----------------------------------------------------
% Friction: ITTC-1957 line on the blended wetted area.
L_char = max(lambda * b, 0.1);  L_char(~isfinite(L_char)) = L_wl;
L_char = (1 - s) .* L_wl + s .* L_char;                    % [m]
Re = max(u .* L_char / E.nu_w, 1e4);
Cf = 0.075 ./ (log10(Re) - 2).^2;
R_fric = 0.5 * E.rho_w * u.^2 .* SW .* Cf * 1.15;          % [N] 1.15 form factor

% Induced/pressure drag of the planing surfaces: lift x tan(running trim).
% Uses the SOLVED trim, not the design trim -- this is what makes the hump
% appear, since tau climbs bow-up as speed falls.
tau_eff = tau_run;  tau_eff(~isfinite(tau_eff)) = tau_max;
R_induced = L_hydro .* tand(tau_eff);                      % [N]

% Residuary/wave drag, only meaningful in and below the transition. Crude hump
% at Fn ~ 0.5 so the model does not under-predict the hump the boat must climb.
Fn = u / sqrt(E.g * V.LOA);
R_wave = (1 - s) .* (0.06 * W * exp(-((Fn - 0.5)/0.35).^2));

R_total = R_fric + R_induced + R_wave + D_aero;            % [N]

%% ---- 6. Pack ----------------------------------------------------------
H.u            = u;
H.SW           = SW;
H.SW_plane     = SW_plane;
H.SW_disp      = SW_disp;
H.lambda       = lambda;
H.tau_run_deg  = tau_eff;      % [deg] solved running trim vs speed
H.lambda_max   = lambda_max;
H.L_aero       = L_aero;
H.f_aero       = L_aero / W;
H.L_hydro      = L_hydro;
H.R_fric       = R_fric;
H.R_induced    = R_induced;
H.R_wave       = R_wave;
H.D_aero       = D_aero;
H.R_total      = R_total;
H.regime       = s;
H.U_transition = V.U_plane_on;
H.T_draft      = T_draft;
H.CL_alpha_tunnel = CL_alpha_tunnel;
H.Fn           = Fn;

end

% =========================================================================
function CL0 = invertDeadrise(CLb, beta_deg)
% INVERTDEADRISE  Solve CLb = CL0 - 0.0065*beta*CL0^0.60 for CL0.
% Monotonic increasing in CL0 over the physical range, so bisection is safe.
f = @(c) c - 0.0065 * beta_deg * c.^0.60 - CLb;
CL0 = bisect(f, max(CLb, 1e-8), 10 * CLb + 1);
end

% -------------------------------------------------------------------------
function x = bisect(f, lo, hi)
% BISECT  Plain bisection. No toolbox, no fzero, deterministic iteration count.
flo = f(lo);  fhi = f(hi);
if flo * fhi > 0
    x = NaN;  return              % no bracketed root: caller handles via blend
end
for i = 1:80
    mid = 0.5*(lo+hi);  fm = f(mid);
    if flo * fm <= 0, hi = mid; fhi = fm; else, lo = mid; flo = fm; end
end
x = 0.5*(lo+hi);
end

% -------------------------------------------------------------------------
function y = smoothstep(t)
% SMOOTHSTEP  C1-continuous 0->1 ramp. Continuous first derivative matters:
% a kink here would show up as a spurious impulse in the yaw damping.
t = min(max(t, 0), 1);
y = t.^2 .* (3 - 2*t);
end
