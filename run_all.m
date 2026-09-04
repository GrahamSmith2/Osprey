% RUN_ALL  Single entry point for the Osprey rudder-sizing study.
%
% Usage (from anywhere):
%   addpath(genpath('C:\Osprey')); run_all
%
% Runs the sanity checks first and STOPS if any fail -- no result from this
% repo is meaningful if the checks below it are broken.
%
% BUILD STATUS. This repo is being assembled layer by layer, each layer
% validated before the next is written. Stages marked [pending] are not yet
% implemented and are listed here so the entry point never silently omits them.
%
%   [done]    params            -- all constants, all declared uncertainties
%   [done]    hullSteadyState   -- wetted area / trim / resistance vs speed
%                                  validated to ~6% against the 13.4 m/s log
%   [done]    rudderForces      -- lift, ventilation w/ hysteresis, hinge moment
%   [pending] propForces        -- KT(J) thrust + ESC spool-up lag
%   [pending] hullForces        -- Yv, Yr, Nv, Nr scaled by SW(u)
%   [pending] eom3dof           -- rigid body, Coriolis, Munk, quadratic damping
%   [pending] servoDynamics     -- rate limit, lag, backlash, saturation
%   [pending] sensors           -- GPS, IMU, COG-vs-heading, ZOH, compute delay
%   [pending] rollEnvelope      -- blow-over / hook check -> r_max_safe(u)
%   [pending] control           -- LOS guidance, heading PID, yaw-rate PI, allocator
%   [pending] analyses 1-6      -- sizing sweep, servo spec, gain schedule,
%                                  Monte Carlo, full course, failure modes

clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(here));
if ~exist(fullfile(here, 'results'), 'dir'), mkdir(fullfile(here, 'results')); end

%% ---- 0. Sanity checks --------------------------------------------------
run_tests();

%% ---- 1. Diagnostic: hull regime vs speed ------------------------------
P = buildParams();
u = linspace(0.5, 35.8, 300);
H = hullSteadyState(u, P);

fig = figure('Position', [100 100 1100 780], 'Color', 'w');

subplot(2,2,1);
plot(u, H.SW, 'LineWidth', 1.6); grid on;
xlabel('Forward speed u  [m/s]'); ylabel('Wetted area S_W  [m^2]');
title(sprintf('Hydrodynamic wetted area (collapses %.0f\\times)', H.SW_disp/H.SW(end)));

subplot(2,2,2);
plot(u, H.tau_run_deg, 'LineWidth', 1.6); grid on;
xlabel('Forward speed u  [m/s]'); ylabel('Running trim \tau  [deg]');
title('Solved running trim (not fixed at 1.5\circ)');

subplot(2,2,3);
plot(u, H.R_total, 'LineWidth', 1.6); hold on;
plot(u, H.R_fric, '--', u, H.R_induced, '--', u, H.D_aero, '--', 'LineWidth', 1.1);
grid on; legend('total','friction','induced','aero', 'Location','northwest');
xlabel('Forward speed u  [m/s]'); ylabel('Resistance  [N]');
title('Resistance breakdown');

subplot(2,2,4);
plot(u, H.f_aero*100, 'LineWidth', 1.6); grid on;
xlabel('Forward speed u  [m/s]'); ylabel('Aero lift / weight  [%]');
title('Tunnel lift fraction (36% at 35.8 m/s by calibration)');

exportgraphics(fig, fullfile(here,'results','hull_regime_vs_speed.png'), 'Resolution', 150);

%% ---- 2. Diagnostic: rudder authority and the ventilation cliff --------
fig2 = figure('Position', [100 100 1100 420], 'Color', 'w');
d = linspace(0, deg2rad(30), 200);
speeds = [5 10 13.4 20];

subplot(1,2,1); hold on; grid on;
for U = speeds
    N = zeros(size(d)); vs = 0;
    for i = 1:numel(d)
        Rf = rudderForces(d(i), U, 0, 0, vs, P); vs = Rf.vent_state; N(i) = Rf.N;
    end
    plot(rad2deg(d), N, 'LineWidth', 1.6, 'DisplayName', sprintf('u = %.1f m/s', U));
end
xlabel('Rudder deflection \delta  [deg]'); ylabel('Yaw moment N_{rud}  [N\cdotm]');
title('Control authority is NON-MONOTONIC past ventilation onset');
legend('Location','northwest');

subplot(1,2,2);
uu = linspace(2, 35.8, 200);
sig = (P.E.p_atm + P.E.rho_w*P.E.g*P.R.h_sub/2 - P.E.p_vap) ./ (0.5*P.E.rho_w*uu.^2);
semilogy(uu, sig, 'LineWidth', 1.6); hold on; grid on;
yline(0.5, 'r--', 'model invalid below \sigma = 0.5');
xlabel('Forward speed u  [m/s]'); ylabel('Cavitation number \sigma  [-]');
title('Cavitation ceiling');

exportgraphics(fig2, fullfile(here,'results','rudder_authority_and_cavitation.png'), 'Resolution', 150);

%% ---- 3. Headline caveats ----------------------------------------------
U_sig = sqrt((P.E.p_atm + P.E.rho_w*P.E.g*P.R.h_sub/2 - P.E.p_vap)/(0.5*0.5*P.E.rho_w));
fprintf('\n--- Validity limits ---\n');
fprintf('Froude number: Fn = %.2f at 13.4 m/s, Fn = %.2f at 35.8 m/s.\n', ...
    13.4/sqrt(P.E.g*P.V.LOA), 35.8/sqrt(P.E.g*P.V.LOA));
fprintf('Standard manoeuvring regressions are fitted at Fn < 0.3. Hull\n');
fprintf('derivatives here are placeholders swept +/-5x, not estimates.\n\n');
fprintf('MODEL NOT VALID ABOVE U = %.1f m/s (%.0f mph): cavitation number\n', ...
    U_sig, U_sig*2.237);
fprintf('falls below 0.5. sigma = %.3f at the 35.8 m/s design speed.\n\n', ...
    (P.E.p_atm + P.E.rho_w*P.E.g*P.R.h_sub/2 - P.E.p_vap)/(0.5*P.E.rho_w*35.8^2));

fprintf('Figures written to %s\n', fullfile(here, 'results'));
