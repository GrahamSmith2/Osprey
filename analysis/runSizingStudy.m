function ST = runSizingStudy(P, NM, spec, outdir)
% RUNSIZINGSTUDY  Produce the rudder sizing band, the servo spec, and figures.
%
%   ST = runSizingStudy(P, NM, spec, outdir)
%
% spec fields (the REQUIREMENT, which is an input, not something to invent)
%   .U_min        [m/s] lowest autonomous speed the turn spec must hold at
%   .R_turn_max   [LOA] required steady turn radius at U_min, in boat lengths
%   .t90_max      [s]   required time for a 90 deg heading change at U_min
%   .servo_margin [-]   required stall-torque margin (2.0 = use at most 50%)
%   .drag_max     [%]   max acceptable rudder drag as a fraction of hull drag
%
% THE SIZING VARIABLE IS SPAN, NOT AREA.
% Sweeping area at fixed span means growing the chord, which collapses the
% aspect ratio. Since the authority that matters is the product A_r*CL_alpha,
% and CL_alpha falls roughly as fast as A_r rises, area alone buys almost
% nothing: 30x the area at fixed span yields 1.32x the authority. Growing SPAN
% at fixed chord raises A_r and CL_alpha together and yields 5.4x for 3.3x area.
% This function therefore sweeps submerged span at fixed chord.

if nargin < 3 || isempty(spec)
    spec = struct('U_min',5, 'R_turn_max',5.0, 't90_max',5.0, ...
                  'servo_margin',2.0, 'drag_max',6.0);
end
if nargin < 4, outdir = fullfile(fileparts(fileparts(mfilename('fullpath'))),'results'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

c_fixed = P.R.c;                                   % hold chord at the as-built value
spans   = [0.060 0.075 0.090 0.110 0.130 0.150 0.180 0.220];
n = numel(spans);

ST.spans = spans;  ST.chord = c_fixed;  ST.spec = spec;
[ST.A_r, ST.AR, ST.R_min, ST.t90_min, ST.R_hi, ST.drag_pct, ...
 ST.tau_kgcm_LE, ST.tau_frac_LE, ST.tau_kgcm_bal] = deal(nan(1,n));

U_hi = 10;                                          % second evaluation speed

for i = 1:n
    h  = spans(i);
    A  = h * c_fixed;
    Pk = buildParams(P.S, struct('R_A_r', A, 'R_h_sub', h));
    ST.A_r(i) = A;  ST.AR(i) = h / c_fixed;

    % --- turn performance at the spec speed and at 10 m/s
    for k = 1:2
        U  = [spec.U_min, U_hi];  U = U(k);
        Tt = hullSteadyState(U, Pk).R_total/2;
        ct = struct('delta_cmd', Pk.R.delta_max, 'T_cmd_port', Tt, 'T_cmd_stbd', Tt);
        S  = simOsprey([U;0;0;0;0;0;Tt;Tt;0], 25, ct, Pk, struct('dt',0.005));
        m  = numel(S.t);
        R  = hypot(S.u(m),S.v(m)) / max(abs(S.r(m)),1e-9) / Pk.V.LOA;
        if k == 1
            ST.R_min(i) = R;
            j = find(abs(S.psi) >= pi/2, 1);
            if ~isempty(j), ST.t90_min(i) = S.t(j); end
        else
            ST.R_hi(i) = R;
        end
    end

    % --- drag penalty at 10 m/s, full deflection
    H0 = hullSteadyState(U_hi, Pk);
    Rd = rudderForces(Pk.R.delta_max, U_hi, 0, 0, 1, Pk);
    ST.drag_pct(i) = 100 * abs(Rd.F_D) / H0.R_total;

    % --- servo torque. Evaluated at 20 m/s, which is the cavitation validity
    % ceiling and therefore the fastest speed at which this model may be used.
    % Worst case: stock at the leading edge (unbalanced blade).
    Pw = Pk;  Pw.R.x_stock_frac = 0;
    Rw = rudderForces(Pk.R.delta_max, 20, 0, 0, 1, Pw);
    ST.tau_kgcm_LE(i) = Rw.tau_servo / 0.0980665;
    ST.tau_frac_LE(i) = Rw.tau_servo / Pk.A.tau_stall;
    % Balanced blade (stock at the CoP) for comparison.
    Rb = rudderForces(Pk.R.delta_max, 20, 0, 0, 1, Pk);
    ST.tau_kgcm_bal(i) = Rb.tau_servo / 0.0980665;
end

%% ---- Apply the bounds --------------------------------------------------
ok_turn  = (ST.R_min <= spec.R_turn_max) & (ST.t90_min <= spec.t90_max);
ok_servo = ST.tau_frac_LE <= 1/spec.servo_margin;
ok_drag  = ST.drag_pct    <= spec.drag_max;
ok_all   = ok_turn & ok_servo & ok_drag;

ST.ok_turn = ok_turn; ST.ok_servo = ok_servo; ST.ok_drag = ok_drag; ST.ok_all = ok_all;
if any(ok_all)
    ST.span_min = min(spans(ok_all));
    ST.span_max = max(spans(ok_all));
    ST.A_min    = ST.span_min * c_fixed;
    ST.A_max    = ST.span_max * c_fixed;
    ST.feasible = true;
else
    ST.feasible = false;
    ST.span_min = NaN; ST.span_max = NaN; ST.A_min = NaN; ST.A_max = NaN;
end
ST.baseline_in_band = ST.feasible && ...
    (P.R.h_sub >= ST.span_min) && (P.R.h_sub <= ST.span_max);

%% ---- Figure ------------------------------------------------------------
fig = figure('Position',[80 80 1180 760],'Color','w');

subplot(2,2,1);
plot(spans*1000, ST.R_min, 'o-','LineWidth',1.8); hold on;
plot(spans*1000, ST.R_hi, 's--','LineWidth',1.3);
yline(spec.R_turn_max,'r--',sprintf('spec: %.1f LOA',spec.R_turn_max));
xline(P.R.h_sub*1000,'k:','as-built');
grid on; xlabel('Submerged span h_{sub}  [mm]'); ylabel('Steady turn radius  [LOA]');
legend(sprintf('u = %.0f m/s',spec.U_min), sprintf('u = %.0f m/s',U_hi));
title('Turn radius sets the MINIMUM');

subplot(2,2,2);
plot(spans*1000, ST.tau_frac_LE*100, 'o-','LineWidth',1.8); hold on;
plot(spans*1000, ST.tau_kgcm_bal*0+0, 's--','LineWidth',1.3);
yline(100/spec.servo_margin,'r--',sprintf('spec: %.0f%% of stall',100/spec.servo_margin));
xline(P.R.h_sub*1000,'k:','as-built');
grid on; xlabel('Submerged span h_{sub}  [mm]'); ylabel('Servo torque  [% of 74 kg\cdotcm stall]');
legend('stock at LE (worst case)','stock balanced at 25%c','Location','northwest');
title('Servo torque sets the MAXIMUM');

subplot(2,2,3);
plot(spans*1000, ST.drag_pct, 'o-','LineWidth',1.8); hold on;
yline(spec.drag_max,'r--','drag spec'); xline(P.R.h_sub*1000,'k:','as-built');
grid on; xlabel('Submerged span h_{sub}  [mm]'); ylabel('Rudder drag  [% of hull resistance]');
title('Drag penalty at 10 m/s, full deflection');

subplot(2,2,4);
b = double([ok_turn; ok_servo; ok_drag]);
imagesc(spans*1000, 1:3, b); colormap([0.85 0.4 0.35; 0.45 0.72 0.45]);
set(gca,'YTick',1:3,'YTickLabel',{'turn rate','servo torque','drag'});
xlabel('Submerged span h_{sub}  [mm]');
title('Feasibility (green = satisfied)');
hold on; xline(P.R.h_sub*1000,'k:','LineWidth',2);

exportgraphics(fig, fullfile(outdir,'rudder_sizing_band.png'), 'Resolution', 150);
ST.figure = fullfile(outdir,'rudder_sizing_band.png');
end
