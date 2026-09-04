function SW = sizingSweep(P, areas, spans, speeds, opts)
% SIZINGSWEEP  Rudder area / submerged-span sweep -> the feasible sizing band.
%
%   SW = sizingSweep(P, areas, spans, speeds, opts)
%
% THE SIZING IS BOUNDED FROM BOTH ENDS. A single "recommended area" would be a
% fiction given +/-5x hull-derivative uncertainty. What this produces is a BAND:
%
%   MINIMUM area  -- set by the turn-rate requirement at the LOWEST autonomous
%                    speed, because rudder authority goes as u^2 and low speed
%                    is where it runs out.
%   MAXIMUM area  -- set by whichever binds first of:
%                      (a) added drag,
%                      (b) required servo torque against the 74 kg-cm stall,
%                      (c) the blow-over / sponson-unloading envelope,
%                      (d) the cavitation validity ceiling.
%
% For each (area, span, speed) the sweep runs the nonlinear model to a settled
% turn at BOTH the mechanical stop AND the ventilation-limited useful angle,
% because those give very different answers and only one of them is real.
%
% Returned arrays are indexed [i_area, i_span, i_speed].

if nargin < 2 || isempty(areas),  areas  = 2.25e-3 * [0.5 0.75 1 1.25 1.5 2 2.5 3]; end
if nargin < 3 || isempty(spans),  spans  = [0.075 0.110]; end
if nargin < 4 || isempty(speeds), speeds = [2 5 10 15 20 30]; end
if nargin < 5, opts = struct(); end
t_sim = getf(opts, 't_sim', 20);
dt    = getf(opts, 'dt', 0.005);
verbose = getf(opts, 'verbose', true);

na = numel(areas); ns = numel(spans); nu = numel(speeds);
z = nan(na, ns, nu);
SW = struct('areas',areas,'spans',spans,'speeds',speeds, ...
            'R_turn',z,'R_turn_LOA',z,'r_ss',z,'t90',z,'u_ss',z,'u_sag',z, ...
            'drag_add',z,'M_hinge',z,'tau_servo',z,'tau_frac',z, ...
            'rate_req',z,'vent',z,'sigma',z, ...
            'R_turn_useful',z,'r_ss_useful',z,'t90_useful',z);

for ia = 1:na
  for is = 1:ns
    % Override BOTH area and span. buildParams recomputes chord and aspect
    % ratio from them, so a bigger area at fixed span correctly means a fatter,
    % LOWER-aspect-ratio blade -- which is the trade that matters.
    ov = struct('R_A_r', areas(ia), 'R_h_sub', spans(is));
    Pk = buildParams(P.S, ov);

    for iu = 1:nu
      U  = speeds(iu);
      H0 = hullSteadyState(U, Pk);
      Tt = H0.R_total / 2;
      x0 = [U;0;0;0;0;0;Tt;Tt;0];

      % --- (1) mechanical stop
      c1 = struct('delta_cmd', Pk.R.delta_max, 'T_cmd_port', Tt, 'T_cmd_stbd', Tt);
      S1 = simOsprey(x0, t_sim, c1, Pk, struct('dt', dt));
      n1 = numel(S1.t);

      SW.r_ss(ia,is,iu)   = abs(S1.r(n1));
      SW.u_ss(ia,is,iu)   = S1.u(n1);
      SW.u_sag(ia,is,iu)  = 100*(S1.u(n1)-U)/U;
      SW.R_turn(ia,is,iu) = hypot(S1.u(n1),S1.v(n1)) / max(abs(S1.r(n1)),1e-9);
      SW.R_turn_LOA(ia,is,iu) = SW.R_turn(ia,is,iu) / Pk.V.LOA;
      SW.t90(ia,is,iu)    = timeTo(S1, pi/2);
      SW.vent(ia,is,iu)   = S1.vent(n1);
      SW.sigma(ia,is,iu)  = S1.sigma(n1);

      % Hinge moment and servo demand at the worst point of the manoeuvre.
      SW.tau_servo(ia,is,iu) = max(S1.tau_servo);
      SW.tau_frac(ia,is,iu)  = max(S1.tau_servo) / Pk.A.tau_stall;
      Rf = rudderForces(Pk.R.delta_max, U, 0, 0, 1, Pk);
      SW.M_hinge(ia,is,iu)   = Rf.M_h;

      % Added drag: rudder drag at full deflection as a fraction of hull
      % resistance -- the penalty paid for carrying a bigger blade.
      SW.drag_add(ia,is,iu)  = abs(Rf.F_D) / H0.R_total * 100;

      % Servo rate demand: enough to slew stop-to-stop in 0.5 s, the usual
      % rule for a steering servo that must answer a guidance step.
      SW.rate_req(ia,is,iu)  = 2*Pk.R.delta_max / 0.5;

      % --- (2) ventilation-limited useful angle
      d_use = min(Pk.R.delta_vent_on, Pk.R.delta_max);
      c2 = struct('delta_cmd', d_use, 'T_cmd_port', Tt, 'T_cmd_stbd', Tt);
      S2 = simOsprey(x0, t_sim, c2, Pk, struct('dt', dt));
      n2 = numel(S2.t);
      SW.r_ss_useful(ia,is,iu)   = abs(S2.r(n2));
      SW.R_turn_useful(ia,is,iu) = hypot(S2.u(n2),S2.v(n2))/max(abs(S2.r(n2)),1e-9) / Pk.V.LOA;
      SW.t90_useful(ia,is,iu)    = timeTo(S2, pi/2);
    end
    if verbose
        fprintf('  sizingSweep: area %d/%d, span %d/%d done\n', ia, na, is, ns);
    end
  end
end
end

% -------------------------------------------------------------------------
function t = timeTo(S, target)
% TIMETO  First time |psi| reaches target. NaN if never.
j = find(abs(S.psi) >= target, 1);
if isempty(j), t = NaN; else, t = S.t(j); end
end

function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
