function S = simOsprey(x0, tf, ctrlfun, P, opts)
% SIMOSPREY  Fixed-step RK4 integration of the 3-DOF model.
%
%   S = simOsprey(x0, tf, ctrlfun, P, opts)
%
% ctrlfun  either a STRUCT of constant commands (open-loop run), or a function
%          handle  ctrl = ctrlfun(t, x, mem)  called at the CONTROLLER rate.
%
% WHY FIXED-STEP RK4 AND NOT ode45
% The controller is a discrete, fixed-rate device with a one-sample compute
% delay. ode45 chooses its own step size and will happily step over a
% controller tick, silently smoothing the zero-order hold and the delay into
% something the real autopilot never does. A digital controller's phase lag is
% a first-order effect on achievable bandwidth, so it has to be simulated as
% what it is. Fixed-step RK4 inside, ZOH controller outside.
%
% THREE THINGS LIVE IN THE STEPPER, NOT THE DERIVATIVE
%   1. Ventilation latch  -- discrete; held fixed across all four RK4 stages so
%                            the stages cannot disagree about the branch.
%   2. Cable backlash     -- non-differentiable hysteresis between servo and blade.
%   3. Controller ZOH     -- commands held constant between ticks.

if nargin < 5, opts = struct(); end
dt      = getf(opts, 'dt',      0.002);   % [s] integrator step (500 Hz)
f_ctrl  = getf(opts, 'f_ctrl',  50);      % [Hz] controller rate
delay_n = getf(opts, 'delay_n', 1);       % [-] compute delay, in controller ticks

n_sub = max(round((1/f_ctrl)/dt), 1);     % integrator steps per controller tick
dt    = (1/f_ctrl)/n_sub;                 % make them commensurate exactly
n_tick = ceil(tf * f_ctrl);
N      = n_tick * n_sub;

nx = numel(x0);
S.t     = (0:N)' * dt;
S.x     = zeros(N+1, nx);
S.x(1,:) = x0(:).';

S.delta_cmd  = zeros(N+1,1);
S.delta_blade= zeros(N+1,1);
S.vent       = zeros(N+1,1);
S.beta       = zeros(N+1,1);
S.crab       = zeros(N+1,1);
S.N_munk     = zeros(N+1,1);
S.N_rud      = zeros(N+1,1);
S.N_hull     = zeros(N+1,1);
S.N_prop     = zeros(N+1,1);
S.tau_servo  = zeros(N+1,1);
S.sigma      = zeros(N+1,1);
S.T_max      = zeros(N+1,1);

x           = x0(:);
vent        = 0;
delta_blade = x(9);
mem         = struct();                    % controller memory (integrators etc.)

% Command queue implements the one-sample compute delay: a command computed at
% tick k is not applied until tick k+delay_n, exactly as on a real autopilot.
if isstruct(ctrlfun)
    ctrl_now = ctrlfun;
else
    ctrl_now = struct('delta_cmd',0, 'T_cmd_port',0, 'T_cmd_stbd',0);
end
queue = repmat(ctrl_now, 1, max(delay_n,1));

k = 1;
for it = 1:n_tick
    t_tick = (it-1) * n_sub * dt;

    %% ---- Controller tick (ZOH + compute delay) ------------------------
    if ~isstruct(ctrlfun)
        [ctrl_new, mem] = ctrlfun(t_tick, x, mem);
        queue = [queue(2:end), ctrl_new];   %#ok<AGROW>  short fixed-length queue
    end
    ctrl_now = queue(1);

    %% ---- Integrate one controller period at the inner step ------------
    for j = 1:n_sub
        t = t_tick + (j-1)*dt;

        % --- Backlash: servo -> blade. Standard play model. The blade only
        % moves once the servo has taken up half the free play in the current
        % direction; inside the deadband the blade is mechanically disconnected.
        b = P.A.backlash;
        if b > 0
            if     x(9) - delta_blade >  b/2, delta_blade = x(9) - b/2;
            elseif x(9) - delta_blade < -b/2, delta_blade = x(9) + b/2;
            end
        else
            delta_blade = x(9);
        end

        % --- Log at the start of the step
        [~, D] = eom3dof(t, x, ctrl_now, vent, P, delta_blade);
        S.x(k,:)          = x.';
        S.delta_cmd(k)    = ctrl_now.delta_cmd;
        S.delta_blade(k)  = delta_blade;
        S.vent(k)         = vent;
        S.beta(k)         = D.beta;
        S.crab(k)         = D.crab;
        S.N_munk(k)       = D.N_munk;
        S.N_rud(k)        = D.N_rud;
        S.N_hull(k)       = D.N_hull;
        S.N_prop(k)       = D.N_prop;
        S.tau_servo(k)    = D.Rf.tau_servo;
        S.sigma(k)        = D.sigma;
        S.T_max(k)        = D.T_max;

        % --- RK4, ventilation latch frozen across all four stages
        f1 = eom3dof(t,        x,             ctrl_now, vent, P, delta_blade);
        f2 = eom3dof(t+dt/2,   x+dt/2*f1,     ctrl_now, vent, P, delta_blade);
        f3 = eom3dof(t+dt/2,   x+dt/2*f2,     ctrl_now, vent, P, delta_blade);
        f4 = eom3dof(t+dt,     x+dt*f3,       ctrl_now, vent, P, delta_blade);
        x  = x + dt/6*(f1 + 2*f2 + 2*f3 + f4);

        % --- Update the latch ONCE, after the step is complete
        vent = D.vent_state;

        k = k + 1;
    end
end
% Populate the FINAL sample. Without this the last index holds a state but zero
% diagnostics, and anything that reads S.<field>(end) -- steady-turn closure
% checks especially -- silently reads a zeroed rudder angle and gets nonsense.
if P.A.backlash > 0
    if     x(9) - delta_blade >  P.A.backlash/2, delta_blade = x(9) - P.A.backlash/2;
    elseif x(9) - delta_blade < -P.A.backlash/2, delta_blade = x(9) + P.A.backlash/2;
    end
else
    delta_blade = x(9);
end
[~, D] = eom3dof(S.t(k), x, ctrl_now, vent, P, delta_blade);
S.x(k,:)         = x.';
S.delta_cmd(k)   = ctrl_now.delta_cmd;
S.delta_blade(k) = delta_blade;
S.vent(k)        = vent;
S.beta(k)        = D.beta;      S.crab(k)   = D.crab;
S.N_munk(k)      = D.N_munk;    S.N_rud(k)  = D.N_rud;
S.N_hull(k)      = D.N_hull;    S.N_prop(k) = D.N_prop;
S.tau_servo(k)   = D.Rf.tau_servo;
S.sigma(k)       = D.sigma;     S.T_max(k)  = D.T_max;
S.n = k;

% Trim logging arrays to the steps actually taken.
S.t = S.t(1:k);  S.x = S.x(1:k,:);
fn = {'delta_cmd','delta_blade','vent','beta','crab','N_munk','N_rud', ...
      'N_hull','N_prop','tau_servo','sigma','T_max'};
for i = 1:numel(fn), S.(fn{i}) = S.(fn{i})(1:k); end

% Convenience aliases
S.u = S.x(:,1);  S.v = S.x(:,2);  S.r = S.x(:,3);
S.X = S.x(:,4);  S.Y = S.x(:,5);  S.psi = S.x(:,6);
S.T_p = S.x(:,7); S.T_s = S.x(:,8); S.delta = S.x(:,9);
S.dt = dt;  S.f_ctrl = f_ctrl;
end

% -------------------------------------------------------------------------
function val = getf(s, name, default)
if isfield(s, name), val = s.(name); else, val = default; end
end
