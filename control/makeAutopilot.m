function fh = makeAutopilot(P, NM, cfg)
% MAKEAUTOPILOT  Build the cascade controller as a ctrlfun for simOsprey.
%
%   fh = makeAutopilot(P, NM, cfg)
%   S  = simOsprey(x0, tf, fh, P, opts)
%
% cfg fields
%   .wpts        n-by-2 [north east] waypoint list, or [] for heading hold
%   .psi_ref     [rad] fixed heading command when .wpts is empty
%   .U_ref       [m/s] commanded speed
%   .scheduled   true (default) speed-scheduled gains, false = fixed gains
%   .U_fix       [m/s] speed at which fixed gains are frozen (default 6.7)
%   .clamp_vent  true (default) limit deflection to ventilation onset
%   .seed        RNG seed for the sensor noise
%
% ARCHITECTURE
%   waypoints -> LOS guidance -> psi_cmd
%             -> heading PID  -> r_cmd  (limited by r_max_safe(u))
%             -> yaw-rate PI  -> N_cmd
%             -> allocator    -> delta_cmd + differential thrust
%
% REQUIRED FEATURES, AND WHY EACH IS THERE
%   Derivative on MEASUREMENT, not error -- a waypoint switch steps psi_cmd, and
%       derivative-on-error would turn that step into an impulse and slam the
%       rudder to the stop at exactly the moment the boat is already turning.
%   Back-calculation anti-windup -- the allocator reports N_deficit, the moment
%       it could not deliver. That is fed back to unwind the integrator at a
%       rate set by the tracking time constant, so the integrator never charges
%       up against authority that does not exist.
%   Integrator freeze on saturation -- belt and braces alongside back-calc, and
%       it is what stops the ventilation cliff (a sudden 40% authority loss)
%       from winding the loop up before the back-calc catches it.
%   Output rate limiting -- the servo has a finite slew rate, so commanding
%       faster than it can move only builds phase lag with no benefit.

if nargin < 3, cfg = struct(); end
cfg = fill(cfg, 'wpts',       []);
cfg = fill(cfg, 'psi_ref',    0);
cfg = fill(cfg, 'U_ref',      8);
cfg = fill(cfg, 'scheduled',  true);
cfg = fill(cfg, 'U_fix',      6.7);          % 15 mph, the brief's tuning point
cfg = fill(cfg, 'clamp_vent', true);
cfg = fill(cfg, 'seed',       1);
cfg = fill(cfg, 'wc_r',       4.0);
% LOS lookahead. Empty (default) means SPEED-ADAPTIVE, which is the physically
% correct scaling and not a boat-length rule of thumb.
%
% The guidance asks for a heading change and the boat needs ~t90 (about 2 s) to
% deliver it. If the lookahead point sits closer than U*t90 the guidance is
% demanding heading faster than the loop can produce, and the boat weaves about
% the line instead of settling on it -- burning rudder authority it does not
% have. Measured, nominal plant with 15 kt crosswind:
%
%   L_a = 6.4 m (3 LOA)  -> leg error 1.79 m, rudder 8.00 deg RMS (at the clamp)
%   L_a = 20 m  (9 LOA)  -> leg error 0.80 m, rudder 3.82 deg RMS
%   L_a = 32 m  (15 LOA) -> leg error 0.65 m, rudder 3.32 deg RMS
%
% Returns diminish past ~3*U, and a longer lookahead cuts corners harder, so
% the default is L_a = 3*U floored at 2 boat lengths for the low-speed case.
cfg = fill(cfg, 'L_a',        []);

Sn = params_sensors();
dt_c = 1/Sn.f_ctrl;

% Fixed-gain set, frozen at U_fix, for the scheduled-vs-fixed comparison.
G_fix = gainSchedule(cfg.U_fix, NM, P, struct('wc_r', cfg.wc_r));

fh = @autopilotStep;

% =====================================================================
    function [ctrl, mem] = autopilotStep(t, x, mem)

        %% ---- First call: initialise memory --------------------------
        if ~isfield(mem, 'init')
            [~, mem.sens] = sensorModel('init', cfg.seed, [], P);
            mem.I_psi   = 0;      % heading integrator      [rad*s]
            mem.I_r     = 0;      % yaw-rate integrator     [rad]
            mem.psi_m_prev = 0;   % for derivative-on-measurement
            mem.delta_prev = 0;
            mem.gi      = struct('crab_valid', false, 'crab', 0, 'L_a', cfg.L_a);
            mem.I_u     = 0;      % speed integrator
            % Telemetry as PREALLOCATED COLUMNS, not a growing struct array.
            % Growing a struct array element by element reallocates the whole
            % thing every tick -- O(n^2) -- and it was a measurable fraction of
            % the run time before the Monte Carlo made it matter.
            % 20,000 ticks at 50 Hz = 400 s of run time, more than any run here
            % needs, and only ~1.6 MB. Sizing this at 200k would cost 16 MB that
            % gets carried through the controller memory on every single tick.
            nmax = 20000;
            mem.n_log = 0;
            mem.log = struct('t',zeros(nmax,1), 'e_cross',zeros(nmax,1), ...
                             'psi_cmd',zeros(nmax,1), 'r_cmd',zeros(nmax,1), ...
                             'N_cmd',zeros(nmax,1), 'sat',false(nmax,1), ...
                             'U_sched',zeros(nmax,1), 'Kp_N',zeros(nmax,1), ...
                             'r_lim',zeros(nmax,1), 'I_r',zeros(nmax,1));
            mem.init    = true;
        end

        %% ---- 1. Sensors ---------------------------------------------
        [meas, mem.sens] = sensorModel(t, x, mem.sens, P);

        % SENSOR fault: GPS dropout. The last valid fix is held, so the
        % guidance keeps steering to a position estimate that is going stale
        % at the boat's own speed -- 8 m/s of accumulating error per second.
        % The IMU is unaffected, so heading hold survives; it is CROSS-TRACK
        % that degrades.
        if isfield(cfg,'gps_dropout') && ~isempty(cfg.gps_dropout) && ...
           t >= cfg.gps_dropout(1) && t <= cfg.gps_dropout(2)
            if isfield(mem,'gps_frozen')
                meas.X = mem.gps_frozen(1);  meas.Y = mem.gps_frozen(2);
                meas.chi = mem.gps_frozen(3); meas.cog_valid = false;
            end
        else
            mem.gps_frozen = [meas.X, meas.Y, meas.chi];
        end

        U_sched = meas.spd_sched;

        %% ---- 2. Gains -----------------------------------------------
        if cfg.scheduled
            G = gainSchedule(U_sched, NM, P, struct('wc_r', cfg.wc_r));
        else
            G = G_fix;
        end

        %% ---- 3. Guidance --------------------------------------------
        mem.gi.crab       = meas.crab;
        mem.gi.crab_valid = meas.cog_valid;
        if isempty(cfg.L_a)
            mem.gi.L_a = max(3 * U_sched, 2 * P.V.LOA);   % speed-adaptive
        else
            mem.gi.L_a = cfg.L_a;
        end
        if isempty(cfg.wpts)
            psi_cmd = cfg.psi_ref;
            e_cross = 0;
        else
            [psi_cmd, mem.gi] = losGuidance(meas.X, meas.Y, meas.chi, ...
                                            cfg.wpts, mem.gi, P);
            e_cross = mem.gi.e_cross;
        end

        %% ---- 4. Heading PID -> yaw-rate command ---------------------
        e_psi = wrapPi(psi_cmd - meas.psi);

        % Derivative on MEASUREMENT (note the sign: d/dt of -measurement).
        dpsi_m = wrapPi(meas.psi - mem.psi_m_prev) / dt_c;
        mem.psi_m_prev = meas.psi;

        r_cmd_raw = G.Kp_psi * e_psi + mem.I_psi - G.Kd_psi * dpsi_m;

        % HARD yaw-rate ceiling from the roll / blow-over envelope. This is a
        % safety limit, not a tuning knob: above it the 3-DOF model cannot see
        % the failure mode that would actually occur.
        RE = rollEnvelope(max(U_sched, 0.5), P);
        r_lim = RE.r_max_safe;
        r_cmd = max(min(r_cmd_raw, r_lim), -r_lim);

        %% ---- 5. Yaw-rate PI -> yaw moment command -------------------
        % Gains are in MOMENT units (N*m per rad/s). Do not reintroduce an
        % Iz factor here -- see the derivation in gainSchedule.m.
        e_r   = r_cmd - meas.r;
        N_cmd = G.Kp_N * e_r + mem.I_r;                 % [N*m]

        %% ---- 6. Allocation ------------------------------------------
        % Speed loop sets the common-mode thrust.
        e_u    = cfg.U_ref - max(meas.spd, 0);
        T_ff   = hullSteadyState(max(cfg.U_ref,0.5), P).R_total / 2;
        T_base = T_ff + 40*e_u + mem.I_u;
        mem.I_u = mem.I_u + 8*e_u*dt_c;
        mem.I_u = max(min(mem.I_u, 100), -100);

        aopts = struct('clamp_at_vent', cfg.clamp_vent);
        if isfield(cfg,'rudder_failed_at') && ~isempty(cfg.rudder_failed_at) ...
                && t >= cfg.rudder_failed_at
            aopts.rudder_failed = true;
        end
        if isfield(cfg,'u_alloc_fix') && ~isempty(cfg.u_alloc_fix)
            aopts.u_alloc_fix = cfg.u_alloc_fix;
        end
        A = allocate(N_cmd, T_base, max(U_sched,0.1), P, aopts);

        %% ---- 7. Integrator updates with anti-windup -----------------
        % Back-calculation: unwind by the moment we could not deliver. The
        % integrator is in moment units, so N_deficit needs no conversion.
        Tt_track = 0.5;                                     % [s]

        if ~A.saturated
            mem.I_r = mem.I_r + G.Ki_N * e_r * dt_c;
        else
            % Freeze the forward term, keep only the unwinding term.
            mem.I_r = mem.I_r - (A.N_deficit / Tt_track) * dt_c;
        end
        % Clamp to a physically meaningful bound: no point integrating past the
        % largest moment either actuator could ever produce.
        I_lim = max(A.N_rud_max + A.N_thr_max, 1);
        mem.I_r = max(min(mem.I_r, I_lim), -I_lim);

        % Heading integrator: freeze whenever the rate command is clipped by
        % either the envelope limiter or downstream saturation.
        r_clipped = abs(r_cmd_raw) > r_lim;
        if ~r_clipped && ~A.saturated
            mem.I_psi = mem.I_psi + G.Ki_psi * e_psi * dt_c;
        end
        mem.I_psi = max(min(mem.I_psi, 0.5), -0.5);

        %% ---- 8. Output rate limiting --------------------------------
        d_max_step = P.A.rate_max * dt_c;
        d_cmd = mem.delta_prev + ...
                max(min(A.delta_cmd - mem.delta_prev, d_max_step), -d_max_step);
        mem.delta_prev = d_cmd;

        ctrl = struct('delta_cmd', d_cmd, ...
                      'T_cmd_port', A.T_port, ...
                      'T_cmd_stbd', A.T_stbd);

        %% ---- 9. Telemetry -------------------------------------------
        n = mem.n_log + 1;  mem.n_log = n;
        if n <= numel(mem.log.t)
            mem.log.t(n)       = t;
            mem.log.e_cross(n) = e_cross;
            mem.log.psi_cmd(n) = psi_cmd;
            mem.log.r_cmd(n)   = r_cmd;
            mem.log.N_cmd(n)   = N_cmd;
            mem.log.sat(n)     = A.saturated;
            mem.log.U_sched(n) = U_sched;
            mem.log.Kp_N(n)    = G.Kp_N;
            mem.log.r_lim(n)   = r_lim;
            mem.log.I_r(n)     = mem.I_r;
        end
    end
end

% -------------------------------------------------------------------------
function s = fill(s, name, default)
if ~isfield(s, name) || isempty(s.(name))
    if ~strcmp(name,'wpts') || ~isfield(s,'wpts'), s.(name) = default; end
end
end

function a = wrapPi(a)
a = mod(a + pi, 2*pi) - pi;
end
