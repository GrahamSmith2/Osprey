function Pf = propForces(T_port, T_stbd, u, P)
% PROPFORCES  Surge force and yaw moment from the twin props.
%
%   Pf = propForces(T_port, T_stbd, u, P)
%   T_port, T_stbd  [N] ACTUAL delivered thrust (the lagged states, not commands)
%
% Twin independent ESCs, so differential thrust is a genuine second actuator:
%
%       N_prop = (T_port - T_stbd) * y_p
%
% with y_p = 0.299 m. THIS IS THE ACTUATOR THAT MATTERS AT LOW SPEED. Rudder
% yaw moment goes as u^2, so at the start line or loitering it is essentially
% zero, while differential thrust is available at u = 0. The crossover is
% computed in analysis/ and drives the control allocator.
%
% MODELLING CHOICE. Open-water KT(J) data for the Graupner K-series 76 mm prop
% is not in hand, so the spec's stated fallback is used: the controller commands
% THRUST, a first-order lag (state, integrated in eom3dof) represents ESC and
% rotor spool-up, and this function only converts delivered thrust into forces.
% The KT map below is used solely for the AVAILABLE-THRUST CEILING, via
% propMaxThrust. See ASSUMPTIONS.md #A8.
%
% Sign convention: T > 0 is forward thrust. Positive N is a turn to starboard,
% so MORE port thrust than starboard gives positive N -- hence (T_port - T_stbd).

V = P.V;

Pf.X = T_port + T_stbd;                        % [N]
Pf.N = (T_port - T_stbd) * V.y_p;              % [N*m]

% Yaw from thrust asymmetry produces no direct sway force: both thrust vectors
% are aligned with the hull centreline. (A real shaft has a small angle; that is
% a second-order effect and is not modelled.)
Pf.Y = 0;

Pf.T_max = propMaxThrust(u, P);                % [N] per-motor ceiling, this speed
end
