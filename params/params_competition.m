function C = params_competition()
% PARAMS_COMPETITION  Verified PEP26 Autonomy Division rules constraints.
%
% Source: "Rules for 2025-2026 PEP Workforce Development Competition
%          (Autonomy Division)", ASNE / ONR.
%   https://www.navalengineers.org/Portals/16/PEP/2026/Signup/PEP_Rules2025-2026_Autonomy.pdf
% Retrieved 2026-09-04. Event: 14-16 April 2026, Portsmouth City Park,
% Portsmouth VA (Elizabeth River).
%
% ONLY VERIFIED NUMBERS ARE IN THIS FILE. Anything the rules do not state is
% listed under C.not_specified rather than guessed.

%% ---- Race format (verified) -------------------------------------------
C.race_distance_m = 2 * 1609.344;   % [m] 3218.7 -- "uncrewed craft will race
                                    %     for 2 miles"
C.heat_duration_s = 55 * 60;        % [s] "we expect to allow teams 55 minutes
                                    %     in their race-day heats"
% Minimum average speed to cover the distance inside the heat. Note how slack
% this is: the boat does not need to be fast to finish.
C.U_min_required  = C.race_distance_m / C.heat_duration_s;   % [m/s] ~0.98

%% ---- Scoring (verified) -----------------------------------------------
% "Uncrewed craft will earn 10 points for each completed half-mile distance."
% "20 points for first, 16 second, 12 third, 8 fourth, 4 fifth."
C.pts_per_half_mile = 10;
C.n_half_miles      = 4;
C.pts_distance_max  = C.pts_per_half_mile * C.n_half_miles;   % 40
C.pts_placement_1st = 20;
C.pts_race_max      = C.pts_distance_max + C.pts_placement_1st;  % 60

% THE STRATEGIC CONSEQUENCE, AND IT SHOULD DRIVE THE DESIGN:
% simply COMPLETING the 2 miles is worth 40 points; WINNING is worth 20.
% Finishing is worth twice as much as being fastest. A boat that reliably
% completes at a modest speed outscores a fast boat that does not finish, and
% the required average speed is under 1 m/s. This is the quantitative
% justification for every conservative choice in this repo -- clamping rudder
% travel at ventilation onset, limiting commanded yaw rate to the roll
% envelope, and slowing on a steering fault.

%% ---- Craft constraints (verified) -------------------------------------
C.V_batt_max      = 55.5;    % [V]  "total voltage at or below 55.5 Volts"
C.capacity_max_Ah = 500;     % [Ah] "total capacity below 500 Ah"
C.payload_lb      = 30;      % [lb] "Autonomy ... craft will carry a 30-pound
                             %      removable payload" (team supplies it)
C.payload_kg      = C.payload_lb * 0.45359237;   % [kg] 13.61
C.launch_time_s   = 5 * 60;  % [s]  ramp to open water

%% ---- Requirements that bear on the control system (verified) ----------
% Rule 21, Autonomous: "Teams must demonstrate how the kill switch is engaged
% when navigation information (like GPS data) is missing or corrupted."
%
% THIS IS A DESIGN REQUIREMENT, NOT A NICE-TO-HAVE, AND IT CONTRADICTS THE
% OBVIOUS ENGINEERING INSTINCT. The failure-mode study found the boat rides
% out a 5 s GPS dropout comfortably (2.42 m cross-track) by dead-reckoning on
% the IMU. The rules require the opposite behaviour: on loss or corruption of
% navigation data the craft must KILL, not coast. Design the dropout response
% as a kill, and treat the dead-reckoning result as evidence that the kill is
% safe to trigger, not as a reason to avoid it.
C.gps_loss_requires_kill = true;

% Rule 19: positive buoyancy -- must float when fully flooded.
% Rule 13: catamarans need a towing bridle across both hulls.
% Rule  7: main power disconnect through which all propulsion current passes.
% Rule  9: fuse in the battery circuit for 24-55 V systems.
C.needs_positive_buoyancy = true;
C.needs_tow_bridle        = true;
C.needs_main_disconnect   = true;
C.needs_battery_fuse      = true;

% Demo video gate: 200 m or 2 minutes of open-water operation with a thrust
% measurement documented, due 1 April 2026, required to get a race slot.
C.demo_distance_m = 200;
C.demo_time_s     = 120;

%% ---- NOT specified by the rules ---------------------------------------
% Searched the Autonomy rules PDF, the PEP26 and PEP25 pages, and a competing
% team's published white paper. None of these state the course layout. It
% appears to be set on site / at the race-day briefing.
C.not_specified = { ...
    'course shape and waypoint coordinates', ...
    'number, spacing and size of marks or buoys', ...
    'turn angles and required turn radii at marks', ...
    'waypoint arrival tolerance', ...
    'number of laps', ...
    'required standoff from obstacles or other craft', ...
    'speed limit on the water', ...
    'station-keeping or loitering tasks'};

end
