function [position, heading, velocity] = GenerateReferenceTrajectory(t, gamma, phaseOffset)
% GENERATEREFERENCETRAJECTORY - Generates the leader's circular reference
% trajectory and supports motion re-coordination under actuator faults.
%
% Inputs:
%   t           - Current simulation time (s) or effective trajectory time
%   gamma       - (Optional) Fault severity factor in [0, 0.65) for motion
%                 re-coordination. Default = 0 (fault-free).
%   phaseOffset - (Optional) Initial angle offset on the circle (rad).
%                 Default = 0 
%                 Use -pi/2 to start at the bottom of the circle 
%
% Outputs:
%   position    - 2x1 reference position [x_r; y_r] (m)
%   heading     - Reference tangent heading angle phi_r (rad)
%   velocity    - 2x1 reference Cartesian velocity [dx_r; dy_r] (m/s)

if nargin < 2 || isempty(gamma)
    gamma = 0; % Fault-free by default
end
if nargin < 3 || isempty(phaseOffset)
    phaseOffset = 0; 
end

% Nominal angular rate 
nominalRate = 0.25; % rad/s

% If non-severe fault (0 < gamma < 0.65), scale trajectory speed by (1 - gamma)
effectiveRate = nominalRate * (1 - gamma);

% Instantaneous angle along the circular path
angle = nominalRate * t + phaseOffset;
if gamma > 0 && gamma < 0.65
    % Scale angular progression rate under degraded mode
    angle = effectiveRate * t + phaseOffset;
end

x_r = cos(angle) - 0.1;
y_r = sin(angle) - 0.6;
position = [x_r; y_r];

% Reference Cartesian velocities: time derivatives scaled by (1 - gamma)
dx_r = -effectiveRate * sin(angle);
dy_r =  effectiveRate * cos(angle);
velocity = [dx_r; dy_r];

% Reference heading angle tangent to the circular trajectory
heading = atan2(dy_r, dx_r);


end