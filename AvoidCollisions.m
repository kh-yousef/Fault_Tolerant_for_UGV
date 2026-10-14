function [linVel, angVel] = AvoidCollisions(currentPose, neighbors, linVel, angVel, robot)
% AVOIDCOLLISIONS - Reactive obstacle/collision avoidance based on virtual
% repulsive forces and input-output feedback linearization mapping.
%
% Inputs:
%   currentPose - 3x1 vector [x; y; theta] of the current robot
%   neighbors   - 2xM (or 3xM) matrix of neighbor/obstacle positions [x_n; y_n; ...]
%   linVel      - Nominal linear velocity command v (m/s)
%   angVel      - Nominal angular velocity command omega (rad/s)
%   robot       - Struct/object with fields: safetyDist, axleLength, maxLinVel, maxAngVel
%
% Outputs:
%   linVel      - Adjusted and saturated linear velocity v (m/s)
%   angVel      - Adjusted and saturated angular velocity omega (rad/s)

repulse = [0; 0];
theta = currentPose(3);

% Repulsive gain (tuneable parameter)
k_rep = 0.3;

for n = 1:size(neighbors, 2)
    % Vector pointing FROM neighbor n TO current robot (away from obstacle)
    dx = currentPose(1) - neighbors(1, n);
    dy = currentPose(2) - neighbors(2, n);
    dist = norm([dx; dy]);

    % Apply repulsive force if neighbor is within the safety zone
    if dist < robot.safetyDist && dist > 1e-6
        % Unit vector pointing away from neighbor n
        dir_away = [dx; dy] / dist;

        % Repulsive force magnitude (increases as dist -> 0)
        force_mag = k_rep * (1/dist - 1/robot.safetyDist);

        repulse = repulse + force_mag * dir_away;
    end
end

% Map Cartesian repulsive velocity vector to [Delta_v; Delta_omega]
% using the nonholonomic look-ahead Jacobian J(theta)
J = [cos(theta), -robot.axleLength * sin(theta);
    sin(theta),  robot.axleLength * cos(theta)];
adjustment = J \ repulse;

% ADD the repulsive adjustment to move AWAY from obstacles
linVel = linVel + adjustment(1);
angVel = angVel + adjustment(2);

% Saturate velocities to physical robot limits
linVel = max(min(linVel, robot.maxLinVel), -robot.maxLinVel);
angVel = max(min(angVel, robot.maxAngVel), -robot.maxAngVel);


end