function newPose = UpdateRobotPose(currentPose, linVel, angVel, dt)
% UPDATEROBOTPOSE - Integrates the nonholonomic WMR kinematic model
% over one time step dt.
%
% Inputs:
%   currentPose - 3x1 pose vector [x; y; theta] (m, m, rad)
%   linVel      - Linear velocity v (m/s)
%   angVel      - Angular velocity omega (rad/s)
%   dt          - Sampling time step (s)
%
% Output:
%   newPose     - Updated 3x1 pose vector [x; y; theta] with theta in [-pi, pi]

currentPose = currentPose(:);
theta = currentPose(3);

% Use exact arc integration when turning, or Euler step when moving straight
if abs(angVel) > 1e-6
    newTheta = theta + angVel * dt;
    newX = currentPose(1) + (linVel / angVel) * (sin(newTheta) - sin(theta));
    newY = currentPose(2) - (linVel / angVel) * (cos(newTheta) - cos(theta));
else
    newX = currentPose(1) + linVel * cos(theta) * dt;
    newY = currentPose(2) + linVel * sin(theta) * dt;
    newTheta = theta + angVel * dt;
end

% Normalize heading angle to [-pi, pi]
newTheta = atan2(sin(newTheta), cos(newTheta));

newPose = [newX; newY; newTheta];


end