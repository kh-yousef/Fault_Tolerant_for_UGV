function [linVel, angVel, state] = ComputeMPCCommand(controller, state, currentPose, refPose, lookAheadDist)
% COMPUTEMPCCOMMAND - Calculates linear and angular velocity commands using
% Linear MPC combined with Input-Output Feedback Linearization.
%
% Inputs:
%   controller    - MATLAB mpc object (from FTCC_MPC_Design)
%   state         - mpcstate handle object (updated in-place)
%   currentPose   - 3x1 vector [x; y; theta] (m, m, rad)
%   refPose       - 2x1 or 3x1 vector [x_ref; y_ref; (theta_ref)]
%   lookAheadDist - (Optional) Feedback linearization look-ahead distance 'a' (m)
%
% Outputs:
%   linVel        - Commanded linear velocity v (m/s)
%   angVel        - Commanded angular velocity omega (rad/s)
%   state         - Updated mpcstate object

if nargin < 5 || isempty(lookAheadDist)
    lookAheadDist = 0.15; % Default look-ahead distance a (m)
end

% Ensure column vectors initially
currentPose = currentPose(:);
refPose     = refPose(:);
theta       = currentPose(3);

% If refPose only has [x_ref; y_ref], compute desired heading toward reference
if length(refPose) < 3
    dx = refPose(1) - currentPose(1);
    dy = refPose(2) - currentPose(2);
    if hypot(dx, dy) > 1e-3
        refHeading = atan2(dy, dx);
    else
        refHeading = theta;
    end
    refPose = [refPose(1:2); refHeading];
end

% Wrap reference heading so (theta_ref - theta) is continuous in [-pi, pi]
headingErr = atan2(sin(refPose(3) - theta), cos(refPose(3) - theta));
refPose(3) = theta + headingErr;

% Determine number of outputs and inputs expected by the MPC controller
ny = size(controller.Model.Plant.C, 1);
nu = length(controller.MV);

% Prepare plant measurement y_meas and reference r_ref as 1 x ny ROW vectors
y_meas = currentPose(1:ny)';
r_ref  = refPose(1:ny)';

% Compute optimal virtual control action via Linear MPC
% NOTE: mpcmove returns at most 2 outputs: [mv, info]
options = mpcmoveopt;
mv = mpcmove(controller, state, y_meas, r_ref, [], options);

% Map Linear MPC virtual control inputs to physical WMR [v; omega]
% via Input-Output Feedback Linearization
if nu == 3
    % 3-MV Feedback-Linearized Plant from FTCC_MPC_Design:
    % mv = [u_x; u_y; u_theta]
    u_x     = mv(1);
    u_y     = mv(2);
    u_theta = mv(3);

    % Project Cartesian velocity onto heading & apply look-ahead inverse
    linVel =  u_x * cos(theta) + u_y * sin(theta);
    angVel = (-u_x * sin(theta) + u_y * cos(theta)) / max(lookAheadDist, 0.10) + 0.4 * u_theta;
    
    maxV = controller.MV(1).Max;
    minV = controller.MV(1).Min;
    maxW = controller.MV(3).Max;
    minW = controller.MV(3).Min;
elseif nu == 2 && ny == 2
    % 2-MV Cartesian Integrator Plant: mv = [u_x; u_y]
    u_x = mv(1);
    u_y = mv(2);
    linVel =  u_x * cos(theta) + u_y * sin(theta);
    angVel = (-u_x * sin(theta) + u_y * cos(theta)) / max(lookAheadDist, 0.10);
    
    maxV = controller.MV(1).Max;
    minV = controller.MV(1).Min;
    maxW = controller.MV(2).Max;
    minW = controller.MV(2).Min;
else
    % Fallback for 2-MV, 3-output linearized plant at theta = 0
    ex = refPose(1) - currentPose(1);
    ey = refPose(2) - currentPose(2);
    u_x = mv(1) * cos(theta) + 0.8 * ex;
    u_y = mv(1) * sin(theta) + 0.8 * ey;
    linVel =  u_x * cos(theta) + u_y * sin(theta);
    angVel = (-u_x * sin(theta) + u_y * cos(theta)) / max(lookAheadDist, 0.15) + 0.5 * mv(2);
    
    maxV = controller.MV(1).Max;
    minV = controller.MV(1).Min;
    maxW = controller.MV(2).Max;
    minW = controller.MV(2).Min;
end

% Constrain velocities to physical robot limits
if ~isempty(maxV) && isfinite(maxV), linVel = min(linVel, maxV); end
if ~isempty(minV) && isfinite(minV), linVel = max(linVel, minV); end
if ~isempty(maxW) && isfinite(maxW), angVel = min(angVel, maxW); end
if ~isempty(minW) && isfinite(minW), angVel = max(angVel, minW); end


end