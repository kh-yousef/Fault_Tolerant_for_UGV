function [mpcController, mpcState] = FTCC_MPC_Design(robot)
% FTCC_MPC_DESIGN - Configures the Linear MPC controller for WMR trajectory
% and formation tracking via Input-Output Feedback Linearization.
%
% Input:
%   robot         - Struct containing robot parameters (maxLinVel, maxAngVel, Ts)
%
% Outputs:
%   mpcController - Configured MATLAB mpc object
%   mpcState      - Initialized mpcstate handle object

if nargin < 1 || isempty(robot)
    robot = struct();
end

% Suppress verbose MPC Toolbox console messages during initialization
mpcverbosity('off');

% Extract parameters with safe defaults
if isfield(robot, 'Ts'),        Ts = robot.Ts;
elseif isfield(robot, 'dt'),    Ts = robot.dt;
else,                           Ts = 0.1; % Default sampling time (s)
end

if isfield(robot, 'maxLinVel'), maxV = robot.maxLinVel;
else,                           maxV = 0.7; % Matches Fig. 12 max v (m/s)
end

if isfield(robot, 'maxAngVel'), maxW = robot.maxAngVel;
else,                           maxW = 2.0; % Default max omega (rad/s)
end

% Feedback-Linearized Discrete-Time State-Space Model
% States:  x_fl = [x; y; theta]
% Virtual Inputs (MV): u_fl = [u_x; u_y; u_theta] (Cartesian & angular velocities)
% This makes both x and y 100% controllable at all headings theta!
A = eye(3);
B = Ts * eye(3);
C = eye(3);
D = zeros(3, 3);

plant = ss(A, B, C, D, Ts);

% Prediction horizon P and Control horizon M
P = 15;
M = 5;
mpcController = mpc(plant, Ts, P, M);

% Configure Manipulated Variable (MV) constraints
% MV(1) = u_x (m/s), MV(2) = u_y (m/s), MV(3) = u_theta (rad/s)
mpcController.MV(1).Min = -maxV;
mpcController.MV(1).Max =  maxV;
mpcController.MV(2).Min = -maxV;
mpcController.MV(2).Max =  maxV;
mpcController.MV(3).Min = -maxW;
mpcController.MV(3).Max =  maxW;

% Rate constraints for smooth actuator commands
mpcController.MV(1).RateMin = -0.5;
mpcController.MV(1).RateMax =  0.5;
mpcController.MV(2).RateMin = -0.5;
mpcController.MV(2).RateMax =  0.5;
mpcController.MV(3).RateMin = -1.5;
mpcController.MV(3).RateMax =  1.5;

% Configure MPC tuning weights: prioritize [x, y] formation tracking
mpcController.Weights.OutputVariables          = [15, 15, 2];
mpcController.Weights.ManipulatedVariables     = [0.1, 0.1, 0.1];
mpcController.Weights.ManipulatedVariablesRate = [0.5, 0.5, 0.2];

% Configure measurement noise model as a valid LTI object (covariance = 0.01*I)
mpcController.Model.Noise = ss(0.1 * eye(3));

% Initialize MPC internal state
mpcState = mpcstate(mpcController);


end