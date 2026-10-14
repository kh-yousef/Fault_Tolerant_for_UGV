function [mpcController, faultDetector, mpcState] = createControlComponents(params)
% CREATECONTROLCOMPONENTS - Initializes the MPC controller, MPC internal
% state, and Two-Stage Kalman Filter (FDD unit) for a WMR.
%
%
% Inputs:
%   params        - Configuration struct containing simulation/robot parameters
%
% Outputs:
%   mpcController - Configured MATLAB mpc object
%   faultDetector - Two_Stage_Kalman_Filter handle object (FDD unit)
%   mpcState      - Initialized mpcstate object for ComputeMPCCommand

if nargin < 1
    params = struct();
end

% 1. Design Linear MPC controller and initialize its state
if exist('FTCC_MPC_Design', 'file')
    [mpcController, mpcState] = FTCC_MPC_Design(params);
elseif exist('configureMPC', 'file')
    mpcController = configureMPC(params);
    mpcState = mpcstate(mpcController);
else
    error('Neither FTCC_MPC_Design.m nor configureMPC.m was found on the MATLAB path.');
end

% 2. Initialize Fault Detection and Diagnosis (FDD) unit
faultDetector = Two_Stage_Kalman_Filter(params);


end