function simData = initializeSimulationData(timeVector, numFollowers)
% INITIALIZESIMULATIONDATA - Preallocates the data logging structure for
% the Fault-Tolerant Cooperative Control (FTCC) simulation.
%
% Inputs:
%   timeVector   - 1xN vector of simulation time steps (s)
%   numFollowers - Number of follower robots in the team
%
% Output:
%   simData      - Preallocated struct for trajectories, velocities, and FDD estimates

numSteps = length(timeVector);

simData = struct(...
    'time',                 timeVector, ...                     % Time vector (s)
    'timeVector',           timeVector, ...                     % Alias for compatibility
    'leaderTrajectory',     zeros(3, numSteps), ...             % Leader [x; y; theta]
    'followerTrajectories', zeros(3, numFollowers, numSteps), ... % Followers [x; y; theta]
    'leaderLinVel',         zeros(1, numSteps), ...             % Leader linear velocity v (m/s) 
    'followerLinVel',       zeros(numFollowers, numSteps), ...  % Followers linear velocity v (m/s) 
    'trueGammaF2',          zeros(2, numSteps), ...             % True [gamma_R; gamma_L] of Follower 2 
    'estGamma',             zeros(2, numFollowers, numSteps), ... % Estimated [gamma_R; gamma_L] per follower 
    'faultEstimates',       zeros(numFollowers, numSteps), ...  % Max estimated gamma per follower
    'f2LeftWheelSpeed',     zeros(1, numSteps), ...             % Left motor speed omega_L of Follower 2 
    'f1DesiredAngleDeg',    zeros(1, numSteps) ...              % Desired formation angle phi_d of Follower 1 
);


end