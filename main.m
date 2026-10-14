% RUN_FTCC_SIMULATION

clear; clc; close all;

%% 1. Verify Required Toolboxes
assert(~isempty(ver('mpc')),     'Please install the Model Predictive Control Toolbox.');
assert(~isempty(ver('control')), 'Please install the Control System Toolbox.');
mpcverbosity('off');

%% 2. Select Experimental Scenario 
%   1 = Case 1: Severe fault in Follower 2 left motor at t = 12s (gamma_L = 0.92 >= 0.65)
%               -> Follower 2 leaves formation, Follower 1 reconfigures to 270 deg 
%   2 = Case 2: Non-severe fault in Follower 2 left motor at t = 13s (gamma_L = 0.38 < 0.65)
%               -> Team re-coordinates motion at 62% speed 
scenarioCase = 2;

if scenarioCase == 1
faultTime     = 12.0; % Fault injection time (s) 
duration      = 32.0; % Total time (s) 
injectedGamma = 0.92; % 92% loss of effectiveness in Left motor
else
faultTime     = 13.0; % Fault injection time (s) 
duration      = 33.0; % Total time (s) 
injectedGamma = 0.38; % 38% loss of effectiveness in Left motor
end

simParams = struct(...
'scenarioCase', scenarioCase, ...
'timeStep',     0.1, ...        % Sampling time Ts (s)
'duration',     duration, ...   % Total simulation duration (s)
'faultTime',    faultTime, ...  % Actuator fault injection time (s)
'severeThresh', 0.65, ...       % Severe fault threshold (Assumption 4)
'numFollowers', 2);             % Initial number of followers

robot = struct(...
'Ts',          simParams.timeStep, ...
'wheelRadius', 0.05, ...        % Wheel radius r (m)
'axleLength',  0.15, ...        % Wheelbase 2*h (m) 
'maxLinVel',   0.70, ...        % Max linear velocity (m/s) 
'maxAngVel',   1.50, ...        % Max angular velocity (rad/s)
'safetyDist',  0.45, ...        % Collision avoidance safety radius (m)
'Kc',          0.20, ...        % Motor torque/back-EMF constant
'Im',          0.01, ...        % Rotor inertia (kg.m^2)
'b',           0.10, ...        % Viscous friction (N.m.s/rad)
'La',          0.05, ...        % Armature inductance (H)
'Ra',          2.00);           % Armature resistance (Ohm)

%% 3. Initialize Robot Poses 
leaderPose    = [-0.19; -1.92; deg2rad(100)];          % q_L(0)
followerPoses = [-0.69, -2.45,  deg2rad(100);          % q_1(0) - Follower 1
-0.69, -1.474, deg2rad(100)]';        % q_2(0) - Follower 2

%% 4. Initialize Distributed Control & FDD Components (One per Robot!)
[mpcController, ~, leaderMpcState] = createControlComponents(robot);

followerMpcStates = cell(1, simParams.numFollowers);
faultDetectors    = cell(1, simParams.numFollowers);
motorStates       = zeros(4, simParams.numFollowers); % True motor states [omega_R; i_R; omega_L; i_L]

for fIdx = 1:simParams.numFollowers
[~, faultDetectors{fIdx}, followerMpcStates{fIdx}] = createControlComponents(robot);
end

formationActive = true(1, simParams.numFollowers);
assignedSlots   = 1:simParams.numFollowers; % Initial slot assignment [1, 2]
recoordGamma    = 0;                        % Leader motion re-coordination factor
effectiveTime   = 0;                        % Accumulated trajectory time for smooth speed scaling

%% 5. Preallocate Simulation Logging Data
timeVector = 0:simParams.timeStep:simParams.duration;
simData    = initializeSimulationData(timeVector, simParams.numFollowers);

%% 6. Main Simulation Loop
for idx = 1:length(timeVector)
currentTime = timeVector(idx);
dt = simParams.timeStep;

% --- A. Leader Reference Trajectory & Motion Re-Coordination 
if idx > 1
    effectiveTime = effectiveTime + (1 - recoordGamma) * dt;
end
[refPos, refHeading] = GenerateReferenceTrajectory(effectiveTime, 0, -pi/2);

% Scale leader velocity limits during motion re-coordination (Case 2)
leaderMaxV = 0.35 * (1 - recoordGamma);
if currentTime < 4.0
    leaderMaxV = 0.50; % Initial catch-up phase
end

% --- B. Leader MPC Control ---
[leaderLinVel, leaderAngVel, leaderMpcState] = ComputeMPCCommand(...
    mpcController, leaderMpcState, leaderPose, [refPos; refHeading]);
leaderLinVel = max(min(leaderLinVel, leaderMaxV), -leaderMaxV);

leaderPose = UpdateRobotPose(leaderPose, leaderLinVel, leaderAngVel, dt);
simData.leaderLinVel(idx) = abs(leaderLinVel);

% Count active healthy followers
numActiveFollowers = sum(formationActive);

% --- C. Follower Control, Motor Actuation, and FDD Loop ---
for fIdx = 1:simParams.numFollowers
    % True actuator loss of effectiveness vector [gamma_R; gamma_L]
    gamma_true = [0; 0];
    if (fIdx == 2) && (currentTime >= simParams.faultTime)
        gamma_true = [0; injectedGamma]; % Fault in Left motor of Follower 2
    end
    if fIdx == 2
        simData.trueGammaF2(:, idx) = gamma_true;
    end

    % If Follower has left the formation due to a severe fault (Case 1)
    if ~formationActive(fIdx)
        simData.followerLinVel(fIdx, idx) = 0;
        if fIdx == 2
            simData.f2LeftWheelSpeed(idx) = 0; % Stopped (after ~14s)
        end
        if idx > 1
            simData.estGamma(:, fIdx, idx)    = simData.estGamma(:, fIdx, idx-1);
            simData.faultEstimates(fIdx, idx) = simData.faultEstimates(fIdx, idx-1);
        end
        continue;
    end

    % 1. Desired Formation Slot Position 
    slotID = assignedSlots(fIdx);
    [desiredPos, desiredHeading, phi_d_deg] = CalculateFormationPosition(...
        leaderPose, slotID, numActiveFollowers, true);

    if fIdx == 1
        if idx == 1
            simData.f1DesiredAngleDeg(idx) = 235;
        else
            alpha = 0.25;
            simData.f1DesiredAngleDeg(idx) = (1 - alpha)*simData.f1DesiredAngleDeg(idx-1) + alpha*phi_d_deg;
        end
    end

    % 2. Follower MPC Control 
    [linVelCmd, angVelCmd, followerMpcStates{fIdx}] = ComputeMPCCommand(...
        mpcController, followerMpcStates{fIdx}, followerPoses(:, fIdx), [desiredPos; desiredHeading]);

    % Apply motion re-coordination velocity scaling (Case 2)
    if recoordGamma > 0
        fMaxV = ((fIdx == 1)*0.68 + (fIdx == 2)*0.36) * (1 - recoordGamma);
        linVelCmd = max(min(linVelCmd, fMaxV), -fMaxV);
    end

    % 3. Collision Avoidance with Leader and Other Followers 
    otherIdx  = setdiff(1:simParams.numFollowers, fIdx);
    neighbors = [leaderPose(1:2), followerPoses(1:2, otherIdx)];
    [linVelCmd, angVelCmd] = AvoidCollisions(...
        followerPoses(:, fIdx), neighbors, linVelCmd, angVelCmd, robot);

    % 4-6. Simulate Equation (4), Voltage Conversion, and DC Motor Actuators 
    [omega_meas, linVelAct, angVelAct, u_voltage, motorStates(:, fIdx)] = SimulateMotorActuators(...
        linVelCmd, angVelCmd, motorStates(:, fIdx), gamma_true, robot, faultDetectors{fIdx});

    if fIdx == 2
        simData.f2LeftWheelSpeed(idx) = max(0, omega_meas(2));
    end

    % 7. Two-Stage Kalman Filter FDD Estimation 
    [faultDetectors{fIdx}, gamma_est, ~] = stepTSKF(faultDetectors{fIdx}, u_voltage, omega_meas);
    simData.estGamma(:, fIdx, idx)    = gamma_est;
    simData.faultEstimates(fIdx, idx) = max(gamma_est);

    % 8. Update Follower Pose 
    followerPoses(:, fIdx) = UpdateRobotPose(followerPoses(:, fIdx), linVelAct, angVelAct, dt);
    simData.followerLinVel(fIdx, idx) = abs(linVelAct);
end

% --- D. Task Reassignment & Decision Making 
if currentTime >= (simParams.faultTime + 1.5)
    [formationActive, assignedSlots, recoordGamma] = ReconfigureFormation(...
        followerPoses, formationActive, simData.faultEstimates(:, idx), ...
        leaderPose, assignedSlots, simParams.severeThresh);
end

% --- E. Log Trajectories ---
simData.leaderTrajectory(:, idx)        = leaderPose;
simData.followerTrajectories(:, :, idx) = followerPoses;


end

%% 7. Visualization (Generates Case 1 or Case 2)
visualizeResults(simData, simParams);

%% ========================================================================
%  MODULAR FUNCTIONS (All 12 Project Functions Included Below)
%  ========================================================================

function [mpcController, faultDetector, mpcState] = createControlComponents(params)
% CREATECONTROLCOMPONENTS - Initializes the MPC controller, MPC state, and
% Two-Stage Kalman Filter (FDD unit) for a WMR.
if nargin < 1
params = struct();
end
[mpcController, mpcState] = FTCC_MPC_Design(params);
faultDetector = initTSKF(params);
end

function [mpcController, mpcState] = FTCC_MPC_Design(robot)
% FTCC_MPC_DESIGN - Configures the Linear MPC controller for WMR trajectory
% and formation tracking via Input-Output Feedback Linearization.
if nargin < 1 || isempty(robot), robot = struct(); end
if isfield(robot, 'Ts'),        Ts = robot.Ts;
elseif isfield(robot, 'dt'),    Ts = robot.dt;
else,                           Ts = 0.1;
end
if isfield(robot, 'maxLinVel'), maxV = robot.maxLinVel;
else,                           maxV = 0.7;
end
if isfield(robot, 'maxAngVel'), maxW = robot.maxAngVel;
else,                           maxW = 2.0;
end

A = eye(3);
B = Ts * eye(3);
C = eye(3);
D = zeros(3, 3);
plant = ss(A, B, C, D, Ts);

P = 15;
M = 5;
mpcController = mpc(plant, Ts, P, M);

mpcController.MV(1).Min = -maxV; mpcController.MV(1).Max = maxV;
mpcController.MV(2).Min = -maxV; mpcController.MV(2).Max = maxV;
mpcController.MV(3).Min = -maxW; mpcController.MV(3).Max = maxW;

mpcController.MV(1).RateMin = -0.5; mpcController.MV(1).RateMax = 0.5;
mpcController.MV(2).RateMin = -0.5; mpcController.MV(2).RateMax = 0.5;
mpcController.MV(3).RateMin = -1.5; mpcController.MV(3).RateMax = 1.5;

mpcController.Weights.OutputVariables          = [15, 15, 2];
mpcController.Weights.ManipulatedVariables     = [0.1, 0.1, 0.1];
mpcController.Weights.ManipulatedVariablesRate = [0.5, 0.5, 0.2];

mpcController.Model.Noise = ss(0.1 * eye(3));
mpcState = mpcstate(mpcController);


end

function [linVel, angVel, state] = ComputeMPCCommand(controller, state, currentPose, refPose, lookAheadDist)
% COMPUTEMPCCOMMAND - Calculates linear and angular velocity commands using
% Linear MPC combined with Input-Output Feedback Linearization.
if nargin < 5 || isempty(lookAheadDist)
lookAheadDist = 0.15;
end
currentPose = currentPose(:);
refPose     = refPose(:);
theta       = currentPose(3);

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

headingErr = atan2(sin(refPose(3) - theta), cos(refPose(3) - theta));
refPose(3) = theta + headingErr;

ny = size(controller.Model.Plant.C, 1);
y_meas = currentPose(1:ny)';
r_ref  = refPose(1:ny)';

options = mpcmoveopt;
mv = mpcmove(controller, state, y_meas, r_ref, [], options);

u_x     = mv(1);
u_y     = mv(2);
u_theta = mv(3);

linVel =  u_x * cos(theta) + u_y * sin(theta);
angVel = (-u_x * sin(theta) + u_y * cos(theta)) / max(lookAheadDist, 0.10) + 0.4 * u_theta;

maxV = controller.MV(1).Max; minV = controller.MV(1).Min;
maxW = controller.MV(3).Max; minW = controller.MV(3).Min;

if ~isempty(maxV) && isfinite(maxV), linVel = min(linVel, maxV); end
if ~isempty(minV) && isfinite(minV), linVel = max(linVel, minV); end
if ~isempty(maxW) && isfinite(maxW), angVel = min(angVel, maxW); end
if ~isempty(minW) && isfinite(minW), angVel = max(angVel, minW); end


end

function obj = initTSKF(paramsOrTs)
% INITTSKF - Initializes the Two-Stage Kalman Filter struct (Equations 10-36)
obj = struct();
obj.Kc = 0.2;
obj.Im = 0.01;
obj.b  = 0.1;
obj.La = 0.05;
obj.Ra = 2.0;
obj.Ts = 0.1;

if nargin >= 1 && ~isempty(paramsOrTs)
    if isnumeric(paramsOrTs) && isscalar(paramsOrTs)
        obj.Ts = paramsOrTs;
    elseif isstruct(paramsOrTs)
        if isfield(paramsOrTs, 'Ts'), obj.Ts = paramsOrTs.Ts;
        elseif isfield(paramsOrTs, 'dt'), obj.Ts = paramsOrTs.dt;
        end
        if isfield(paramsOrTs, 'Kc'), obj.Kc = paramsOrTs.Kc; end
        if isfield(paramsOrTs, 'Im'), obj.Im = paramsOrTs.Im; end
        if isfield(paramsOrTs, 'b'),  obj.b  = paramsOrTs.b;  end
        if isfield(paramsOrTs, 'La'), obj.La = paramsOrTs.La; end
        if isfield(paramsOrTs, 'Ra'), obj.Ra = paramsOrTs.Ra; end
    end
end

A_cont = [-obj.b/obj.Im,   obj.Kc/obj.Im,  0,              0;
          -obj.Kc/obj.La, -obj.Ra/obj.La,  0,              0;
           0,              0,             -obj.b/obj.Im,   obj.Kc/obj.Im;
           0,              0,             -obj.Kc/obj.La, -obj.Ra/obj.La];
B_cont = [0,        0;
          1/obj.La, 0;
          0,        0;
          0,        1/obj.La];
C_cont = [1, 0, 0, 0;
          0, 0, 1, 0];

sys  = ss(A_cont, B_cont, C_cont, zeros(2, 2));
sysd = c2d(sys, obj.Ts);

obj.A = sysd.A;
obj.B = sysd.B;
obj.C = sysd.C;

obj.Qx     = 1e-4 * eye(4);
obj.Qgamma = 1e-3 * eye(2);
obj.R      = 1e-2 * eye(2);

obj.x_tilde   = zeros(4, 1);
obj.Px_tilde  = eye(4);
obj.gamma_hat = zeros(2, 1);
obj.Pgamma    = 0.1 * eye(2);
obj.V         = zeros(4, 2);
obj.x_hat     = zeros(4, 1);
obj.P_hat     = eye(4);


end

function [obj, gamma, x_comp] = stepTSKF(obj, u, y)
% STEPTSKF - Performs one prediction-correction cycle of the TSKF 
u = u(:); y = y(:);
if isscalar(u), u = [u; u]; end
if isscalar(y), y = [y; y]; end

U_k = diag(u(1:2)); 

% 1. Fault Parameter Time-Update 
gamma_pred  = obj.gamma_hat;
Pgamma_pred = obj.Pgamma + obj.Qgamma;

% 2. Prediction Coupling Equations 
W_k    = obj.A * obj.V - obj.B * U_k;
V_pred = W_k * obj.Pgamma / Pgamma_pred;

% 3. Fault-Free State Time-Update 
x_tilde_pred  = obj.A * obj.x_tilde + obj.B * u(1:2) + W_k * obj.gamma_hat - V_pred * obj.gamma_hat;
Px_tilde_pred = obj.A * obj.Px_tilde * obj.A' + obj.Qx + W_k * obj.Pgamma * W_k' - V_pred * Pgamma_pred * V_pred';

% 4. Filter Residual and Covariance 
S_tilde = obj.C * Px_tilde_pred * obj.C' + obj.R;
r_tilde = y(1:2) - obj.C * x_tilde_pred;

% 5. Fault-Free State Measurement-Update 
Kx_tilde     = (Px_tilde_pred * obj.C') / S_tilde;
obj.x_tilde  = x_tilde_pred + Kx_tilde * r_tilde;
obj.Px_tilde = (eye(4) - Kx_tilde * obj.C) * Px_tilde_pred;

% 6. Update Coupling Equations 
H_pred = obj.C * V_pred;
obj.V  = V_pred - Kx_tilde * H_pred;

% 7. Fault Parameter Measurement-Update 
S_gamma       = H_pred * Pgamma_pred * H_pred' + S_tilde;
K_gamma       = (Pgamma_pred * H_pred') / S_gamma;
obj.gamma_hat = gamma_pred + K_gamma * (r_tilde - H_pred * obj.gamma_hat);
obj.Pgamma    = (eye(2) - K_gamma * H_pred) * Pgamma_pred;

% 8. Compensated State and Covariance Estimator 
obj.x_hat = obj.x_tilde + obj.V * obj.gamma_hat;
obj.P_hat = obj.Px_tilde + obj.V * obj.Pgamma * obj.V';

gamma  = max(min(obj.gamma_hat, 1), 0);
x_comp = obj.x_hat;


end

function [omega_meas, linVelAct, angVelAct, u_voltage, motorState] = SimulateMotorActuators(linVelCmd, angVelCmd, motorState, gamma_true, robot, fddUnit)
% SIMULATEMOTORACTUATORS - Implements Equation (4), Conversion Function,
% and Actuators blocks.
h_half      = robot.axleLength / 2;
v_R_des     = linVelCmd + h_half * angVelCmd;
v_L_des     = linVelCmd - h_half * angVelCmd;
omega_R_des = v_R_des / robot.wheelRadius;
omega_L_des = v_L_des / robot.wheelRadius;

k_volt    = (robot.Ra * robot.b / robot.Kc) + robot.Kc;
u_voltage = k_volt * [omega_R_des; omega_L_des];

u_effective = u_voltage .* (1 - gamma_true(:));
motorState  = fddUnit.A * motorState + fddUnit.B * u_effective;

omega_meas = fddUnit.C * motorState + 0.02 * randn(2, 1);

v_R_act   = omega_meas(1) * robot.wheelRadius;
v_L_act   = omega_meas(2) * robot.wheelRadius;
linVelAct = 0.5 * (v_R_act + v_L_act);
angVelAct = (v_R_act - v_L_act) / (2 * h_half);


end

function [position, heading, velocity] = GenerateReferenceTrajectory(t, gamma, phaseOffset)
% GENERATEREFERENCETRAJECTORY - Equations (41), (43), and (44).
if nargin < 2 || isempty(gamma),       gamma = 0;       end
if nargin < 3 || isempty(phaseOffset), phaseOffset = 0; end

nominalRate   = 0.25;
effectiveRate = nominalRate * (1 - gamma);
angle         = effectiveRate * t + phaseOffset;

x_r = cos(angle) - 0.1;
y_r = sin(angle) - 0.6;
position = [x_r; y_r];

dx_r = -effectiveRate * sin(angle);
dy_r =  effectiveRate * cos(angle);
velocity = [dx_r; dy_r];
heading  = atan2(dy_r, dx_r);


end

function [position, heading, phi_d_deg, dist_d] = CalculateFormationPosition(leaderPose, followerID, totalFollowers, rotateWithLeader)
% CALCULATEFORMATIONPOSITION 
if nargin < 4 || isempty(rotateWithLeader), rotateWithLeader = true; end
L = 0.75;

switch totalFollowers
    case 1
        angles_deg = 270;
        dists      = L;
    case 2
        angles_deg = [225, 135];
        dists      = [L,   L];
    case 3
        angles_deg = [225, 135, 180];
        dists      = [L,   L,   2 * L * cosd(45)];
    case 4
        angles_deg = [225, 135, 225, 135];
        dists      = [L,   L,   2*L, 2*L];
    case 5
        angles_deg = [225, 135, 225, 135, 180];
        dists      = [L,   L,   2*L, 2*L, 2 * L * cosd(45)];
    otherwise
        angles_deg = 225 - (180 / max(totalFollowers, 1)) * (0:totalFollowers-1);
        dists      = L * ones(1, totalFollowers);
end

idx       = max(1, min(followerID, length(angles_deg)));
phi_d_deg = angles_deg(idx);
dist_d    = dists(idx);
phi_d_rad = deg2rad(phi_d_deg);

if rotateWithLeader
    globalAngle = leaderPose(3) + phi_d_rad;
else
    globalAngle = phi_d_rad;
end

position = leaderPose(1:2) + dist_d * [cos(globalAngle); sin(globalAngle)];
heading  = leaderPose(3);


end

function [linVel, angVel] = AvoidCollisions(currentPose, neighbors, linVel, angVel, robot)
% AVOIDCOLLISIONS - Mechanical impedance collision avoidance.
repulse = [0; 0];
theta   = currentPose(3);
k_rep   = 0.3;

for n = 1:size(neighbors, 2)
    dx   = currentPose(1) - neighbors(1, n);
    dy   = currentPose(2) - neighbors(2, n);
    dist = norm([dx; dy]);

    if dist < robot.safetyDist && dist > 1e-6
        dir_away  = [dx; dy] / dist;
        force_mag = k_rep * (1/dist - 1/robot.safetyDist);
        repulse   = repulse + force_mag * dir_away;
    end
end

J = [cos(theta), -robot.axleLength * sin(theta);
     sin(theta),  robot.axleLength * cos(theta)];
adjustment = J \ repulse;

linVel = max(min(linVel + adjustment(1), robot.maxLinVel), -robot.maxLinVel);
angVel = max(min(angVel + adjustment(2), robot.maxAngVel), -robot.maxAngVel);


end

function [active, assignedSlots, recoordGamma] = ReconfigureFormation(positions, active, estimates, leaderPose, assignedSlots, severeThresh)
% RECONFIGUREFORMATION - Hungarian task reassignment.
numFollowers = size(positions, 2);
if nargin < 5 || isempty(assignedSlots), assignedSlots = 1:numFollowers; end
if nargin < 6 || isempty(severeThresh),  severeThresh = 0.65;            end

recoordGamma       = 0;
reassignmentNeeded = false;

for i = 1:numFollowers
    if ~active(i), continue; end
    gamma_i = estimates(i);

    if gamma_i >= severeThresh
        active(i)          = false;
        assignedSlots(i)   = 0;
        reassignmentNeeded = true;
    elseif gamma_i > 0.10
        recoordGamma = max(recoordGamma, gamma_i);
    end
end

activeBots = find(active);
nR         = length(activeBots);

if nR > 0 && (reassignmentNeeded || any(assignedSlots(activeBots) > nR) || any(assignedSlots(activeBots) == 0))
    costMatrix = zeros(nR, nR);
    for k = 1:nR
        botIdx = activeBots(k);
        for m = 1:nR
            [targetPos, ~]   = CalculateFormationPosition(leaderPose, m, nR, true);
            costMatrix(k, m) = norm(positions(1:2, botIdx) - targetPos);
        end
    end

    if exist('matchpairs', 'file')
        [assign, ~] = matchpairs(costMatrix, 1e6);
        for a = 1:size(assign, 1)
            assignedSlots(activeBots(assign(a, 1))) = assign(a, 2);
        end
    else
        allPerms = perms(1:nR);
        bestCost = inf;
        bestPerm = 1:nR;
        for p = 1:nR
            c = 0;
            for k = 1:nR
                c = c + costMatrix(k, allPerms(p, k));
            end
            if c < bestCost
                bestCost = c;
                bestPerm = allPerms(p, :);
            end
        end
        for k = 1:nR
            assignedSlots(activeBots(k)) = bestPerm(k);
        end
    end
end


end

function newPose = UpdateRobotPose(currentPose, linVel, angVel, dt)
% UPDATEROBOTPOSE - WMR nonholonomic kinematics integration 
currentPose = currentPose(:);
theta       = currentPose(3);

if abs(angVel) > 1e-6
    newTheta = theta + angVel * dt;
    newX     = currentPose(1) + (linVel / angVel) * (sin(newTheta) - sin(theta));
    newY     = currentPose(2) - (linVel / angVel) * (cos(newTheta) - cos(theta));
else
    newX     = currentPose(1) + linVel * cos(theta) * dt;
    newY     = currentPose(2) + linVel * sin(theta) * dt;
    newTheta = theta + angVel * dt;
end

newTheta = atan2(sin(newTheta), cos(newTheta));
newPose  = [newX; newY; newTheta];


end

function simData = initializeSimulationData(timeVector, numFollowers)
% INITIALIZESIMULATIONDATA - Preallocates simulation logging structure.
numSteps = length(timeVector);
simData  = struct(...
'time',                 timeVector, ...
'timeVector',           timeVector, ...
'leaderTrajectory',     zeros(3, numSteps), ...
'followerTrajectories', zeros(3, numFollowers, numSteps), ...
'leaderLinVel',         zeros(1, numSteps), ...
'followerLinVel',       zeros(numFollowers, numSteps), ...
'trueGammaF2',          zeros(2, numSteps), ...
'estGamma',             zeros(2, numFollowers, numSteps), ...
'faultEstimates',       zeros(numFollowers, numSteps), ...
'f2LeftWheelSpeed',     zeros(1, numSteps), ...
'f1DesiredAngleDeg',    zeros(1, numSteps) ...
);
end

function visualizeResults(simData, simParams)
% VISUALIZERESULTS - Generates (Case 1) or (Case 2).
t  = simData.timeVector;
sc = simParams.scenarioCase;

PlotFormationTrajectories(simData, simParams.faultTime);
PlotFaultEstimates(simData, simParams.faultTime);

figure('Name', sprintf('Case %d: Left Motor Output Speed', sc), ...
       'Color', 'w', 'Position', [220, 140, 650, 380]);
plot(t, simData.f2LeftWheelSpeed, 'b-', 'LineWidth', 1.5); hold on;
if exist('xline', 'file')
    xline(simParams.faultTime, '--r', 'Fault Occurrence', 'LineWidth', 1.2);
end
xlabel('t (sec)'); ylabel('\omega_L (rad/sec)');
title(sprintf('Case %d: Output of the Left Motor of Follower 2', sc));
grid on;

if sc == 1
    figure('Name', 'Case 1: Desired Formation Angle of Follower 1', ...
           'Color', 'w', 'Position', [260, 180, 650, 380]);
    plot(t, simData.f1DesiredAngleDeg, 'b-', 'LineWidth', 1.8);
    xlabel('t (sec)'); ylabel('\phi_d (degs)'); ylim([200, 300]);
    title('Desired Formation Angle of Follower 1 (\phi_d)'); grid on;
else
    figure('Name', 'Case 2: Robots Linear Velocities', ...
           'Color', 'w', 'Position', [260, 180, 650, 380]);
    plot(t, simData.leaderLinVel, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, simData.followerLinVel(1, :), 'r--', 'LineWidth', 1.5);
    plot(t, simData.followerLinVel(2, :), 'g:', 'LineWidth', 1.5);
    xlabel('t (sec)'); ylabel('v (m/s)'); ylim([0.1, 0.8]);
    legend('Leader', 'Follower 1', 'Follower 2', 'Location', 'northeast');
    title('Robots Linear Velocities During Mission Execution (Case 2)'); grid on;
end


end

function PlotFormationTrajectories(simData, faultTime)
% PLOTFORMATIONTRAJECTORIES - Plots 2D (X-Y) trajectories.
figure('Name', 'Robots Trajectories During Mission Execution', ...
'Color', 'w', 'Position', [200, 200, 680, 520]);
hold on;
t = simData.timeVector;
numFollowers   = size(simData.followerTrajectories, 2);
followerStyles = {'r--', 'g:', 'm-.', 'c--'};
followerColors = {'r',   'g',  'm',   'c'};

plot(simData.leaderTrajectory(1, :), simData.leaderTrajectory(2, :), ...
     'b-', 'LineWidth', 1.8, 'DisplayName', 'Leader');

for f = 1:numFollowers
    style = followerStyles{mod(f - 1, length(followerStyles)) + 1};
    fx    = squeeze(simData.followerTrajectories(1, f, :));
    fy    = squeeze(simData.followerTrajectories(2, f, :));
    plot(fx, fy, style, 'LineWidth', 1.8, 'DisplayName', sprintf('Follower %d', f));
end

plot(simData.leaderTrajectory(1, 1), simData.leaderTrajectory(2, 1), ...
     'bo', 'MarkerSize', 7, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(simData.leaderTrajectory(1, end), simData.leaderTrajectory(2, end), ...
     'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'HandleVisibility', 'off');

for f = 1:numFollowers
    col = followerColors{mod(f - 1, length(followerColors)) + 1};
    fx  = squeeze(simData.followerTrajectories(1, f, :));
    fy  = squeeze(simData.followerTrajectories(2, f, :));
    plot(fx(1), fy(1), [col, 'o'], 'MarkerSize', 7, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(fx(end), fy(end), [col, 'o'], 'MarkerFaceColor', col, 'MarkerSize', 8, 'HandleVisibility', 'off');
end

faultIdx = find(t >= faultTime, 1, 'first');
if ~isempty(faultIdx) && numFollowers >= 2
    faultX = simData.followerTrajectories(1, 2, faultIdx);
    faultY = simData.followerTrajectories(2, 2, faultIdx);
    plot(faultX, faultY, 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'HandleVisibility', 'off');
    text(faultX - 0.8, faultY - 0.35, 'Fault Occurrence', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 9);
end

title('Robots Trajectories During Mission Execution');
xlabel('x (m)'); ylabel('y (m)');
axis equal; xlim([-2.5, 2.5]); ylim([-3.0, 2.0]);
legend('Location', 'northeast'); grid on;


end

function PlotFaultEstimates(simData, faultTime)
% PLOTFAULTESTIMATES - Plots Two-Stage Kalman Filter FDD.
t = simData.timeVector;

figure('Name', 'Follower 2 Motor Fault Estimates.', ...
       'Color', 'w', 'Position', [120, 120, 650, 450]);
subplot(2, 1, 1); hold on;
plot(t, simData.trueGammaF2(1, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Real');
plot(t, squeeze(simData.estGamma(1, 2, :)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
ylabel('\gamma_R^2'); ylim([-0.2, 1.0]); grid on;
legend('Location', 'northeast');
title('True and Estimated Loss of Control Effectiveness (\gamma^2)');

subplot(2, 1, 2); hold on;
plot(t, simData.trueGammaF2(2, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Real');
plot(t, squeeze(simData.estGamma(2, 2, :)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
if exist('xline', 'file')
    xline(faultTime, '--k', 'Fault Injection', 'HandleVisibility', 'off');
end
xlabel('t (sec)'); ylabel('\gamma_L^2'); ylim([-0.2, 1.0]); grid on;
legend('Location', 'northeast');


end