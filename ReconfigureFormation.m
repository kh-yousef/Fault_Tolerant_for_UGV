function [active, assignedSlots, recoordGamma] = ReconfigureFormation(positions, active, estimates, leaderPose, assignedSlots, severeThresh)
% RECONFIGUREFORMATION - Implements the Task Reassignment and Decision Making mechanism
%
% Inputs:
%   positions     - 3xN matrix of current follower poses [x; y; theta]
%   active        - 1xN logical array indicating active/healthy followers
%   estimates     - Nx1 vector of estimated actuator fault severity (gamma in [0, 1])
%   leaderPose    - 3x1 leader pose vector [x_L; y_L; phi_L]
%   assignedSlots - (Optional) 1xN vector of current slot indices
%   severeThresh  - (Optional) Severe fault threshold (Default = 0.65 per Assumption 4)
%
% Outputs:
%   active        - Updated 1xN logical array of active followers
%   assignedSlots - 1xN vector mapping each follower to its optimal slot ID in {1..n_R}
%   recoordGamma  - Motion re-coordination factor gamma in [0, 0.65) for Case 2

numFollowers = size(positions, 2);

if nargin < 5 || isempty(assignedSlots)
    assignedSlots = 1:numFollowers;
end
if nargin < 6 || isempty(severeThresh)
    severeThresh = 0.65; % Assumption 4
end

recoordGamma = 0;
reassignmentNeeded = false;

% 1. Evaluate Fault Severity for Each Active Follower
for i = 1:numFollowers
    if ~active(i)
        continue;
    end

    gamma_i = estimates(i);

    if gamma_i >= severeThresh
        % Case 1: Severe fault (gamma >= 0.65) -> Robot leaves formation
        active(i) = false;
        assignedSlots(i) = 0;
        reassignmentNeeded = true;
    elseif gamma_i > 0.10
        % Case 2: Non-severe fault (0 < gamma < 0.65) -> Motion re-coordination
        recoordGamma = max(recoordGamma, gamma_i);
    end
end

% 2. Optimal Task Reassignment via Hungarian Algorithm
activeBots = find(active);
nR = length(activeBots);

if nR > 0 && (reassignmentNeeded || any(assignedSlots(activeBots) > nR) || any(assignedSlots(activeBots) == 0))
    costMatrix = zeros(nR, nR);

    % Build nR x nR Euclidean distance cost matrix C(F_k, S_m)
    for k = 1:nR
        botIdx = activeBots(k);
        for m = 1:nR
            % Use CalculateFormationPosition so slot geometry matches 
            [targetPos, ~] = CalculateFormationPosition(leaderPose, m, nR, true);
            costMatrix(k, m) = norm(positions(1:2, botIdx) - targetPos);
        end
    end

    % Solve optimal linear assignment problem
    if exist('matchpairs', 'file')
        [assign, ~] = matchpairs(costMatrix, 1e6);
        for a = 1:size(assign, 1)
            botIdx  = activeBots(assign(a, 1));
            slotIdx = assign(a, 2);
            assignedSlots(botIdx) = slotIdx;
        end
    else
        % Fallback exact permutation solver if matchpairs is unavailable
        allPerms = perms(1:nR);
        bestCost = inf;
        bestPerm = 1:nR;
        for p = 1:size(allPerms, 1)
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