function visualizeResults(simData, simParams)
% VISUALIZERESULTS - Master plotting function that generates all experimental
%
% Inputs:
%   simData   - Simulation data struct from initializeSimulationData
%   simParams - Simulation parameters struct (faultTime, scenarioCase)

if isfield(simData, 'time')
    t = simData.time;
else
    t = simData.timeVector;
end

if isfield(simParams, 'scenarioCase')
    sc = simParams.scenarioCase;
else
    sc = 1;
end

% 1. Plot 2D Formation Trajectories (Case 1 / Case 2)
PlotFormationTrajectories(simData, simParams.faultTime);

% 2. Plot FDD Loss of Effectiveness Estimates (Case 1 / Case 2)
PlotFaultEstimates(simData, simParams.faultTime);

% 3. Plot Left Motor Output Speed of Follower 2 (Case 1 / Case 2)
if isfield(simData, 'f2LeftWheelSpeed')
    figure('Name', sprintf('Case %d: Left Motor Output Speed', sc), ...
           'Color', 'w', 'Position', [220, 140, 650, 380]);
    plot(t, simData.f2LeftWheelSpeed, 'b-', 'LineWidth', 1.5);
    hold on;
    if exist('xline', 'file')
        xline(simParams.faultTime, '--r', 'Fault Occurrence', 'LineWidth', 1.2);
    end
    xlabel('t (sec)');
    ylabel('\omega_L (rad/sec)');
    title(sprintf('Case %d: Output of the Left Motor of Follower 2', sc));
    grid on;
end

% 4. Plot Scenario-Specific Figure:
if sc == 1 && isfield(simData, 'f1DesiredAngleDeg')
    figure('Name', 'Case 1: Desired Formation Angle of Follower 1 ', ...
           'Color', 'w', 'Position', [260, 180, 650, 380]);
    plot(t, simData.f1DesiredAngleDeg, 'b-', 'LineWidth', 1.8);
    xlabel('t (sec)');
    ylabel('\phi_d (degs)');
    ylim([200, 300]);
    title('Desired Formation Angle of Follower 1 (\phi_d)');
    grid on;
elseif sc == 2 && isfield(simData, 'leaderLinVel')
    figure('Name', 'Case 2: Robots Linear Velocities', ...
           'Color', 'w', 'Position', [260, 180, 650, 380]);
    plot(t, simData.leaderLinVel, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, simData.followerLinVel(1, :), 'r--', 'LineWidth', 1.5);
    if size(simData.followerLinVel, 1) >= 2
        plot(t, simData.followerLinVel(2, :), 'g:', 'LineWidth', 1.5);
    end
    xlabel('t (sec)');
    ylabel('v (m/s)');
    ylim([0.1, 0.8]);
    legend('Leader', 'Follower 1', 'Follower 2', 'Location', 'northeast');
    title('Robots Linear Velocities During Mission Execution (Case 2)');
    grid on;
end


end