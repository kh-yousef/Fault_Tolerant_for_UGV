function PlotFormationTrajectories(simData, faultTime)
% PLOTFORMATIONTRAJECTORIES - Plots the 2D (X-Y) trajectories of the Leader
% and Followers during mission execution, marking start/end positions and
% the spatial location of fault occurrence.
%
% Inputs:
%   simData   - Simulation data struct containing leaderTrajectory,
%               followerTrajectories, and timeVector/time
%   faultTime - Time of fault injection (s)

figure('Name', 'Robots Trajectories During Mission Execution', ...
       'Color', 'w', 'Position', [200, 200, 680, 520]);
hold on;

% 1. Extract time vector to locate the exact spatial index of faultTime
if isfield(simData, 'timeVector')
    t = simData.timeVector;
elseif isfield(simData, 'time')
    t = simData.time;
else
    t = [];
end

numFollowers = size(simData.followerTrajectories, 2);
followerStyles = {'r--', 'g:', 'm-.', 'c--'};
followerColors = {'r',   'g',  'm',   'c'};

% 2. Plot Leader Trajectory (Solid Blue)
plot(simData.leaderTrajectory(1, :), simData.leaderTrajectory(2, :), ...
     'b-', 'LineWidth', 1.8, 'DisplayName', 'Leader');

% 3. Plot Follower Trajectories
for f = 1:numFollowers
    style = followerStyles{mod(f - 1, length(followerStyles)) + 1};
    fx = squeeze(simData.followerTrajectories(1, f, :));
    fy = squeeze(simData.followerTrajectories(2, f, :));
    plot(fx, fy, style, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Follower %d', f));
end

% 4. Plot Start (hollow circle) and Final (filled circle) Robot Positions
% Leader start & end
plot(simData.leaderTrajectory(1, 1), simData.leaderTrajectory(2, 1), ...
     'bo', 'MarkerSize', 7, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(simData.leaderTrajectory(1, end), simData.leaderTrajectory(2, end), ...
     'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'HandleVisibility', 'off');

% Followers start & end
for f = 1:numFollowers
    col = followerColors{mod(f - 1, length(followerColors)) + 1};
    fx = squeeze(simData.followerTrajectories(1, f, :));
    fy = squeeze(simData.followerTrajectories(2, f, :));
    
    plot(fx(1), fy(1), [col, 'o'], 'MarkerSize', 7, ...
         'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(fx(end), fy(end), [col, 'o'], 'MarkerFaceColor', col, ...
         'MarkerSize', 8, 'HandleVisibility', 'off');
end

% 5. Annotate Spatial Fault Occurrence Position (instead of xline at x = 12m!)
if nargin >= 2 && ~isempty(faultTime) && ~isempty(t) && numFollowers >= 2
    faultIdx = find(t >= faultTime, 1, 'first');
    if ~isempty(faultIdx)
        faultX = simData.followerTrajectories(1, 2, faultIdx);
        faultY = simData.followerTrajectories(2, 2, faultIdx);
        plot(faultX, faultY, 'kp', 'MarkerFaceColor', 'y', ...
             'MarkerSize', 10, 'HandleVisibility', 'off');
        text(faultX - 0.8, faultY - 0.35, 'Fault Occurrence', ...
             'Color', 'r', 'FontWeight', 'bold', 'FontSize', 9);
    end
end

title('Robots Trajectories During Mission Execution');
xlabel('x (m)');
ylabel('y (m)');
axis equal;
xlim([-2.5, 2.5]);
ylim([-3.0, 2.0]);
legend('Location', 'northeast');
grid on;


end