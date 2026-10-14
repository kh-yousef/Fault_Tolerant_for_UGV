function PlotFaultEstimates(simData, faultTime)
% PLOTFAULTESTIMATES - Plots the actuator loss of effectiveness estimates
% from the Two-Stage Kalman Filter (FDD unit).
% fault comparison across all followers.
%
% Inputs:
%   simData   - Simulation data struct containing timeVector/time,
%               faultEstimates, and (optionally) estGamma and trueGammaF2
%   faultTime - Time of fault injection (s)

% 1. Extract time vector safely (supports both .timeVector and .time)
if isfield(simData, 'timeVector')
    t = simData.timeVector;
elseif isfield(simData, 'time')
    t = simData.time;
else
    t = 1:size(simData.faultEstimates, 2);
end

% 2. If per-motor estimates [gamma_R; gamma_L] exist
if isfield(simData, 'estGamma') && size(simData.estGamma, 2) >= 2
    figure('Name', 'Follower 2 Motor Fault Estimates', ...
        'Color', 'w', 'Position', [120, 120, 650, 450]);

    % Top Subplot: Right Motor Loss of Effectiveness (\gamma_R^2)
    subplot(2, 1, 1);
    hold on;
    if isfield(simData, 'trueGammaF2')
        plot(t, simData.trueGammaF2(1, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Real');
    end
    plot(t, squeeze(simData.estGamma(1, 2, :)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    ylabel('\gamma_R^2');
    ylim([-0.2, 1.0]);
    grid on;
    legend('Location', 'northeast');
    title('True and Estimated Loss of Control Effectiveness (\gamma^2)');

    % Bottom Subplot: Left Motor Loss of Effectiveness (\gamma_L^2)
    subplot(2, 1, 2);
    hold on;
    if isfield(simData, 'trueGammaF2')
        plot(t, simData.trueGammaF2(2, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Real');
    end
    plot(t, squeeze(simData.estGamma(2, 2, :)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    if nargin >= 2 && ~isempty(faultTime)
        if exist('xline', 'file')
            xline(faultTime, '--k', 'Fault Injection', 'HandleVisibility', 'off');
        else
            plot([faultTime, faultTime], [-0.2, 1.0], '--k', 'HandleVisibility', 'off');
        end
    end
    xlabel('t (sec)');
    ylabel('\gamma_L^2');
    ylim([-0.2, 1.0]);
    grid on;
    legend('Location', 'northeast');
end

% 3. Team-Wide Fault Estimate Plot Across All Followers
figure('Name', 'Team Actuator Loss of Effectiveness Estimates', ...
    'Color', 'w', 'Position', [180, 180, 650, 380]);
hold on;
numFollowers = size(simData.faultEstimates, 1);
lineStyles = {'b-', 'r--', 'g-.', 'm:', 'c-'};

for f = 1:numFollowers
    style = lineStyles{mod(f - 1, length(lineStyles)) + 1};
    plot(t, simData.faultEstimates(f, :), style, 'LineWidth', 1.8);
end

if nargin >= 2 && ~isempty(faultTime)
    if exist('xline', 'file')
        xline(faultTime, '--k', 'Fault Injection', 'LineWidth', 1.2);
    else
        plot([faultTime, faultTime], [-0.1, 1.1], '--k', 'LineWidth', 1.2);
    end
end

% Plot severe fault threshold (gamma = 0.65 from Assumption 4)
if exist('yline', 'file')
    yline(0.65, ':r', 'Severe Fault Threshold (\gamma = 0.65)', 'LineWidth', 1.2);
end

title('Actuator Loss of Effectiveness Estimates (\gamma)');
xlabel('t (sec)');
ylabel('Loss of Effectiveness \gamma');
legend(arrayfun(@(n) sprintf('Follower %d', n), 1:numFollowers, 'UniformOutput', false), ...
    'Location', 'northwest');
grid on;
ylim([-0.1, 1.1]);


end