function [position, heading, phi_d_deg, dist_d] = CalculateFormationPosition(leaderPose, followerID, totalFollowers, rotateWithLeader)
% CALCULATEFORMATIONPOSITION - Computes the desired slot position and heading
% for a given follower ID based on the number of remaining healthy followers.
%
% Inputs:
%   leaderPose       - 3x1 vector [x_L; y_L; phi_L] (x, y in meters, phi_L in rad)
%   followerID       - Assigned slot index i in {1, ..., totalFollowers}
%   totalFollowers   - Number of active/healthy followers F = N_healthy - 1
%   rotateWithLeader - (Optional) Boolean, default = true. If true, formation
%                      offsets rotate with leader heading phi_L.
%
% Outputs:
%   position   - 2x1 desired global position [x_d; y_d] 
%   heading    - Desired heading angle phi_d (rad)
%   phi_d_deg  - Desired relative formation angle (deg) 
%   dist_d     - Desired separation distance from leader 

if nargin < 4 || isempty(rotateWithLeader)
    rotateWithLeader = true;
end

% Nominal separation distance 
L = 0.75; % meters

% Predefined formation shapes
switch totalFollowers
    case 1
        % 2 robots total (Leader + 1 Follower) - Line/Side formation 
        angles_deg = 270;
        dists      = L;

    case 2
        % 3 robots total (Leader + 2 Followers) - Triangular formation 
        % Slot 1: 225 deg, Slot 2: 135 deg
        angles_deg = [225, 135];
        dists      = [L,   L];

    case 3
        % 4 robots total (Leader + 3 Followers) - Diamond formation 
        angles_deg = [225, 135, 180];
        dists      = [L,   L,   2 * L * cosd(45)];

    case 4
        % 5 robots total (Leader + 4 Followers) - V formation 
        angles_deg = [225, 135, 225, 135];
        dists      = [L,   L,   2*L, 2*L];

    case 5
        % 6 robots total (Leader + 5 Followers) - Filled Triangle/Arrow 
        angles_deg = [225, 135, 225, 135, 180];
        dists      = [L,   L,   2*L, 2*L, 2 * L * cosd(45)];

    otherwise
        % Fallback for arbitrary number of followers
        angles_deg = 225 - (180 / max(totalFollowers, 1)) * (0:totalFollowers-1);
        dists      = L * ones(1, totalFollowers);
end

% Clamp followerID to valid slot indices
idx = max(1, min(followerID, length(angles_deg)));
phi_d_deg = angles_deg(idx);
dist_d    = dists(idx);

% Convert relative formation angle to radians
phi_d_rad = deg2rad(phi_d_deg);

% Compute global desired position
if rotateWithLeader
    globalAngle = leaderPose(3) + phi_d_rad;
else
    globalAngle = phi_d_rad;
end

position = leaderPose(1:2) + dist_d * [cos(globalAngle); sin(globalAngle)];
heading  = leaderPose(3);


end