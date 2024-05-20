function [pose_hist, pose_gps_hist, pose_est_hist, P] = updateDronePosition(H, path, pose_hist, vmax, offset, Dt, threshold, id, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid)
% Cumpute the trajectory given a list of target positions considering
% limitations.
% INPUTS:
%   H           -> environment
%   path        -> list of target positions (just x-y)
%   pose_hist   -> history of the drone
%   vmax        -> vector containing the max velocities in x-y-z
%   offset      -> distance from the ground level (m)
%   Dt          -> temporal interval
%   threshold   -> distance from the target inside which it is reached
%   id          -> drone number

drone_hist = pose_hist(:,:,id);
drone_hist = drone_hist(drone_hist(:,3) ~= 0, :);
pose_hist_index = size(drone_hist,1);

% Following every coverage path points
for n = 1:size(path,1)
    
    % Coverage path point to be reached
    destin = [path(n,:), H(path(n,2),path(n,1))+offset, 0];

    i = 1;
    while i % movements to get there
        pose_actual = pose_hist(pose_hist_index,:,id);
        u = Drone_control(H,pose_actual,destin,Dt,vmax,offset);

        % update the list of pose
        pose_new = Drone_Kine(H,pose_actual,u,Dt,offset,0);
        pose_hist(pose_hist_index+1,:,id) = pose_new;

        % Kalman Filter
        [pose_gps_hist, pose_est_hist, P] = KalmanFilter(id, pose_hist_index, u, pose_new, pose_actual, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid);
        
        pose_hist_index = pose_hist_index + 1;
              
        if i >= 2000
            error('Error. Taking over 1000 iteration to reach next point: [%i,%i]',path(n,1),path(n,2));
        end

        i = i+1;

        % check if it's reached
        if norm(destin - pose_new) <= threshold
            i = 0;
        end
     end
end
end