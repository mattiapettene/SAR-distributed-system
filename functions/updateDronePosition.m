function pose_hist = updateDronePosition(H, path, sp_drone, vmax, offset, Dt, threshold)

% Cumpute the trajectory given a list of target positions considering
% limitations.
% INPUTS:
%   H           -> environment
%   path        -> list of target positions (just x-y)
%   sp_drone    -> starting point drone
%   vmax        -> vector containing the max velocities in x-y-z
%   offset      -> distance from the ground level (m)
%   Dt          -> temporal interval
%   threshold   -> distance from the target inside which it is reached

target_list = path;

% Set here the first position - origin of the trajectory
pose_hist = [sp_drone(1),sp_drone(2),H(sp_drone(1),sp_drone(2))+offset,0];

% Following every coverage path points
for n = 1:size(target_list,1)
    
    % Coverage path point to be reached
    destin = [target_list(n,:), H(target_list(n,2),target_list(n,1))+offset, 0];

    i = 1;
    while i % movements to get there
        pose_actual = pose_hist(end,:);
        u = Drone_control(H,pose_actual,destin,Dt,vmax,offset);

        % update the list of pose
        pose_new = Drone_Kine(H,pose_actual,u,Dt,offset);
        pose_hist = [pose_hist; pose_new];
              
        if i >= 2000
            error('Error. Taking over 1000 iteration to reach next point: [%i,%i]',target_list(n,1),target_list(n,2));
        end

        i = i+1;

        % check if it's reached
        if norm(destin - pose_new) <= threshold
            i = 0;
        end
    end
    
end
end