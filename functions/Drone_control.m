function u = Drone_control(init_state,traject,mode,Dt)
%Drone_control compute the velocity control
%   Drone_control take as input the trajectoy and the initial state and
%   cumpute the velocities in x,y,z,theta that the drone has to do in order to
%   reamin on the trajectory. 
% INPUT:
%   init_state  -> it is a vector with the initial pose (x,y,z,th)
%   traject     -> it is a matrix of 3 colomns with the x y z
%   mode        -> to say if it is in rapid movement (0) or in searching (1)
%   Dt          -> time step

if numel(init_state) ~= 4
   error('Error in init_state. It has to contain 4 values: x,y,z,theta');
end

if size(traject, 2) ~= 3
   error('Error in traject size. It has to have 3 colomns!');
end

if mode ~= 1 && mode ~= 0
   error('Error. Input "mode" must be 0 (rapid) or 1 (slow search).')
end

% drone max velocities
vmaxx = 5;
vmaxy = 5;
vmaxz = 5;
omegamax = 5;

if mode == 0 % rapid movement

    u = [vmaxx vmaxy vmaxz omegamax];

else % mode == 1 move slower 
    
    k = find(traject(:, 1) == init_state(1) & traject(:, 2) == init_state(2));
    target = traject(k+1,:);

    dx = target(1) - init_state(1);
    dy = target(2) - init_state(2);
    dz = target(3) - init_state(3);

    vx = dx/Dt;
    vy = dy/Dt;
    vz = dz/Dt;

    
    omega = (init_state(4)-atan2(dy,dx))/Dt;

    if vx >= vmaxx 
        warning('Velocity in x has overcome the max!');
    end
    if vy >= vmaxy 
        warning('Velocity in y has overcome the max!');
    end
    if vz >= vmaxz 
        warning('Velocity in z has overcome the max!');
    end
    if omega >= omegamax 
        warning('Velocity in omega has overcome the max!');
    end

    u = [vx vy vz omega];
   
end



end