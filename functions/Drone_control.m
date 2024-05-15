function u = Drone_control(environment,init_state,destination,Dt,maxvel,offset)
%Drone_control compute the velocity control
%   Drone_control take as input the destinationo and the initial state and
%   cumpute the velocities in x,y,z,theta that the drone has to do in order to reach the target position. 
% INPUT:
%   environment -> the heigth matrix
%   init_state  -> it is a vector with the initial position (x,y,z,theta)
%   destination -> it is a vector of 4 colomns with the x y z theta
%   Dt          -> time step
%   maxvel      -> vecor with the max velocities in x and y

if numel(init_state) ~= 4
   error('Error in init_state. It has to contain 4 values: x,y,z,theta');
end

if size(destination) ~= 4
   error('Error in destination size. It has to have 4 values: x,y,z,theta');
end

if size(maxvel) < 3
   error('Error in destination size. It has to have at least 3 values: vx,vy,vz');
end

if size(maxvel) > 3
   warning('Error in destination size. It has to have at 3 values: vx,vy,vz. Now taking the first 3 values: vx=%f, vy=%f, vz=%f',maxvel(1),maxvel(2),maxvel(3));
end


% drone max velocities
if ~exist('maxvel','var')
    vmaxx = 5;
    vmaxy = 5;
    vmaxz = 5;
end
vmaxx = maxvel(1);
vmaxy = maxvel(2);
vmaxz = maxvel(3);


dx = destination(1) - init_state(1);
dy = destination(2) - init_state(2);
dth= atan2(dy,dx) - init_state(4);

vx = dx/Dt;
vy = dy/Dt;
omega = dth/Dt;

if abs(vx) > vmaxx
    warning('Velocity in x (%.2f) has overcome the max!',vx);
    vx = vmaxx*sign(vx);
end
if abs(vy) > vmaxy
    warning('Velocity in y (%.2f) has overcome the max!',vy);
    vy = vmaxy*sign(vy);
end

dz = interp2(environment,init_state(1)+vx*Dt,init_state(2)+vy*Dt, 'linear') - init_state(3) + offset;
%dz = interp2(environment,destination(1),destination(2), 'linear') - init_state(3) + offset;

vz = dz/Dt;
if abs(vz) > vmaxz
    warning('Velocity in z (%.2f) has overcome the max!',vz);
    vz = vmaxz*sign(vz);
end

u = [vx vy vz omega];

end