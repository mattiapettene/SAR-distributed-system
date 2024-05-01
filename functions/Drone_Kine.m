function pose = Drone_Kine(environ,initstate,u,Dt,offset)
%Drone_Kine is used to compute the kinematics of a drone given the map, the initiale state (x,y,z,th), the
%controls (vx,vy,vz,omega) and the time step
%    The input are:
%    environ  -> it is the full map in matrix form
%    initstate-> it is the initial position and orientation
%    u        -> it is the vector with the control velocities
%    Dt       -> is the time step


if numel(u) ~= 4
   warning('u has to contain the velocities for x-y-x-theta directions');
end


% local variable
x = initstate(1);
y = initstate(2);
z = initstate(3);
theta = initstate(4);

% check offset
oldheigth = environ(initstate(2),initstate(1));
if z == oldheigth
   warning('Forget to add offset in previous state. Adding now \n');
   z = oldheigth + offset;
end

    % Dynamics
    x_new = x + u(1)*Dt;
    y_new = y + u(2)*Dt;
    z_new = z + u(3)*Dt;
    theta_new = theta + u(4)*Dt;

if z_new < environ(round(y_new),round(x_new))
   warning('Drone has crashed in the terrain ');
end



pose = [x_new y_new z_new theta_new];
end