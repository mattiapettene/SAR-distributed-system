function u = Drone_Verticalcontrol(environ,init_state,xyVor,Dt)
%Drone_control compute the vertical velocity
%   Drone_control take as input the environment, the elevaton and the initial state and
%   cumpute the velocities in z that the drone has to do in order to follow the velocities in x and y without crasching. 
% INPUT:
%   environ     -> elevation matrix
%   init_state  -> it is a vector with the initial pose (x,y,z,th)
%   xyVor       -> is the velocity obtained by Voronoi tassellation for x&y 
%   Dt          -> time step

if numel(init_state) < 2
   error('Error in init_state. It has to contain at least 2 values: x,y');
end

if numel(xyVor) ~= 2
   error('Error in xyVor. It has to contain 2 values: velocity in X and Y');
end

% next position absolute domain
deltax = xyVor(1)*Dt;
deltay = xyVor(2)*Dt;

newpos = round([init_state(1)+deltax init_state(2)+deltay]);

newheigth = environ(newpos(2),newpos(1));
oldheigth = environ(init_state(2),init_state(1));

vz = (newheigth-oldheigth)/Dt;

%% to be implemented OMEGA
omega = 7;

u = [xyVor(1) xyVor(2) vz omega];


end