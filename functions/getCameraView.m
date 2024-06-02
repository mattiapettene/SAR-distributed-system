function [local_grid] = getCameraView(id,pose_new,obstacleFull,theta,fov)
%getCameraView Get the camera view of obstacle matrix
%   Given the position of the drone, it geto only the obstacle it can see.
%   The output is a complete matrix with only the small section with
%   obstacle position. 
% 
% INPUT:
% id            the number of the drone (check formation)
% pose_new      the drone poition
% obstacleFull  the full obstacle matrix
% theta         angle of the formation
% fov           camera field of view

if mod(fov, 2) ~= 0
    warning('N is odd, incrementing by 1 to make it even.');
    fov = fov + 1;
end

% generate the grid
[x, y] = meshgrid(1:fov, 1:fov);
x = x - fov/2;
y = y - fov/2;

% rotate the grid
x_rot = x*cos(theta) - y*sin(theta);
y_rot = x*sin(theta) + y*cos(theta);

% move the grid in position 
if id == 3 % back-left
    x_rot = x_rot + pose_new(1);
    y_rot = y_rot + pose_new(2);
elseif id == 1 % front
    x_rot = x_rot + pose_new(1);
    y_rot = y_rot + pose_new(2);
elseif id == 2 % back right
    x_rot = x_rot + pose_new(1);
    y_rot = y_rot + pose_new(2);
end

% Initialize output
outputGrid = false(size(obstacleFull));

% Round the points and check limits
x_rot = round(x_rot);
y_rot = round(y_rot);

valid_indices = x_rot >= 1 & x_rot <= 1000 & y_rot >= 1 & y_rot <= 1000;
x_rot = x_rot(valid_indices);
y_rot = y_rot(valid_indices);

% Fill the output grid
for f = 1:length(x_rot)
    outputGrid(y_rot(f), x_rot(f)) = obstacleFull(y_rot(f), x_rot(f));
end

% % Visualize the output grid with the rotated submatrix values from occupancyGrid
% figure
% spy(outputGrid)

local_grid = outputGrid;

end