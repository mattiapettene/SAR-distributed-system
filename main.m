%% Intelligent distributed system project
% Pettene Mattia - Poggesi Gabriele

%% Initialization

clc;
close all;
clear;

set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',  22)
set(0,'DefaultLegendFontSize', 20)

addpath('functions/');
addpath('Plot/');


%% Map creation

map = load("torbiera_viote.txt");

s = sqrt(length(map));
H = zeros(s);

for y_i = 1:s
    for x_i = 1:s
        idx = x_i + (y_i-1)*1000;
        H(x_i,y_i) = map(idx, 3);
    end
end



figure('Name','3D map')
hold on;
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
saveas(gcf, 'Plot/3D_map.eps', 'epsc')

%% Obstacles addition

nObstacles = 100;
mapWidth = s;
mapLength = s;
z_obstacles = zeros(s, s);

n = 0;
while n < nObstacles
    x_idx = randi([1 mapWidth],1);
    y_idx = randi([1 mapLength],1);
    z_obstacles(x_idx, y_idx) = H(x_idx, y_idx);
    n = n+1;
end

[x, y] = find(z_obstacles);
indices = sub2ind(size(z_obstacles), x, y);
z = z_obstacles(indices);

% Plot dei punti in 3D
figure;
hold on;
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
scatter3(x(:),y(:),z(:), 'filled', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
grid on;