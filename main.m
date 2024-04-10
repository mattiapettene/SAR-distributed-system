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

H = flipud(H);

figure('Name','3D map')
hold on;
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
saveas(gcf, 'Plot/3D_map.eps', 'epsc')

%% Obstacles addition (red points)

nObstacles = 100;
mapWidth = s;
mapLength = s;
z_obstacles = zeros(s, s);

n = 0;
while n < nObstacles
    x_idx = randi([1 mapWidth],1);
    y_idx = randi([1 mapLength],1);
    z_obstacles(x_idx, y_idx) = H(y_idx, x_idx);
    n = n+1;
end

[x, y] = find(z_obstacles);
z = z_obstacles(sub2ind(size(z_obstacles), x, y));

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
hold off;
grid on;


%% Test function tree
[xT, yT, zT, xC, yC, zC] = tree(0,0,0);
figure;
surf(xT,yT,zT, 'FaceColor', '#53350A', "EdgeColor","none");
hold on;
axis equal;
surf(xC,yC,zC, 'FaceColor', '#2A7E19', "EdgeColor","none");
view(3)

%% Obstacles addition (trees)

nObstacles = 50;
mapWidth = s;
mapLength = s;
xyzObstacles = cell(nObstacles, 1);

n = 0;
while n < nObstacles
    x_idx = randi([1 mapWidth],1);
    y_idx = randi([1 mapLength],1);
    z_val = H(y_idx, x_idx);
    [xT, yT, zT, xC, yC, zC] = tree(x_idx, y_idx, z_val);
    xyzObstacles{n+1,1} = [xT(:), yT(:), zT(:)];
    xyzObstacles{n+1,2} = [xC(:), yC(:), zC(:)];
    n = n+1;
end

% Plot di tutti gli alberi generati
figure('Name','3D map with trees');
hold on;

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);

for i = 1:nObstacles
    % Estraiamo le coordinate x, y e z per il tronco e la chioma dell'i-esimo albero
   
    % Coordinate per il tronco e la chioma
    xTrunk = xyzObstacles{i, 1}(:, 1);
    yTrunk = xyzObstacles{i, 1}(:, 2);
    zTrunk = xyzObstacles{i, 1}(:, 3);
    
    xCrown = xyzObstacles{i, 2}(:, 1);
    yCrown = xyzObstacles{i, 2}(:, 2);
    zCrown = xyzObstacles{i, 2}(:, 3);
    
    % Plot del tronco
    plot3(xTrunk, yTrunk, zTrunk, 'Color', '#53350A', 'LineWidth', 1);
    
    % Plot della chioma
    plot3(xCrown, yCrown, zCrown, 'Color', '#2A7E19', 'LineWidth', 1);
end

grid on;
axis equal;
view(3);
hold off;
saveas(gcf, 'Plot/3D_map_trees.eps', 'epsc')
