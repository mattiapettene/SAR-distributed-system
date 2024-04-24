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
figure('Name','Red points');
hold on;
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
scatter3(x(:),y(:),z(:), 'filled', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
scatter3(1,1,1, 'filled', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
hold off;
grid on;

%% Test function tree
% [xT, yT, zT, xC, yC, zC] = tree(0,0,0);
% figure;
% surf(xT,yT,zT, 'FaceColor', '#53350A', "EdgeColor","none");
% hold on;
% axis equal;
% surf(xC,yC,zC, 'FaceColor', '#2A7E19', "EdgeColor","none");
% view(3)

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

% Trees plot
figure('Name','3D map with trees');
hold on;

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);

for i = 1:nObstacles
   
    xTrunk = xyzObstacles{i, 1}(:, 1);
    yTrunk = xyzObstacles{i, 1}(:, 2);
    zTrunk = xyzObstacles{i, 1}(:, 3);
    
    xCrown = xyzObstacles{i, 2}(:, 1);
    yCrown = xyzObstacles{i, 2}(:, 2);
    zCrown = xyzObstacles{i, 2}(:, 3);
    
    plot3(xTrunk, yTrunk, zTrunk, 'Color', '#53350A', 'LineWidth', 1);
    plot3(xCrown, yCrown, zCrown, 'Color', '#2A7E19', 'LineWidth', 1);
end

grid on;
axis equal;
view(3);
hold off;
saveas(gcf, 'Plot/3D_map_trees.eps', 'epsc')

%% Map gradient matrix

[grady,gradx] = gradient(H);
gradmod = sqrt(gradx.^2 + grady.^2);

figure('Name','Terrain slope');
contour(1:1:mapWidth,1:1:mapLength,H)
hold on
quiver(1:1:mapWidth,1:1:mapLength,gradx,grady)
axis equal
hold off
saveas(gcf, 'Plot/terrain_slope.eps', 'epsc')

% Satellite image in background

RGB = imread('sat.png');
RGB = imresize(RGB,[mapLength mapWidth]);

I = im2double(RGB);

figure('Name','Satellite slope');
hold on
image(gradmod,'CDataMapping','scaled')
colorbar
axis equal
hImg = imshow(I,'InitialMagnification', 'fit'); 
set(hImg, 'AlphaData', 0.6);
hold off

%% From satellite to tree locations
        
[BW,maskedRGBImage] = WhereAreTree(RGB);

figure('Name','Real trees location')
hold on
spy(double(BW))
axis equal
hImg = imshow(I,  'InitialMagnification', 'fit'); 
set(hImg, 'AlphaData', 0.6);
spy(BW);
hold off
saveas(gcf, 'Plot/real_trees_satellite.eps', 'epsc')

%% Voronoi tessellation

% Drones starting points
sp_drone1 = [1,1];
sp_drone2 = [1,10];
sp_drone3 = [10,1];

kp = 0.5;

fprintf('Starting Voronoi coverage iteration...\n');
[c1, c2, c3, v1, v2, v3, a1, a2, a3, b1, b2, b3] = voronoi_coverage(H, sp_drone1, sp_drone2, sp_drone3, kp);
fprintf('Voronoi coverage iteration finished!\n');