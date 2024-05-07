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

%% Simulation parameters

dt = 2;                 % Time step (s)
fov = 40;               % Camera FoV (m)
sp_drone1 = [1,1];      % Starting point drone 1
sp_drone2 = [1,10];     % Starting point drone 2
sp_drone3 = [10,1];     % Starting point drone 3
kp = 1/dt;              % Voronoi kp parameter
threshold = 0.8;        % Set target accuracy [m]
vmax = [10,10,10];      % Drone maximum velocities (vx, vy, vz) [m/s]
offset = 120;           % Distance from ground

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

%% Map gradient matrix

mapWidth = s;
mapLength = s;

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

%% Obstacle addition (trees)
% From satellite to tree (obstacles) locations

% Satellite image recognition
[BW,maskedRGBImage] = WhereAreTree(RGB);

nObstaclesrand = 20;
nForestTree = 80;

occupancyGrid = false(size(BW));

xyzObstacles = cell(nObstaclesrand+nForestTree, 1);

n = 0;
while n <= nObstaclesrand
    x_idx = randi([1 mapWidth],1);
    y_idx = randi([1 mapLength],1);
    z_val = H(y_idx, x_idx);
    % fill occupancy map
    ind = sub2ind(size(BW),x_idx,y_idx);
    occupancyGrid(ind) = true;

    [xT, yT, zT, xC, yC, zC] = tree(x_idx, y_idx, z_val);
    xyzObstacles{n+1,1} = [xT(:), yT(:), zT(:)];
    xyzObstacles{n+1,2} = [xC(:), yC(:), zC(:)];
    n = n+1;
end


treespos = randsample(find(BW), nForestTree);
BW = false(size(BW));
BW(treespos) = true;
occupancyGrid(treespos) = true;
[row_coords, col_coords] = ind2sub(size(BW), treespos);
treespos = [row_coords col_coords];

n = 1;
while n <= nForestTree
    x_idx = treespos(n,1);
    y_idx = treespos(n,2);
    z_val = H(y_idx, x_idx);
    [xT, yT, zT, xC, yC, zC] = tree(x_idx, y_idx, z_val);
    xyzObstacles{nObstaclesrand+n+1,1} = [xT(:), yT(:), zT(:)];
    xyzObstacles{nObstaclesrand+n+1,2} = [xC(:), yC(:), zC(:)];
    n = n+1;
end

% Set occupancy around tree locations
toleranceLevel = 2;
new_grid = occupancyGrid;

for i = 1:mapLength
    for j = 1:mapWidth
        % find the positive ones
        if occupancyGrid(i, j) == 1
            % change value around
            for k = max(1, i-2):min(mapWidth, i+2)
                for m = max(1, j-2):min(mapLength, j+2)
                    new_grid(k, m) = 1;
                end
            end
        end
    end
end

occupancyGrid = new_grid;
clear new_grid;
spy(occupancyGrid);


% Trees plot
figure('Name','3D map with trees');
hold on;

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);

for i = 1:(nObstaclesrand+nForestTree)
   
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

figure('Name','Real trees location')
hold on
spy(double(BW))
axis equal
hImg = imshow(I); 
set(hImg, 'AlphaData', 0.6);
spy(BW);
hold off
saveas(gcf, 'Plot/real_trees_satellite.eps', 'epsc')

warning('off', 'images:imshow:magnificationMustBeFitForDockedFigure')

%% Voronoi tessellation

fprintf('\nStarting Voronoi coverage iteration...\n');
[c1, c2, c3, v1, v2, v3, a1, a2, a3, b1, b2, b3] = voronoi_coverage(H, sp_drone1, sp_drone2, sp_drone3, kp);
fprintf('Voronoi coverage iteration finished!\n');

%% Testing movent Voronoi (no velocity limits)

pose1 = [sp_drone1 H(sp_drone1(2),sp_drone1(1))+offset 0];
pose2 = [sp_drone2 H(sp_drone2(2),sp_drone2(1))+offset 0];
pose3 = [sp_drone3 H(sp_drone3(2),sp_drone3(1))+offset 0];

i=1;
while i < size(v1,1)+1
    
    u1 = Drone_Verticalcontrol(H,pose1(i,:),v1(i,:),dt);
    pose1 = [pose1; Drone_Kine(H,pose1(i,:),u1,dt)];

    u2 = Drone_Verticalcontrol(H,pose2(i,:),v2(i,:),dt);
    pose2 = [pose2; Drone_Kine(H,pose2(i,:),u2,dt)];

    u3 = Drone_Verticalcontrol(H,pose3(i,:),v3(i,:),dt);
    pose3 = [pose3; Drone_Kine(H,pose3(i,:),u3,dt)];

    i = i + 1;
end

figure('Name','Voronoi fast travel')
hold on
mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
plot3(pose1(:, 1), pose1(:, 2), pose1(:, 3), '-or',LineWidth=4,MarkerSize=4);
plot3(pose2(:, 1), pose2(:, 2), pose2(:, 3), '-om',LineWidth=4,MarkerSize=4);
plot3(pose3(:, 1), pose3(:, 2), pose3(:, 3), '-oc',LineWidth=4,MarkerSize=4);
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Plot 3D con coordinate aggiornate');
hold off

%% Coverage path planning

fprintf('\nComputing areas exploration path...\n');
path1 = coveragePathPlanning(a1, fov, 0);
path2 = coveragePathPlanning(a2, fov, 0);
path3 = coveragePathPlanning(a3, fov, 1);

figure('Name', 'Coverage Path');
hold on;
fill(a1(b1, 1), a1(b1, 2), [1, 1, 0.4784], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
fill(a2(b2, 1), a2(b2, 2), [1, 1, 0.6196], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
fill(a3(b3, 1), a3(b3, 2), [1, 1, 0.7490], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
plot(path1(1, 1), path1(1, 2),'o', 'MarkerSize', 7, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
plot(path2(1, 1), path2(1, 2),'o', 'MarkerSize', 7, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
plot(path3(1, 1), path3(1, 2),'o', 'MarkerSize', 7, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
plot(path1(:,1), path1(:,2), '-', 'Color', 'r', 'LineWidth', 2);
plot(path2(:,1), path2(:,2), '-', 'Color', 'r', 'LineWidth', 2);
plot(path3(:,1), path3(:,2), '-', 'Color', 'r', 'LineWidth', 2);
hold off;
axis equal;
saveas(gcf, 'Plot/coverage_path.eps', 'epsc')

fprintf('Computing areas exploration path finished!\n');

%% Simulation (to be extended to three drones)

target_list = c3;

% Set here the first position - origin of the trajectory
pose_hist = [1,1,H(1,1)+offset,0];

% Following every coverage path points
for n = 1:size(target_list,1)
    
    % Coverage path point to be reached
    destin = [target_list(n,:), H(target_list(n,2),target_list(n,1))+offset, 0];

    i = 1;
    while i % movements to get there
        pose_actual = pose_hist(end,:);
        u = Drone_control(H,pose_actual,destin,dt,vmax,offset);

        % update the list of pose
        pose_new = Drone_Kine(H,pose_actual,u,dt,offset);
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

    n = n+1;
    
end

figure('Name','Ideal simulation path')
hold on
plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);
plot3(pose_hist(:, 1), pose_hist(:, 2), pose_hist(:, 3), '-or', LineWidth=2, MarkerSize=2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off


%% Update drone position function test

pose_hist_d1 = updateDronePosition(H, path1, sp_drone1, vmax, offset, dt, threshold);
pose_hist_d2 = updateDronePosition(H, path2, sp_drone2, vmax, offset, dt, threshold);
pose_hist_d3 = updateDronePosition(H, path3, sp_drone2, vmax, offset, dt, threshold);

figure('Name','Test function')
hold on
plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);
plot3(pose_hist_d1(:, 1), pose_hist_d1(:, 2), pose_hist_d1(:, 3), '-or', LineWidth=2, MarkerSize=2);
%plot3(pose_hist_d2(:, 1), pose_hist_d2(:, 2), pose_hist_d2(:, 3), '-or', LineWidth=2, MarkerSize=2);
%plot3(pose_hist_d3(:, 1), pose_hist_d3(:, 2), pose_hist_d3(:, 3), '-or', LineWidth=2, MarkerSize=2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off

%% Update drone position function test

pose_hist_d1 = updateDronePosition(H, c1, sp_drone1, vmax, offset, dt, threshold);
pose_hist_d2 = updateDronePosition(H, c2, sp_drone2, vmax, offset, dt, threshold);
pose_hist_d3 = updateDronePosition(H, c3, sp_drone2, vmax, offset, dt, threshold);

figure('Name','Test function')
hold on
plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);
plot3(pose_hist_d1(:, 1), pose_hist_d1(:, 2), pose_hist_d1(:, 3), '-or', LineWidth=2, MarkerSize=2);
plot3(pose_hist_d2(:, 1), pose_hist_d2(:, 2), pose_hist_d2(:, 3), '-oc', LineWidth=2, MarkerSize=2);
plot3(pose_hist_d3(:, 1), pose_hist_d3(:, 2), pose_hist_d3(:, 3), '-om', LineWidth=2, MarkerSize=2);
grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off


%% Test isObstacle function

figure;
imagesc(occupancyGrid);
hold on;

start = [randi(1000), randi(1000)];
goal = [randi(1000), randi(1000)];

test_path = [start; goal];

isObstacle(occupancyGrid, start, goal)

plot(test_path(:,1), test_path(:,2), '-or', LineWidth=2, MarkerSize=3);

axis equal;

%% RRT* path planning

map = occupancyGrid;

start = [10, 200];
goal = [randi(size(map, 1)), randi(size(map, 2))];

maxIterations = 800;
stepSize = 20;
radius = 60;

goalbias = 1;
bias = 0.4;
bias_radius = 200;
plot = 1;

[rrt_tree, rrt_path] = rrt_star(map, start, goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, plot);