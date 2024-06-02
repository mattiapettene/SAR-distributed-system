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

dt = 0.5;                       % Time step (s)
sp_drone1 = [1,1];              % Starting point drone 1
sp_drone2 = [1,10];             % Starting point drone 2
sp_drone3 = [10,1];             % Starting point drone 3
kp = 1/dt;                      % Voronoi kp parameter
threshold = 0.8;                % Set target accuracy [m]
vmax = [10,10,10];              % Drone maximum velocities (vx, vy, vz) [m/s]
offset = 100;                   % Distance from ground
fov = round(offset*tand(30));   % Camera FoV (m)

% RRT* parameters
maxIterations = 2000;
stepSize = 15;
radius = 25;
goalbias = 1;
bias = 0.1;
bias_radius = 200;
plt = 1;

% Kalman Filter parameters
A = eye(4);
B = eye(4)*dt;
P = 10^2;

% WLS parameters
nWLS = 3; 
sigma_cam = 0.1;
mWLS = 5;
pltWLS = 0;

%% Sensors uncertainty

% - Input
mu_u = zeros(1,4);
sigma_u = 0.1;
Q = [sigma_u,       0,          0,        0;
         0,       sigma_u,      0,        0;
         0,         0,      sigma_u,      0;
         0,         0,          0,     sigma_u];

% - GPS
mu_gps = zeros(1,4);
sigma_gps = 0.1;
R = [sigma_gps,       0,           0,         0;
          0,       sigma_gps,      0,         0;
          0,          0,      sigma_gps,      0;
          0,          0,           0,     sigma_gps];

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

H_flip = flipud(H);

figure('Name','3D map')
hold on;
mesh(H_flip);
colormap('summer');
contour3(H_flip, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);
saveas(gcf, 'Plot/3D_map.eps', 'epsc')

%% Map gradient matrix

mapWidth = s;
mapLength = s;

[grady,gradx] = gradient(H);
gradmod = sqrt(gradx.^2 + grady.^2);
slope = rad2deg(atan(gradmod));

figure('Name','Terrain slope');
contour(1:1:mapWidth,1:1:mapLength,H)
hold on
quiver(1:1:mapWidth,1:1:mapLength,gradx,grady)
axis equal
ax = gca;
ax.YDir = 'reverse';
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
    z_val = H(x_idx, y_idx);
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
    z_val = H(x_idx, y_idx);
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
            for k = max(1, i-toleranceLevel):min(mapWidth, i+toleranceLevel)
                for m = max(1, j-toleranceLevel):min(mapLength, j+toleranceLevel)
                    new_grid(k, m) = 1;
                end
            end
        end
    end
end

occupancyGrid = new_grid;
clear new_grid;

figure('Name','Initial occupancy map');
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
    
    plot3(yTrunk, xTrunk, zTrunk, 'Color', '#53350A', 'LineWidth', 1);
    plot3(yCrown, xCrown, zCrown, 'Color', '#2A7E19', 'LineWidth', 1);

end

grid on;
axis equal;
view(3);
ax = gca;
ax.YDir = 'reverse';
hold off;
saveas(gcf, 'Plot/3D_map_trees.eps', 'epsc')

figure('Name','Real trees location')
hold on
spy(double(BW))
axis equal
hImg = imshow(I, 'InitialMagnification', 'fit'); 
set(hImg, 'AlphaData', 0.6);
spy(BW);
hold off
saveas(gcf, 'Plot/real_trees_satellite.eps', 'epsc')

warning('off', 'images:imshow:magnificationMustBeFitForDockedFigure')

%% Voronoi tessellation

fprintf('\nStarting Voronoi coverage iteration...\n');
[c1, c2, c3, v1, v2, v3, a1, a2, a3, b1, b2, b3] = voronoi_coverage(H, sp_drone1, sp_drone2, sp_drone3, kp);
fprintf('Voronoi coverage iteration finished!\n');

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

%% Searching for victim

victims = [980 980; randi([20,980]) randi([20,980])];
n = 3; % numero droni
drone_victim = zeros(1,n);
warning('off', 'all');

conditions = true(1,n);
target_index = ones(1,n);
pose_hist = zeros(1,4,n); % lista posizioni + layer(drone)
pose_hist(1,:,1) = [sp_drone1, H(sp_drone1(2),sp_drone1(1))+offset,0];
pose_hist(1,:,2) = [sp_drone2, H(sp_drone2(2),sp_drone2(1))+offset,0];
pose_hist(1,:,3) = [sp_drone3, H(sp_drone3(2),sp_drone3(1))+offset,0];

% Kalman Filter initialization
pose_est_hist = zeros(1, 4, 4);
pose_est_hist(1,:,1) = [sp_drone1, H(sp_drone1(2),sp_drone1(1))+offset,0];
pose_est_hist(1,:,2) = [sp_drone2, H(sp_drone2(2),sp_drone2(1))+offset,0];
pose_est_hist(1,:,3) = [sp_drone3, H(sp_drone3(2),sp_drone3(1))+offset,0];
pose_gps_hist = zeros(1, 4, 4);
pose_gps_hist(1,:,1) = [sp_drone1, H(sp_drone1(2),sp_drone1(1))+offset,0] + randn(1,4)*R + mu_gps;
pose_gps_hist(1,:,2) = [sp_drone2, H(sp_drone2(2),sp_drone2(1))+offset,0] + randn(1,4)*R + mu_gps;
pose_gps_hist(1,:,3) = [sp_drone3, H(sp_drone3(2),sp_drone3(1))+offset,0] + randn(1,4)*R + mu_gps;


time = 1;
while sum(conditions)
    % Drone 1
    if conditions(1)
        % Positions for each iteration
        target = [path1(target_index(1),:), H(path1(target_index(1),2),path1(target_index(1),1))+offset, 0];
        actual_pose = pose_hist(time,:,1);

        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target,dt,vmax,offset);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset,0);
        pose_hist(time+1,:,1) = new_pose;
        
        % Kalman Filter
        [pose_gps_hist, pose_est_hist, P] = KalmanFilter(1, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid);

        % check if it's reached
        if norm(target(1:3) - new_pose(1:3)) <= threshold
            target_index(1) = target_index(1) + 1;
        end
        if target_index(1) == size(path1,1)+1
            conditions(1) = false;
        end

        dist = pdist2(new_pose(1:2),victims);
        mindist = min(dist);
        if mindist <= fov/2 + 1
            drone_victim(1) = 1;
            break;
        end

    end

    % Drone 2
    if conditions(2)
        % Positions for each iteration
        target = [path2(target_index(2),:), H(path2(target_index(2),2),path2(target_index(2),1))+offset, 0];
        actual_pose = pose_hist(time,:,2);

        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target,dt,vmax,offset);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset,0);
        pose_hist(time+1,:,2) = new_pose;

        % Kalman Filter
        [pose_gps_hist, pose_est_hist, P] = KalmanFilter(2, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid);

        % check if it's reached
        if norm(target(1:3) - new_pose(1:3)) <= threshold
            target_index(2) = target_index(2) + 1;
        end
        if target_index(2) == size(path2,1)+1
            conditions(2) = false;
        end

        dist = pdist2(new_pose(1:2),victims);
        mindist = min(dist);
        if mindist <= fov/2 + 1
            drone_victim(2) = 1;
            break;
        end
    end

    % Drone 3
    if conditions(3)
        % Positions for each iteration
        target = [path3(target_index(3),:), H(path3(target_index(3),2),path3(target_index(3),1))+offset, 0];
        actual_pose = pose_hist(time,:,3);

        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target,dt,vmax,offset);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset,0);
        pose_hist(time+1,:,3) = new_pose;

        % Kalman Filter
        [pose_gps_hist, pose_est_hist, P] = KalmanFilter(3, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid);

        % check if it's reached
        if norm(target(1:3) - new_pose(1:3)) <= threshold
            target_index(3) = target_index(3) + 1;
        end
        if target_index(3) == size(path3,1)+1
            conditions(3) = false;
        end

        dist = pdist2(new_pose(1:2),victims);
        mindist = min(dist);
        if mindist <= fov/2 + 1
            drone_victim(3) = 1;
            break;
        end
    end
    
    time = time + 1;
    if time >= 10000
        error('Error. Taking over 10000 time steps to complete the simulation');
    end

end

%% Rendezvous
robot_depl = [200 1; 210 1; 220 1];
robot_id = 1;

for i = 1 : size(drone_victim,2)
    [pose_hist, pose_gps_hist, pose_est_hist, P] = updateDronePosition(H,robot_depl(i,:),pose_hist,vmax,offset,dt,threshold,i, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyGrid);
end

ind = find(pose_hist(:,1,1)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,1),1)
        pose_hist(i,:,1) = pose_hist(ind-1,:,1);
        pose_est_hist(i,:,1) = pose_est_hist(ind-1,:,1);
        pose_gps_hist(i,:,1) = pose_gps_hist(ind-1,:,1);
    end
end
ind = find(pose_hist(:,1,2)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,2),1)
        pose_hist(i,:,2) = pose_hist(ind-1,:,2);
        pose_est_hist(i,:,2) = pose_est_hist(ind-1,:,2);
        pose_gps_hist(i,:,2) = pose_gps_hist(ind-1,:,2);
    end
end
ind = find(pose_hist(:,1,3)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,3),1)
        pose_hist(i,:,3) = pose_hist(ind-1,:,3);
        pose_est_hist(i,:,3) = pose_est_hist(ind-1,:,3);
        pose_gps_hist(i,:,3) = pose_gps_hist(ind-1,:,3);
    end
end

%% Plots 
figure('Name','Research and Randezvous')
hold on
plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);

plot3(victims(2,1),victims(2,2),H(victims(2,2),victims(2,1))+offset, 'p', 'MarkerSize', 20, 'MarkerFaceColor', 'y');

fullrows = pose_hist(:,:,1);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-or', 'LineWidth', 2, 'MarkerSize', 2);

fullrows = pose_hist(:,:,2);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-oc', 'LineWidth', 2, 'MarkerSize', 2);

fullrows = pose_hist(:,:,3);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-om', 'LineWidth', 2, 'MarkerSize', 2);
grid on;
view(3);
ax = gca;
ax.YDir = 'reverse';
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off

saveas(gcf, 'Plot/Research_and_Randezvous_path.eps', 'epsc');

%% RRT* path planning

map = occupancyGrid;

start = robot_depl(2,:);
goal = victims(2,:);

[rrt_tree, rrt_path] = rrt_star(map, start, goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, plt, slope);

%% New random obstacle addition

new_obstacle_number = 300; 
new_occupancy = false(size(occupancyGrid));

rand_pos = randperm(numel(occupancyGrid), new_obstacle_number);

new_occupancy(rand_pos) = true;

% Set occupancy around tree locations
toleranceLevel = 1;
new_grid = new_occupancy;

for i = 1:mapLength
    for j = 1:mapWidth
        % find the positive ones
        if new_occupancy(i, j) == 1
            % change value around
            for k = max(1, i-toleranceLevel):min(mapWidth, i+toleranceLevel)
                for m = max(1, j-toleranceLevel):min(mapLength, j+toleranceLevel)
                    new_grid(k, m) = 1;
                end
            end
        end
    end
end

new_occupancy = new_grid;
occupancyGridComplete = new_occupancy | occupancyGrid;
clear new_grid new_occupancy rand_pos;

figure('Name','Complete occupancy grid')
spy(occupancyGridComplete);

%% Victim rescue

% Moving into formation 
clear path1d path2d path3d
offset_rdd = offset;
offset_rob = 1;
farward = fov/2;
lateral = fov/4;

cam_plot = zeros(1000, 1000);

path4 = rrt_path;
theta = atan2((rrt_path(2,2)-rrt_path(1,2)),(rrt_path(2,1)-rrt_path(1,1)));

% build formation
path1d(:,1) = rrt_path(:, 1) + cos(pi/2+theta)*lateral;
path1d(:,2) = rrt_path(:, 2) + sin(pi/2+theta)*lateral;
path2d(:,1) = rrt_path(:, 1) + cos(theta)*farward;
path2d(:,2) = rrt_path(:, 2) + sin(theta)*farward;
path3d(:,1) = rrt_path(:, 1) + cos(-pi/2+theta)*lateral;
path3d(:,2) = rrt_path(:, 2) + sin(-pi/2+theta)*lateral;

% anti negative
path1d(path1d < 1) = 1;
path2d(path2d < 1) = 1;
path3d(path3d < 1) = 1;
path1d(path1d > 1000) = 1000;
path2d(path2d > 1000) = 1000;
path3d(path3d > 1000) = 1000;

conditions = true(1,n+1);
target_index = ones(1,n+1);
in_form = zeros(1,n);
rdd_hist = zeros(1,4,n+1); % position 4 is for the robot

%initialize (drones)
%[col, ~] = find(pose_hist(:, :, 1), 1, 'last');
rdd_hist(1,:,1) = pose_hist(end,:,1);
%[col, ~] = find(pose_hist(:, :, 2), 1, 'last');
rdd_hist(1,:,2) = pose_hist(end,:,2);
%[col, ~] = find(pose_hist(:, :, 3), 1, 'last');
rdd_hist(1,:,3) = pose_hist(end,:,3);
rdd_hist(1,:,4) = [robot_depl(2,:), H(robot_depl(2,2),robot_depl(2,1))+offset_rob,0];

% Kalman Filter initialization (drones)
rdd_est_hist = zeros(1, 4, 4);
rdd_est_hist(1,:,1) = pose_est_hist(end,:,1);
rdd_est_hist(1,:,2) = pose_est_hist(end,:,2);
rdd_est_hist(1,:,3) = pose_est_hist(end,:,3);
rdd_est_hist(1,:,4) = rdd_hist(1,:,4);
rdd_gps_hist = zeros(1, 4, 4);
rdd_gps_hist(1,:,1) = pose_gps_hist(end,:,1);
rdd_gps_hist(1,:,2) = pose_gps_hist(end,:,2);
rdd_gps_hist(1,:,3) = pose_gps_hist(end,:,3);
rdd_gps_hist(1,:,4) = rdd_hist(1,:,4);

target1 = [path1d(target_index(1),:), interp2(H,path1d(target_index(1),1),path1d(target_index(1),2), 'linear')+offset_rdd, 0];
target2 = [path2d(target_index(2),:), interp2(H,path2d(target_index(2),1),path2d(target_index(2),2), 'linear')+offset_rdd, 0];
target3 = [path3d(target_index(3),:), interp2(H,path3d(target_index(3),1),path3d(target_index(3),2), 'linear')+offset_rdd, 0];

time = 1;
while sum(in_form) < 3
    
    % drone 1
    if conditions(1) && in_form(1) == 0
        actual_pose = rdd_hist(time,:,1);
        
        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target1,dt,vmax,offset_rdd);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
        rdd_hist(time+1,:,1) = new_pose;
        
        % Kalman Filter
        [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(1, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
        
        % check if it's reached
        if norm(target1(1:3) - new_pose(1:3)) <= threshold
            in_form(1) = 1;
            target_index(1) = target_index(1) + 1;
        end

    elseif conditions(1) || in_form(1) == 1
        rdd_hist(time+1,:,1) = rdd_hist(time,:,1);
    end

    % drone 2
    if conditions(2) && in_form(2) == 0
        actual_pose = rdd_hist(time,:,2);
        
        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target2,dt,vmax,offset_rdd);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
        rdd_hist(time+1,:,2) = new_pose;

        % Kalman Filter
        [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(2, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
        
        % check if it's reached
        if norm(target2(1:3) - new_pose(1:3)) <= threshold
            in_form(2) = 1;
            target_index(2) = target_index(2) + 1;
        end

    elseif conditions(2) || in_form(2) == 1
        rdd_hist(time+1,:,2) = rdd_hist(time,:,2);
    end

    % drone 3
    if conditions(3) && in_form(3) == 0
        actual_pose = rdd_hist(time,:,3);
        
        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target3,dt,vmax,offset_rdd);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
        rdd_hist(time+1,:,3) = new_pose;

        % Kalman Filter
        [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(3, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
        
        % check if it's reached
        if norm(target3(1:3) - new_pose(1:3)) <= threshold
            in_form(3) = 1;
            target_index(3) = target_index(3) + 1;
        end

    elseif conditions(3) || in_form(3) == 1
        rdd_hist(time+1,:,3) = rdd_hist(time,:,3);
    end

    % robot
    if sum(in_form) <= 3
        rdd_hist(time+1,:,4) = rdd_hist(time,:,4);
        rdd_gps_hist(time+1,:,4) = rdd_gps_hist(time,:,4);
        rdd_est_hist(time+1,:,4) = rdd_est_hist(time,:,4);
    end

    time = time + 1;
    if time >= 100000
        error('Error. Taking over 10000 time steps to complete the simulation');
    end

end

ind = find(rdd_hist(:,1,1)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_hist(:,:,1),1)
        rdd_hist(i,:,1) = rdd_hist(ind-1,:,1);
        rdd_est_hist(i,:,1) = rdd_est_hist(ind-1,:,1);
        rdd_gps_hist(i,:,1) = rdd_gps_hist(ind-1,:,1);
    end
end

ind = find(rdd_hist(:,1,1)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_hist(:,:,1),1)
        rdd_hist(i,:,1) = rdd_hist(ind-1,:,1);
        rdd_est_hist(i,:,1) = rdd_est_hist(ind-1,:,1);
        rdd_gps_hist(i,:,1) = rdd_gps_hist(ind-1,:,1);
    end
end
ind = find(rdd_hist(:,1,2)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_hist(:,:,2),1)
        rdd_hist(i,:,2) = rdd_hist(ind-1,:,2);
        rdd_est_hist(i,:,2) = rdd_est_hist(ind-1,:,2);
        rdd_gps_hist(i,:,2) = rdd_gps_hist(ind-1,:,2);
    end
end
ind = find(rdd_hist(:,1,3)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_hist(:,:,3),1)
        rdd_hist(i,:,3) = rdd_hist(ind-1,:,3);
        rdd_est_hist(i,:,3) = rdd_est_hist(ind-1,:,3);
        rdd_gps_hist(i,:,3) = rdd_gps_hist(ind-1,:,3);
    end
end

for j = 1:4
ind = find(rdd_hist(:,1,j)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_hist(:,:,j),1)
        rdd_hist(i,:,1) = rdd_hist(ind-1,:,j);
    end
end
ind = find(rdd_est_hist(:,1,j)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_est_hist(:,:,j),1)
        rdd_est_hist(i,:,1) = rdd_est_hist(ind-1,:,j);
    end
end
ind = find(rdd_gps_hist(:,1,j)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(rdd_gps_hist(:,:,j),1)
        rdd_gps_hist(i,:,1) = rdd_gps_hist(ind-1,:,j);
    end
end

end

% Moving towards victim
conditions = true(1,n+1);

% formation moves following rrt
while conditions(4)
    % robot 1 step
    % new formation
    % drone moves formation new

    % Robot 1 step
    if conditions(4)
        % Positions for each iteration
        target = [path4(2,:), H(path4(2,2),path4(2,1))+1, 0];
        actual_pose = rdd_hist(time,:,4);

        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target,dt,vmax,0);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
        rdd_hist(time+1,:,4) = new_pose;

        % Kalman Filter
        [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(4, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);

        % check if victim is reached
        if norm(new_pose(1:2) - goal) <= threshold
            fprintf('\nVictim reached!\n');
            break;
        end

    end

    
    % New RRT* trajectory
    [rrt_tree, rrt_path] = rrt_star(occupancyGrid, new_pose(1:2), goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, 0, slope);
    path4 = rrt_path;

    % New formation positions  
    theta = atan2((path4(2,2)-path4(1,2)),(path4(2,1)-path4(1,1)));

    clear path1d path2d path3d
    % build formation
    path1d(:,1) = rrt_path(:, 1) + cos(pi/2+theta)*lateral;
    path1d(:,2) = rrt_path(:, 2) + sin(pi/2+theta)*lateral;
    path2d(:,1) = rrt_path(:, 1) + cos(theta)*farward;
    path2d(:,2) = rrt_path(:, 2) + sin(theta)*farward;
    path3d(:,1) = rrt_path(:, 1) + cos(-pi/2+theta)*lateral;
    path3d(:,2) = rrt_path(:, 2) + sin(-pi/2+theta)*lateral;
    
    % anti negative
    path1d(path1d < 1) = 1;
    path2d(path2d < 1) = 1;
    path3d(path3d < 1) = 1;
    path1d(path1d > 1000) = 1000;
    path2d(path2d > 1000) = 1000;
    path3d(path3d > 1000) = 1000;
    
    in_form = zeros(1,n);

    while sum(in_form) < 3
        % Drone 1
        if conditions(1)
            % Positions for each iteration
            target = [path1d(1,:), interp2(H,path1d(1,1),path2d(1,2), 'linear')+offset_rdd, 0];
            actual_pose = rdd_hist(time,:,1);
    
            % compute control and kine for 1 step
            u = Drone_control(H,actual_pose,target,dt,vmax,offset_rdd);
            new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
            rdd_hist(time+1,:,1) = new_pose;
            
            % Kalman Filter
            [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(1, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
    
            occupancyLocal1 = getCameraView(1,new_pose,occupancyGridComplete,theta,fov);

            
            % check if it's reached
            if norm(target(1:3) - new_pose(1:3)) <= threshold
                in_form(1) = 1;
            end
        end
    
        % Drone 2
        if conditions(2)
            % Positions for each iteration
            target = [path2d(1,:), interp2(H,path2d(1,1),path2d(1,2), 'linear')+offset_rdd, 0];
            actual_pose = rdd_hist(time,:,2);
    
            % compute control and kine for 1 step
            u = Drone_control(H,actual_pose,target,dt,vmax,offset_rdd);
            new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
            rdd_hist(time+1,:,2) = new_pose;

            % Kalman Filter
            [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(2, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
    
            occupancyLocal2 = getCameraView(2,new_pose,occupancyGridComplete,theta,fov);
            
            % check if it's reached
            if norm(target(1:3) - new_pose(1:3)) <= threshold
                in_form(2) = 1;
            end
        end
    
        % Drone 3
        if conditions(3)
            % Positions for each iteration
            target = [path3d(1,:), interp2(H,path3d(1,1),path3d(1,2), 'linear')+offset_rdd, 0];
            actual_pose = rdd_hist(time,:,3);
    
            % compute control and kine for 1 step
            u = Drone_control(H,actual_pose,target,dt,vmax,offset_rdd);
            new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
            rdd_hist(time+1,:,3) = new_pose;

            % Kalman Filter
            [rdd_gps_hist, rdd_est_hist, P] = KalmanFilter(3, time, u, new_pose, actual_pose, Q, R, mu_gps, mu_u, A, B, P, rdd_gps_hist, rdd_est_hist, occupancyGrid);
    
            occupancyLocal3 = getCameraView(3,new_pose,occupancyGridComplete,theta,fov);
            
            % check if it's reached
            if norm(target(1:3) - new_pose(1:3)) <= threshold
                in_form(3) = 1;
            end
        end
    
        % robot
        if rdd_hist(time+1,3,4) == 0
            rdd_hist(time+1,:,4) = rdd_hist(time,:,4);
            rdd_est_hist(time+1,:,4) = rdd_est_hist(time,:,4);
            rdd_gps_hist(time+1,:,4) = rdd_gps_hist(time,:,4);
        end

        cam = occupancyLocal1 & occupancyLocal2 & occupancyLocal3;
        cam_plot = cam_plot | cam;
        
        H_detected = WLS_distributed(cam, nWLS, sigma_cam, mWLS, pltWLS);
        occupancyGrid = occupancyGrid | H_detected;
    
        time = time + 1;
        if time >= 100000
            error('Error. Taking over 10000 time steps to complete the simulation');
        end
        
    end

end

%% Plot rescue path
figure('Name','Rescue path')
hold on

plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);

plot3(victims(2,1),victims(2,2),H(victims(2,2),victims(2,1))+10, 'p', 'MarkerSize', 20, 'MarkerFaceColor', 'y');

fullrows = rdd_hist(:,:,1);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-or', 'LineWidth', 2, 'MarkerSize', 2);

fullrows = rdd_hist(:,:,2);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-oc', 'LineWidth', 2, 'MarkerSize', 2);

fullrows = rdd_hist(:,:,3);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-om', 'LineWidth', 2, 'MarkerSize', 2);

fullrows = rdd_hist(:,:,4);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-ob', 'LineWidth', 2, 'MarkerSize', 2);

grid on;
view(3);
ax = gca;
ax.YDir = 'reverse';
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off
saveas(gcf, 'Plot/Rescue_Path.eps', 'epsc');

%% Plot 2D obtacles and path 

cam_vec = [];
obs_vec = [];

for i = 1:1000
    for j = 1:1000
        if cam_plot(i,j) == 1
            cam_vec = [cam_vec; i,j];
        end
        if occupancyGrid(i,j) == 1
            obs_vec = [obs_vec; i,j];
        end
    end
end

figure('Name', 'Obstacles Camera and Path');
hold on;
plot(obs_vec(:,2), obs_vec(:,1), '.', 'Color', '#0072BD');
plot(cam_vec(:,2), cam_vec(:,1), '.', 'Color', '#D95319');
plot(rdd_hist(:, 1, 4), rdd_hist(:, 2, 4), '-', 'Color', '#EDB120', 'LineWidth', 0.5);
axis equal;
ax = gca;
ax.YDir = 'reverse';
xlim([0 1000]);
ylim([0 1000]);
legend('Priori known obstacles', 'Camera detected obstacles', 'Robot path', 'Location', 'eastoutside');

%% Plot Real vs GPS vs Estimated trajectories end errors

% ---------------------------- DRONES ----------------------------------

all_hist = [pose_hist(:,:,1:3); rdd_hist(:,:,1:3)];
all_gps_hist = [pose_gps_hist(:,:,1:3); rdd_gps_hist(:,:,1:3)];
all_est_hist = [pose_est_hist(:,:,1:3); rdd_est_hist(:,:,1:3)];

for d = 1:3  % Plot for each drone

% Trajectories X
figure('Name', sprintf('x Position Drone %d', d));
hold on;
plot(all_hist(1:end-1,1,d), '-r'); % true
plot(all_gps_hist(1:end-1,1,d), '-b'); % gps 
plot(all_est_hist(1:end-1,1,d), '-g'); % estimated
legend('xTrue', 'xGPS', 'xEst');
xlabel('Time');
ylabel('x Position');
saveas(gcf, sprintf('Plot/x_Position_Drone_%d.eps', d), 'epsc')

% Error histogram X 
figure('Name', sprintf('x Error Drone %d', d));
histogram(all_est_hist(1:end-1,1,d)-all_hist(1:end-1,1,d));
saveas(gcf, sprintf('Plot/x_Error_Drone_%d.eps', d), 'epsc')

% Trajectories Y
figure('Name', sprintf('y Position Drone %d', d));
hold on;
plot(all_hist(1:end-1,2,d), '-r'); % true
plot(all_gps_hist(1:end-1,2,d), '-b'); % gps 
plot(all_est_hist(1:end-1,2,d), '-g'); % estimated
legend('yTrue', 'yGPS', 'yEst');
xlabel('Time');
ylabel('y Position');
saveas(gcf, sprintf('Plot/y_Position_Drone_%d.eps', d), 'epsc')

% Error histogram Y 
figure('Name', sprintf('y Error Drone %d', d));
histogram(all_est_hist(1:end-1,2,d)-all_hist(1:end-1,2,d));
saveas(gcf, sprintf('Plot/y_Error_Drone_%d.eps', d), 'epsc')

% Trajectories Z
figure('Name', sprintf('z Position Drone %d', d));
hold on;
plot(all_hist(1:end-1,3,d), '-r'); % true
plot(all_gps_hist(1:end-1,3,d), '-b'); % gps 
plot(all_est_hist(1:end-1,3,d), '-g'); % estimated
legend('zTrue', 'zGPS', 'zEst');
xlabel('Time');
ylabel('z Position');
saveas(gcf, sprintf('Plot/z_Position_Drone_%d.eps', d), 'epsc')

% Error histogram Z 
figure('Name', sprintf('z Error Drone %d', d));
histogram(all_est_hist(:,3,d)-all_hist(:,3,d));
saveas(gcf, sprintf('Plot/z_Error_Drone_%d.eps', d), 'epsc')

% Trajectories theta
figure('Name', sprintf('theta Drone %d', d));
hold on;
plot(all_hist(1:end-1,4,d), '-r'); % true
plot(all_gps_hist(1:end-1,4,d), '-b'); % gps 
plot(all_est_hist(1:end-1,4,d), '-g'); % estimated
legend('$\theta$ True', '$\theta$ GPS', '$\theta$ Est');
xlabel('Time');
ylabel('$\theta$');
saveas(gcf, sprintf('Plot/theta_Drone_%d.eps', d), 'epsc')

% Error histogram theta
figure('Name', sprintf('theta Error Drone %d', d));
histogram(all_est_hist(1:end-1,4,d)-all_hist(1:end-1,4,d));
saveas(gcf, sprintf('Plot/thata_Error_Drone_%d.eps', d), 'epsc')

end

% ---------------------------- ROBOT ----------------------------------

% Trajectories X
figure('Name', 'x Position robot');
hold on;
plot(rdd_hist(1:end-1,1,4), '-r'); % true
plot(rdd_gps_hist(1:end-1,1,4), '-b'); % gps 
plot(rdd_est_hist(1:end-1,1,4), '-g'); % estimated
legend('xTrue', 'xGPS', 'xEst');
xlabel('Time');
ylabel('x Position');
saveas(gcf, 'Plot/x_Position_Robot.eps', 'epsc');

% Error histogram X 
figure('Name','x Error robot');
histogram(rdd_est_hist(1:end-1,1,4)-rdd_hist(1:end-1,1,4));
saveas(gcf, 'Plot/x_Error_Robot.eps', 'epsc');

% Trajectories Y
figure('Name', 'y Position robot');
hold on;
plot(rdd_hist(1:end-1,2,4), '-r'); % true
plot(rdd_gps_hist(1:end-1,2,4), '-b'); % gps 
plot(rdd_est_hist(1:end-1,2,4), '-g'); % estimated
legend('yTrue', 'yGPS', 'yEst');
xlabel('Time');
ylabel('y Position');
saveas(gcf, 'Plot/y_Position_Robot.eps', 'epsc');

% Error histogram Y 
figure('Name', 'y Error robot');
histogram(rdd_est_hist(1:end-1,2,4)-rdd_hist(1:end-1,2,4));
saveas(gcf, 'Plot/y_Error_Robot.eps', 'epsc');

% Trajectories Z
figure('Name', 'z Position robot');
hold on;
plot(rdd_hist(1:end-1,3,4), '-r'); % true
plot(rdd_gps_hist(1:end-1,3,4), '-b'); % gps 
plot(rdd_est_hist(1:end-1,3,4), '-g'); % estimated
legend('zTrue', 'zGPS', 'zEst');
xlabel('Time');
ylabel('z Position');
saveas(gcf, 'Plot/z_Position_Robot.eps', 'epsc');

% Error histogram Z 
figure('Name', 'z Error robot');
histogram(rdd_est_hist(:,3,4)-rdd_hist(:,3,4));
saveas(gcf, 'Plot/z_Error_Robot.eps', 'epsc');

% Trajectories theta
figure('Name', 'theta robot');
hold on;
plot(rdd_hist(1:end-1,4,4), '-r'); % true
plot(rdd_gps_hist(1:end-1,4,4), '-b'); % gps 
plot(rdd_est_hist(1:end-1,4,4), '-g'); % estimated
legend('$\theta$ True', '$\theta$ GPS', '$\theta$ Est');
xlabel('Time');
ylabel('$\theta$');
saveas(gcf, 'Plot/theta_Robot.eps', 'epsc');

% Error histogram theta
figure('Name', 'theta Error robot');
histogram(rdd_est_hist(1:end-1,4,4)-rdd_hist(1:end-1,4,4));
saveas(gcf, 'Plot/theta_Error_Robot.eps', 'epsc');


%% Save time and distance to compare different simulations

n_time_steps = size(rdd_hist(:, 1, 4), 1);                  % Number of time steps
t_s = n_time_steps*dt;                                      % Time in seconds
d = vecnorm(robot_depl(2, 1:2)-victims(2, 1:2), 2, 2);      % Distance robot-victim
index = t_s/d;

% Save to file
filename = 'data.txt';
file = fopen(filename, 'a');

fprintf(file, '%d\n', index);

fclose(file);

%% Plot
f = load("data.txt");
val = f(:,1);
figure('Name', 'Comparison')
histogram(val);

