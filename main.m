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
fov = 80;               % Camera FoV (m)
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
slope = rad2deg(atan(gradmod));

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
    pose1 = [pose1; Drone_Kine(H,pose1(i,:),u1,dt,0)];

    u2 = Drone_Verticalcontrol(H,pose2(i,:),v2(i,:),dt);
    pose2 = [pose2; Drone_Kine(H,pose2(i,:),u2,dt,0)];

    u3 = Drone_Verticalcontrol(H,pose3(i,:),v3(i,:),dt);
    pose3 = [pose3; Drone_Kine(H,pose3(i,:),u3,dt,0)];

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


%% My test dt update

victims = [1000 1000; randi(1000) randi(1000)];
n = 3; %numero droni

drone_victim = zeros(1,n);

conditions = true(1,n);
target_index = ones(1,n);
pose_hist = zeros(1,4,n); % lista posizioni + layer(drone)
pose_hist(1,:,1) = [sp_drone1, H(sp_drone1(2),sp_drone1(1))+offset,0];
pose_hist(1,:,2) = [sp_drone2, H(sp_drone2(2),sp_drone2(1))+offset,0];
pose_hist(1,:,3) = [sp_drone3, H(sp_drone3(2),sp_drone3(1))+offset,0];

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
    if time >= 1000
        error('Error. Taking over 1000 time steps to complete the simulation');
    end

end

% rendezvous
robot_depl = [200 1; 210 1; 220 1];
robot_id = 1;

for i = 1 : size(drone_victim,2)
    % if drone_victim(i) == 1
    %     continue
    % end

    pose_hist = updateDronePosition(H,robot_depl(i,:),pose_hist,vmax,offset,dt,threshold,i);

end

ind = find(pose_hist(:,1,1)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,1),1)
        pose_hist(i,:,1) = pose_hist(ind-1,:,1);
    end
end
ind = find(pose_hist(:,1,2)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,2),1)
        pose_hist(i,:,2) = pose_hist(ind-1,:,2);
    end
end
ind = find(pose_hist(:,1,3)==0, 1);
if( ~isempty(ind) )
    for i = ind:size(pose_hist(:,:,3),1)
        pose_hist(i,:,3) = pose_hist(ind-1,:,3);
    end
end



%% Plot untill search and radevouz

figure('Name','Test rendezvous')
hold on
plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand);

plot3(victims(2,1),victims(2,2),H(victims(2,2),victims(2,1))+offset, 'p', 'MarkerSize', 20, 'MarkerFaceColor', 'y');

fullrows = pose_hist(:,:,1);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-or', 'LineWidth', 2, 'MarkerSize', 2);
% % Loop attraverso ogni posizione e plotta la freccia di orientazione nel piano XY
% for i = 1:size(fullrows,1)
%     % Estrai l'angolo theta da ogni punto
%     theta = fullrows(i,4);
% 
%     % Calcola i componenti x e y della freccia di orientazione nel piano XY
%     x_comp = cos(theta)*scale;
%     y_comp = sin(theta)*scale;
% 
%     % Plotta la freccia di orientazione nel piano XY
%     quiver3(fullrows(i,1), fullrows(i,2), fullrows(i,3), x_comp, y_comp, 0, 'Color', 'k', 'LineWidth', 2);
% end

fullrows = pose_hist(:,:,2);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-oc', 'LineWidth', 2, 'MarkerSize', 2);
% % Loop attraverso ogni posizione e plotta la freccia di orientazione nel piano XY
% for i = 1:size(fullrows,1)
%     % Estrai l'angolo theta da ogni punto
%     theta = fullrows(i,4);
% 
%     % Calcola i componenti x e y della freccia di orientazione nel piano XY
%     x_comp = cos(theta)*scale;
%     y_comp = sin(theta)*scale;
% 
%     % Plotta la freccia di orientazione nel piano XY
%     quiver3(fullrows(i,1), fullrows(i,2), fullrows(i,3), x_comp, y_comp, 0, 'Color', 'k', 'LineWidth', 2);
% end

fullrows = pose_hist(:,:,3);
fullrows = fullrows(fullrows(:,3) ~= 0, :);
plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-om', 'LineWidth', 2, 'MarkerSize', 2);
% % Loop attraverso ogni posizione e plotta la freccia di orientazione nel piano XY
% for i = 1:size(fullrows,1)
%     % Estrai l'angolo theta da ogni punto
%     theta = fullrows(i,4);
% 
%     % Calcola i componenti x e y della freccia di orientazione nel piano XY
%     x_comp = cos(theta)*scale;
%     y_comp = sin(theta)*scale;
% 
%     % Plotta la freccia di orientazione nel piano XY
%     quiver3(fullrows(i,1), fullrows(i,2), fullrows(i,3), x_comp, y_comp, 0, 'Color', 'k', 'LineWidth', 2,'MaxHeadSize', 5);
% end


grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off


%% RRT* path planning

map = occupancyGrid;

start = robot_depl(2,:);
goal = victims(2,:);

maxIterations = 1200;
stepSize = 20;
radius = 60;

goalbias = 1;
bias = 0.4;
bias_radius = 200;
plot_ = 1;

[rrt_tree, rrt_path] = rrt_star(map, start, goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, plot_, slope);

% create full obstacle
new_obstacle_number = 1000; 
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


%% MOVEMENTS
clear path1d path2d path3d
offset_rdd = 20;
offset_rob = 1;
farward = fov/2;
lateral = fov/4;

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
%conditions(logical(drone_victim)) = false;
target_index = ones(1,n+1);
in_form = zeros(1,n);
%in_form = in_form + drone_victim;
rdd_hist = zeros(1,4,n+1); % position 4 is for the robot

%initialize
[col, ~] = find(pose_hist(:, :, 1), 1, 'last');
rdd_hist(1,:,1) = pose_hist(col,:,1);
[col, ~] = find(pose_hist(:, :, 2), 1, 'last');
rdd_hist(1,:,2) = pose_hist(col,:,2);
[col, ~] = find(pose_hist(:, :, 3), 1, 'last');
rdd_hist(1,:,3) = pose_hist(col,:,3);
rdd_hist(1,:,4) = [robot_depl(2,:), H(robot_depl(2,2),robot_depl(2,1))+offset_rob,0];


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
    end


    
    time = time + 1;
    if time >= 1000
        error('Error. Taking over 1000 time steps to complete the simulation');
    end

end

%% debug 
conditions = true(1,n+1);
%conditions(logical(drone_victim)) = false;

deb_time=zeros(time,1);

% formation moves followinf rrt
while conditions(4)
    % robot 1 step
    % new formation
    % drone moves formation new
    deb_time = [deb_time;time];

    % Robot 1 step
    if conditions(4)
        % Positions for each iteration
        target = [path4(2,:), H(path4(2,2),path4(2,1))+1, 0];
        actual_pose = rdd_hist(time,:,4);

        % compute control and kine for 1 step
        u = Drone_control(H,actual_pose,target,dt,vmax,0);
        new_pose = Drone_Kine(H,actual_pose,u,dt,offset_rdd,0);
        rdd_hist(time+1,:,4) = new_pose;

        % check if victim is reached
        if norm(new_pose(1:2) - goal) <= threshold
            fprintf('Vittima raggiunta, maestro\n');
            break;
        end

    end

    
    % insert here the new RRT* trajectory
    [rrt_tree, rrt_path] = rrt_star(occupancyGrid, new_pose(1:2), goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, 0, slope);
    path4 = rrt_path;

    % new formation positions  
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
    %in_form = in_form + drone_victim;

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
    
            occupancyLocal1 = zeros(size(occupancyGridComplete));
            camview = [round(new_pose(1))-fov/4 round(new_pose(1))+fov/4; round(new_pose(2)) round(new_pose(2))+fov/2];
            camview(camview < 1) = 1;
            camview(camview > 1000) = 1000;
            occupancyLocal1(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2)) = occupancyGridComplete(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2));
            
    
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
    
            occupancyLocal2 = zeros(size(occupancyGridComplete));
            camview = [round(new_pose(1))-fov/4 round(new_pose(1))+fov/4; round(new_pose(2)) round(new_pose(2))+fov/2];
            camview(camview < 1) = 1;
            camview(camview > 1000) = 1000;
            occupancyLocal2(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2)) = occupancyGridComplete(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2));
            

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
    
            occupancyLocal3 = zeros(size(occupancyGridComplete));
            camview = [round(new_pose(1))-fov/4 round(new_pose(1))+fov/4; round(new_pose(2)) round(new_pose(2))+fov/2];
            camview(camview < 1) = 1;
            camview(camview > 1000) = 1000;
            occupancyLocal3(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2)) = occupancyGridComplete(camview(1,1) : camview(1,2), camview(2,1) : camview(2,2));
            

            % check if it's reached
            if norm(target(1:3) - new_pose(1:3)) <= threshold
                in_form(3) = 1;
            end
        end
    
        % robot
        if rdd_hist(time+1,3,4) == 0
            rdd_hist(time+1,:,4) = rdd_hist(time,:,4);
        end

        cam = occupancyLocal1 | occupancyLocal2 | occupancyLocal3;
    
        time = time + 1;
        if time >= 1000
            error('Error. Taking over 1000 time steps to complete the simulation');
        end

    end

end

%% Plot rescue path
figure('Name','Test formation')
hold on

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);

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
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off

%% test plot

deb_time_reduced = deb_time(1:3:end);

figure('Name','Test formation')
hold on

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
colorbar;
daspect([1 1 0.5]);
view(3);

plot3(victims(2,1),victims(2,2),H(victims(2,2),victims(2,1))+10, 'p', 'MarkerSize', 20, 'MarkerFaceColor', 'y');

for i = 1:size(deb_time_reduced,1)
    if deb_time_reduced(i) == 0
        continue
    else
    fullrows1 = rdd_hist(deb_time_reduced(i), :, 1);
    fullrows1 = fullrows1(fullrows1(:, 3) ~= 0, :);
    
    fullrows2 = rdd_hist(deb_time_reduced(i), :, 2);
    fullrows2 = fullrows2(fullrows2(:, 3) ~= 0, :);
    
    fullrows3 = rdd_hist(deb_time_reduced(i), :, 3);
    fullrows3 = fullrows3(fullrows3(:, 3) ~= 0, :);
    
    % Plot dei punti
    plot3(fullrows1(:, 1), fullrows1(:, 2), fullrows1(:, 3), '-or', 'LineWidth', 2, 'MarkerSize', 2);
    plot3(fullrows2(:, 1), fullrows2(:, 2), fullrows2(:, 3), '-oc', 'LineWidth', 2, 'MarkerSize', 2);
    plot3(fullrows3(:, 1), fullrows3(:, 2), fullrows3(:, 3), '-om', 'LineWidth', 2, 'MarkerSize', 2);
    
    % Numero di punti
    numPoints = size(fullrows1, 1);
    
    for j = 1:numPoints
        % Coordinate dei punti
        x1 = fullrows1(j, 1);
        y1 = fullrows1(j, 2);
        z1 = fullrows1(j, 3);
        
        x2 = fullrows2(j, 1);
        y2 = fullrows2(j, 2);
        z2 = fullrows2(j, 3);
        
        x3 = fullrows3(j, 1);
        y3 = fullrows3(j, 2);
        z3 = fullrows3(j, 3);
        
        plot3([x1, x3], [y1, y3], [z1, z3], '-k', 'LineWidth', 1.5);
        
        xm = (x1 + x3) / 2;
        ym = (y1 + y3) / 2;
        zm = (z1 + z3) / 2;
        plot3([xm, x2], [ym, y2], [zm, z2], '-c', 'LineWidth', 1.5);
    end
        
        fullrows = rdd_hist(deb_time_reduced(i),:,4);
        fullrows = fullrows(fullrows(:,3) ~= 0, :);
        plot3(fullrows(:,1), fullrows(:,2), fullrows(:,3),'-ob', 'LineWidth', 2, 'MarkerSize', 2);
    end
end
grid on;
view(3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off


