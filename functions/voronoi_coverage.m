function [c1, c2, c3, v1, v2, v3, a1, a2, a3, b1, b2, b3] = voronoi_coverage(H, sp_drone1, sp_drone2, sp_drone3, kp)

% OUTPUTS
% c1, c2, c3 -> evolution of the centroids of the three areas
% a1, a2, a3 -> areas to be explored for each drone
% b1, b2, b3 -> bounds of the areas

% INPUTS
% H -> map matrix
% sp_drone1, sp_drone2, sp_drone3 -> starting points [x,y] for each drone
% kp -> tuning parameter for Lloyd control

[X, Y] = meshgrid(1:size(H, 1), 1:size(H, 2));
voronoi_grid = [X(:), Y(:)];

centroid_x1 = [sp_drone1(2)];
centroid_y1 = [sp_drone1(1)];
centroid_x2 = [sp_drone2(2)];
centroid_y2 = [sp_drone2(1)];
centroid_x3 = [sp_drone3(2)];
centroid_y3 = [sp_drone3(1)];
vel_lloyd1 = [];
vel_lloyd2 = [];
vel_lloyd3 = [];

niter = 1;
voronoi = 1;

while voronoi

    starting_points = [sp_drone1; sp_drone2; sp_drone3];
    
    % Compute distances between starting points and each point on the grid
    distances = pdist2(voronoi_grid, starting_points);
    
    % Find closer starting point index for each point on the grid
    [~, minimum_indices] = min(distances, [], 2);
    
    % Assign index to each point on the grid
    indices_cell = reshape(minimum_indices, size(H));
    
    %fprintf('Voronoi iter: %d\n', niter);
    
    % Identificare i punti all'interno di ciascuna regione
    points_area1 = voronoi_grid(indices_cell(:) == 1, :);
    points_area2 = voronoi_grid(indices_cell(:) == 2, :);
    points_area3 = voronoi_grid(indices_cell(:) == 3, :);
    
    % Compute centroids for each Voronoi region
    centroid_area1 = round(mean(points_area1));
    centroid_area2 = round(mean(points_area2));
    centroid_area3 = round(mean(points_area3));

    % Compute drones velocities using Lloyd control law
    vel_1 = - kp * (sp_drone1 - centroid_area1);
    vel_2 = - kp * (sp_drone2 - centroid_area2);
    vel_3 = - kp * (sp_drone3 - centroid_area3);

    % Save points
    centroid_x1 = [centroid_x1; sp_drone1(2)];
    centroid_y1 = [centroid_y1; sp_drone1(1)];
    centroid_x2 = [centroid_x2; sp_drone2(2)];
    centroid_y2 = [centroid_y2; sp_drone2(1)];
    centroid_x3 = [centroid_x3; sp_drone3(2)];
    centroid_y3 = [centroid_y3; sp_drone3(1)];
    vel_lloyd1 = [vel_lloyd1; vel_1];
    vel_lloyd2 = [vel_lloyd2; vel_2];
    vel_lloyd3 = [vel_lloyd3; vel_3];
    
    % figure('Name', 'Voronoi');
    % imagesc(indices_cell);
    % hold on;
    % plot(centroid_area1(2), centroid_area1(1), 'k.', 'MarkerSize', 10);
    % plot(centroid_area2(2), centroid_area2(1), 'k.', 'MarkerSize', 10);
    % plot(centroid_area3(2), centroid_area3(1), 'k.', 'MarkerSize', 10);
    % hold off;
    % axis equal

    sp_drone1_old = sp_drone1;
    sp_drone2_old = sp_drone2;
    sp_drone3_old = sp_drone3;

    sp_drone1 = centroid_area1;
    sp_drone2 = centroid_area2;
    sp_drone3 = centroid_area3;

    niter = niter + 1;

    if all(sp_drone1-sp_drone1_old == 0) && all(sp_drone2-sp_drone2_old == 0) && all(sp_drone3-sp_drone3_old == 0)
        voronoi = 0;
    end
    
end

% Compute bounds for the three areas
boundary1 = boundary(points_area1(:, 1), points_area1(:, 2));
boundary2 = boundary(points_area2(:, 1), points_area2(:, 2));
boundary3 = boundary(points_area3(:, 1), points_area3(:, 2));

% Plot Voronoi tessellation and drones trajectories
figure('Name', 'Voronoi');
hold on;
fill(points_area1(boundary1, 1), points_area1(boundary1, 2), [1, 1, 0.4784], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
fill(points_area2(boundary2, 1), points_area2(boundary2, 2), [1, 1, 0.6196], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
fill(points_area3(boundary3, 1), points_area3(boundary3, 2), [1, 1, 0.7490], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 2);
plot(centroid_area1(2), centroid_area1(1),'o', 'MarkerSize', 20, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.7, 0.7, 0.7]);
plot(centroid_area2(2), centroid_area2(1), 'o', 'MarkerSize', 20, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.7, 0.7, 0.7]);
plot(centroid_area3(2), centroid_area3(1), 'o', 'MarkerSize', 20, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.7, 0.7, 0.7]);
% plot(centroid_x1, centroid_y1, 'k--', 'LineWidth', 0.8);
% plot(centroid_x2, centroid_y2, 'k--', 'LineWidth', 0.8);
% plot(centroid_x3, centroid_y3, 'k--', 'LineWidth', 0.8);
hold off;
axis equal;
exportgraphics(gcf, 'Plot/voronoi_coverage.pdf')

% OUTPUTS
c1 = [centroid_x1, centroid_y1];
c2 = [centroid_x2, centroid_y2];
c3 = [centroid_x3, centroid_y3];

a1 = points_area1;
a2 = points_area2;
a3 = points_area3;

b1 = boundary1;
b2 = boundary2;
b3 = boundary3;

v1 = vel_lloyd1;
v2 = vel_lloyd2;
v3 = vel_lloyd3;

end