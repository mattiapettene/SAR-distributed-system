function [tree, path] = rrt_star(map, start, goal, maxIterations, stepSize, radius, goalbias, bias, bias_radius, plt, slope)

% map -> logic matrix where obstacles are set to 1
% start -> starting point [x,y]
% goal -> arrival point [x,y]
% maxIterations -> number of iterations
% stepSize -> extension length
% radius -> nearby nodes radius
% goalbias -> activate/deactivate goal bias 0/1
% bias -> percentage of points generated in goal direction 0...1
% bias radius -> radius around the goal where random points are generated 


% -------------------------- INITIALIZATION ------------------------------

% Define structure tree
X_INDEX = 1;
Y_INDEX = 2;
COST_INDEX = 3;
PARENT_INDEX = 4;

% tree = [X_INDEX, Y_INDEX, COST_INDEX, PARENT_INDEX];
tree = [start, 0, NaN];      % Tree initialization

% ------------------------------ALGORITHM --------------------------------

for iter=1:maxIterations

    % Random point generation
    if goalbias == 1
        randomPoint = goalBias(map, goal, bias, bias_radius);
    else
        randomPoint = [randi(size(map, 2)), randi(size(map, 1))];
    end

    % Find nearest node
    nearestNode = findNearestNode(tree, randomPoint);

    % Extension in random point direction
    extendedPoint = extend(randomPoint, nearestNode, stepSize);
    
    % If obstacle it is not intersected, add extendedPoint to tree
    if ~ isObstacle(map, nearestNode(1:2), extendedPoint(1:2)) && isInside(extendedPoint, map)
            
        % Find nearby indices
        nearbyIndices = findNearbyNodes(tree, extendedPoint, radius, map);

        % Choose best parent
        [cost, parent] = chooseBestParent(tree, extendedPoint, nearbyIndices);
        extendedPoint(COST_INDEX) = cost;
        extendedPoint(PARENT_INDEX) = parent;

        % Add node to tree
        tree = [tree; extendedPoint];

        % Rewiring
        rewire(tree, nearbyIndices, extendedPoint);

        % Check if extendedPoint is near to the goal
        if (vecnorm(goal-extendedPoint(1:2), 2, 2) <= stepSize)
            [goalPoint, nearbyGoalIndices] = addGoal(tree, goal, radius, map);
            tree = [tree; goalPoint];
            rewire(tree, nearbyGoalIndices, goalPoint);
        end
    end
end

% Create path
goal_indices = find(tree(:, 1)==goal(1) & tree(:, 2)==goal(2));

if isempty(goal_indices)
    fprintf('\nUnable to find a path with this number of iterations!\n');
    path = [];
else
    path = goal;
    min_path = min(tree(goal_indices, 3));
    best_index = goal_indices(find(tree(goal_indices, 3) == min_path));
    index = tree(best_index, 4);
    while index ~= 1
        path = [tree(index, 1:2); path];
        index = tree(index, 4);
    end
    path = [tree(1, 1:2); path];
    path = unique(path, "rows", "stable");
end


% -------------------------- FUNCTIONS -------------------------------

function nearestNode = findNearestNode(tree, point)
    distances = vecnorm(tree(:, 1:2)-point, 2, 2);
    [~, index] = min(distances);
    nearestNode = tree(index,:);
end

function extendedPoint = extend(randomPoint, nearestNode, stepSize)
    distance = vecnorm(randomPoint-nearestNode(1:2), 2, 2);
    if distance == 0
        extendedPoint = nearestNode;
    else
        x = nearestNode(1) - (nearestNode(1)-randomPoint(1))*(stepSize/distance);
        y = nearestNode(2) - (nearestNode(2)-randomPoint(2))*(stepSize/distance);
        extendedPoint = [round(x), round(y), NaN, NaN];
    end
end

function nearbyIndices = findNearbyNodes(tree, extendedPoint, radius, map)
    nearbyIndices = [];
    distances = vecnorm(tree(:,1:2)-extendedPoint(1:2), 2, 2);
    all_indices = find(distances <= radius);
    for i=1:length(all_indices)
        if ~isObstacle(map, tree(all_indices(i), 1:2), extendedPoint(1:2))
            nearbyIndices = [nearbyIndices; all_indices(i)];
        end
    end
end

function [cost, parent] = chooseBestParent(tree, extendedPoint, nearbyIndices)
    cost = [];
    for i=1:length(nearbyIndices)
        slp = 5*slope(tree(i,1), tree(i,2));
        cost = [cost; tree(i, 3) + vecnorm(tree(i,1:2)-extendedPoint(1:2), 2, 2) + slp];
    end
    [~, index] = min(cost);
    parent = nearbyIndices(index);
    cost = cost(index);
end

function rewire(tree, nearbyIndices, extendedPoint)
    for i=1:length(nearbyIndices)
        index = nearbyIndices(i);
        slp = 5*slope(tree(index,1), tree(index,2));
        old_cost = tree(index, 3);
        new_cost = extendedPoint(3) + vecnorm(tree(index,1:2)-extendedPoint(1:2), 2, 2) + slp;
        if new_cost < old_cost
            tree(index, 3) = new_cost;
            tree(index, 4) = size(tree, 1);     % extendedPoint index
        end
    end
end

function [goal_point, nearbyIndices] = addGoal(tree, goal, radius, map)
  
    goal_point = [goal, NaN, NaN];
    
    % Find nearby indices
    nearbyIndices = findNearbyNodes(tree, goal, radius, map);
    
    % Choose best parent
    [cost, parent] = chooseBestParent(tree, goal, nearbyIndices);
    goal_point(3) = cost;
    goal_point(4) = parent;
end

function random_point = goalBias(map, goal, bias, bias_radius)
    [map_height, map_width] = size(map);
    if rand() <= bias
        random_point = [min(max(randi([-bias_radius, bias_radius]) + goal(1), 1), map_width), ...
                        min(max(randi([-bias_radius, bias_radius]) + goal(2), 1), map_height)];
    else
        random_point = [randi([1, map_width]), randi([1, map_height])];
    end
end

    function is_inside = isInside(point, map)
    % Verifica se il punto è all'interno della matrice
    is_inside = point(1) >= 1 && point(1) <= size(map, 1) && point(2) >= 1 && point(2) <= size(map, 2);
    end


% ---------------------------- PLOT ---------------------------------
if plt
    figure('Name','RRT* path planning')
    imagesc(map);
    colormap([1 1 1; 0 0 0]);
    hold on;
    axis equal;
    
    for i=2:length(tree)
        plot([tree(i, 1), tree(tree(i, 4), 1)], [tree(i, 2), tree(tree(i, 4), 2)], 'b')
        plot(tree(i, 1) ,tree(i, 2), '.b', 'MarkerSize', 10);
    end
    if ~isempty(path)
        plot(path(:, 1), path(:, 2), '-r', 'LineWidth', 3)
    end
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor','g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor','r');
end
end