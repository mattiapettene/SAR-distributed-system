function check = isObstacle(matrix, point1, point2)
    % Check if the line that connects two points intersects an obstacle during the RRT* planning

    % Input:
    % - matrix: logic map matrix (obstacles point set to 1)
    % - point1: starting point coordinates [x1, y1]
    % - point2: arrival point coordinates [x2, y2]

    % Output:
    % - check: 0 if not, 1 if an obstacle is intersected
    
    % Extact points coordinates
    x1 = point1(1);
    y1 = point1(2);
    x2 = point2(1);
    y2 = point2(2);
    
    % Return initialization
    check = false;
    
    % Compute number of points on the line and their indices
    numPoints = (max(abs(x2 - x1), abs(y2 - y1)) + 1)*2;
    xIndices = round(linspace(x1, x2, numPoints));
    yIndices = round(linspace(y1, y2, numPoints));
    
    % Check each point of the line
    for i = 1:numPoints
        x = xIndices(i);
        y = yIndices(i);
        
        % Check if the point is an obstacle in the map logic matrix
        if x >= 1 && x <= size(matrix, 2) && y >= 1 && y <= size(matrix, 1)
            if matrix(y, x) == 1
                check = true;
                return;
            end
        end
    end
end