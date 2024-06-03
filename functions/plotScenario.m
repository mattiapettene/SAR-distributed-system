function  plotScenario(H, xyzObstacles, nForestTree, nObstaclesrand)

mesh(H);
colormap('summer');
contour3(H, 80, 'k', 'LineWidth', 1);
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

end