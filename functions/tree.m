function [xT, yT, zT, xC, yC, zC] = tree(xPos, yPos, zPos)

%Tree dimensions
trunkWidth = 10;
treeHeight = 70;     
crownRadius = 25;

[xTrunk, yTrunk, zTrunk] = cylinder(trunkWidth/2, 100);
xTrunk = xTrunk + xPos;
yTrunk = yTrunk + yPos;
zTrunk = zTrunk*treeHeight + zPos;

[xCrown, yCrown, zCrown] = sphere(100);
xCrown = xCrown*crownRadius + xPos;
yCrown = yCrown*crownRadius + yPos;
zCrown = zCrown*crownRadius + treeHeight + zPos;

xT = xTrunk;
yT = yTrunk;
zT = zTrunk;

xC = xCrown;
yC = yCrown;
zC = zCrown;

end