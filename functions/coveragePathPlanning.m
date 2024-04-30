function path = coveragePathPlanning(a, d, o)

% a = area to be covered
% d = camera FoV
% o = orientation (0 = verical, 1 = horizontal)

% Consider just the intested portion of the total area
area = zeros(1000, 1000);
for i = 1:size(a, 1)
    y = a(i, 1);
    x = a(i, 2);
    area(x, y) = 1;
end

% Pick interested points
index_x = [];
index_y = [];
path_x = [];
path_y = [];
for i = 20:d:size(area, 1)
    for j = 20:d:size(area, 2)
        index_x = [index_x, i];
        index_y = [index_y, j];
    end
end

% Select points if their are inside interested area
for j = 1:length(index_y)
    for i = 1:length(index_x)
        if(area(index_x(i), index_y(j))) == 1
            path_x = [path_x; index_x(i)];
            path_y = [path_y; index_y(j)];
        end
    end
end

% Change orientation if necessary
if o == 0
    x = path_x;
    y = path_y;
else
    y = path_x;
    x = path_y;
end

% Create subvectors to change orientation 
x_val_unique = unique(x);
y_subvectors = cell(size(x_val_unique));

for i = 1:length(x_val_unique)

    indices = find(x == x_val_unique(i));
    
    y_matching = y(indices);
    y_sorted = sort(y_matching);
    
    x_costant = x_val_unique(i) * ones(size(y_sorted));
    
    y_subvectors{i} = y_sorted;
    x_subvectors{i} = x_costant;
end

% Fill vectors with final path to be followed
vec_x = [];
vec_y = [];

dir = 0; % Set direction

for i = 1:length(x_subvectors)
    vec_x = [vec_x; x_subvectors{i}];

    if dir == 0
        temp = y_subvectors{i};
        dir = 1;
    else
        temp = flip(y_subvectors{i});
        dir = 0;
    end

    vec_y = [vec_y; temp];
end

% Adapt path to orientation
if o == 0
    path = unique([vec_y, vec_x], "rows", "stable");
else
    path = unique([vec_x, vec_y], "rows", "stable");
end

end