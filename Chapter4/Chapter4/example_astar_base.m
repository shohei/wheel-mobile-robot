map = zeros(14, 14); % Map
map(5:10,[4 11]) = 1; map(5,4:11) = 1; % Obstacles

astar = AStarBase();
astar.map = map;
path = astar.find([11; 6], [4; 10])