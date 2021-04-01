param_map;

% Plot obstacles and Delaunay triangles
hFig = OFig();
for i = 1:length(obstacles)
    if ispolycw(obstacles{i}(:,1), obstacles{i}(:,2))
        line(obstacles{i}([1:end,1],1), obstacles{i}([1:end,1],2), 'Color', 'c', 'LineWidth', 4);
    else
        patch(obstacles{i}(:,1), obstacles{i}(:,2), [0.7 1 1], 'EdgeColor', 'c', 'LineWidth', 4);
    end
end
axis equal tight; bb = axis();

% Break down the line segments into intermediate points
dMin = 50;
points = []; obst = [];
B = length(obstacles);
for i = 1:B
    ob = obstacles{i};
    M = size(ob, 1);
    for j = 1:M
        k = mod(j, M)+1; % j+1
        d = sqrt((ob(j,1)-ob(k,1))^2 + (ob(j,2)-ob(k,2))^2);
        n = ceil(d/dMin)+1;
        x = linspace(ob(j,1), ob(k,1), n).';
        y = linspace(ob(j,2), ob(k,2), n).';
        points = [points; x(1:end-1) y(1:end-1)];
    end
    obst = [obst; obstacles{i}([1:end,1],:); nan(1,2)];
end

% Calculate voronoi segments on a set of intermediate points
[vx, vy] = voronoi(points(:,1), points(:,2));

% Eliminate auxilary (not in the free space) voronoi segments
s = false(1, size(vx, 2));
for j = 1:size(vx, 2)
    in = inpolygon(vx(:,j), vy(:,j), obst(:,1), obst(:,2));
    s(j) = all(in==1);
end
ux = vx(:,s); uy = vy(:,s); % Approximated Voronoi segments

% Plot
plot([vx;nan(1,size(vx,2))], [vy;nan(1,size(vy,2))], 'b:'); hold on;
plot(points(:,1), points(:,2), 'b.');
plot([ux;nan(1,size(ux,2))], [uy;nan(1,size(uy,2))], 'g-', 'LineWidth', 1);
axis equal tight;

bb = bb + [-1 1 -1 1]*0.01*(bb(2)-bb(1));
hFig.sketch(); hFig.axes(); axis(bb); box off;