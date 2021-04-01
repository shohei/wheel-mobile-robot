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

points = cell2mat(obstacles(:));
if ~exist('delaunayTriangulation', 'class')
    delaunayTriangulation = @DelaunayTri;
    disp('Using "DelaunayTri" instead of "delaunayTriangulation".');
end
dt = delaunayTriangulation(points);
triplot(dt, 'b-'); axis equal tight;

bb = bb + [-1 1 -1 1]*0.01*(bb(2)-bb(1));
hFig.sketch(); hFig.axes(); axis(bb); box off;