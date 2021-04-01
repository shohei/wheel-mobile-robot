function [d, c] = nearestObstacle(p, obstacles)
% Given the array of points $p = [px, py]$ and a list of closed polygons
% (obstacles) made of linear segments, it calculates the distances $d$ to
% the nearest points $c = [cx, cy]$ on the nearest obstacle for each of the
% points. Based on the order of the points polygon inside is distinguished
% from the polygon outside.

obst = [];
for i = 1:length(obstacles)
    obst = [obst; obstacles{i}([1:end,1],:); nan(1,2)];
end

N = size(p, 1);
d = zeros(N,1);
c = zeros(N,2);
for i = 1:N
    [dd, cc] = nearestSegment(p(i,:), obst);
    d(i) = dd;
    c(i,:) = cc;
end

d = d.*(-1+2*inpolygon(p(:,1), p(:,2), obst(:,1), obst(:,2)));