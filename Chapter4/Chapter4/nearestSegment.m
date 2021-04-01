function [d, c, v] = nearestSegment(p, vertices)
% Given the point $p$ and an array of vertices that define a closed polygon
% made of linear segments, it calculates the distance $d$ to the nearest
% point $c$ on the nearest segment $v$.

N = size(vertices, 1);
d = inf;
c = [];
v = 0;
for i = 1:N
    s = vertices([i,mod(i,N)+1],:).';
    if ~any(isnan(s(:)))
        [di, ci] = segmentDistance(p.', s(:,1), s(:,2));
        if di<d
            d = di;
            c = ci;
            v = i;
        end
    end
end