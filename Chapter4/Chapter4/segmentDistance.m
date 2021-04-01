function [d, c] = segmentDistance(p, q, r)
% Given the point $p$ and a segment defined by the end points $q$ and $r$,
% it calculates the distance $d$ to the nearest point $c$ on the segment.

a = r-q;
b = p-q;
e = ((a.'*b)/(a.'*a));
if e<0
    c = q;
elseif e>1
    c = r;
else
    c = q + a.*e;
end
v = p-c;
d = norm(v);