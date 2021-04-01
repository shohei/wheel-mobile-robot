% Intrinsic camera parameters and image screen size
alphaF = 1000; % alpha*f, in px/m
s = [1024; 768]; % screen size, in px
c = s/2; % image centre, in px
S = [alphaF, 0, c(1); 0, alphaF, c(2); 0, 0, 1]; % Camera model

% Relative pose between the cameras
R = rotX(pi/6)*rotY(pi/3); t = [4; -1; 2];

% Set of points
pP1 = [262, 634; 762, 634; 512, 134; 443, 457; 412, 284].';
pP2 = [259, 409; 397, 153; 488, 513; 730, 569; 115, 214].';

N1 = size(pP1, 2); N2 = size(pP2, 2); % Number of points

% Fundamental matrix
tx = [0, -t(3), t(2); t(3), 0, -t(1); -t(2), t(1), 0];
F = S.'\tx*R/S;

epsilon = 1e-2; % Distance error tolerance
pP1 = [pP1; ones(1,N1)]; pP2 = [pP2; ones(1,N2)]; % Homogenous points

% Epipolar lines in frame P1 associated with the points in frame P2
lP1 = F*pP2;
% Epipolar lines in frame P2 associated with the points in frame P1
lP2 = F.'*pP1;

% Find point pairs (evaluate fundamental contraint in frame P2)
pairs = [];
for i = 1:N1
    d = abs(lP2(:,i).'*pP2);
    k = find(d<epsilon);
    if ~isempty(k), pairs = [pairs, [i; k(1)]]; end
end
pairs

hFig1 = OFig(2,2,2);
bb = [0, s(1), 0, s(2)]; hFig1.axes(1); axis(bb); hFig1.axes(2); axis(bb);
hFig1.image0();
hFig1.axes(1);
for i = 1:N1
    line(pP1(1,i), pP1(2,i), 1, 'Color', 'b', 'Marker', '.');
    text(pP1(1,i)+0.03*s(1), pP1(2,i), 2, sprintf('$p_{P_1,%d}$', i), 'HorizontalAlignment', 'left');
end
hFig1.axes(2);
for i = 1:N2
    line(pP2(1,i), pP2(2,i), 1, 'Color', 'r', 'Marker', '.');
    text(pP2(1,i)+0.03*s(1), pP2(2,i), 2, sprintf('$p_{P_2,%d}$', i), 'HorizontalAlignment', 'left');
end

hFig2 = OFig(2,2,2);
bb = [0, s(1), 0, s(2)]; hFig2.axes(1); axis(bb); hFig2.axes(2); axis(bb);
hFig2.image0();
for i = 1:N1
    hFig2.axes(2);
    [x, y] = lineInRoi(lP2(:,i), s);
    line(x, y, zeros(size(x)), 'Color', 'b', 'LineStyle', '-');
    x = i/(N1+1)*abs(diff(x))+min(x); y = -(lP2(1,i)*x+lP2(3,i))/lP2(2,i);
    text(x+0.03*s(1), y, 2*ones(size(x)), sprintf('$l_{P_2,%d}$', i), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    hFig2.axes(1);
    line(pP1(1,i), pP1(2,i), 1, 'Color', 'b', 'Marker', '.');
end
for i = 1:N2
    hFig2.axes(1);
    [x, y] = lineInRoi(lP1(:,i), s);
    line(x, y, zeros(size(x)), 'Color', 'r', 'LineStyle', '-');
    x = i/(N2+1)*abs(diff(x))+min(x); y = -(lP1(1,i)*x+lP1(3,i))/lP1(2,i);
    text(x+0.03*s(1), y, 2*ones(size(x)), sprintf('$l_{P_1,%d}$', i), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    hFig2.axes(2);
    line(pP2(1,i), pP2(2,i), 1, 'Color', 'r', 'Marker', '.');
end