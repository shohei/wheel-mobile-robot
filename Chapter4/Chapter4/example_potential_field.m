param_map;

goal = [1350 140];

% Mesh
G = 25;
x = min(o(:,1)):G:max(o(:,1));
y = min(o(:,2)):G:max(o(:,2));
[X, Y] = meshgrid(x, y);

% Attractive field
Kp = 0.0002;
p = Kp*sum((repmat(goal, length(X(:)), 1)-[X(:), Y(:)]).^2, 2);
p = reshape(p, size(X,1), []);

% Repelling field
Kd = 150;
D = 50;
d = nearestObstacle([X(:), Y(:)], obstacles);
d = reshape(d, size(X,1), []);
d(d<0) = D;
i = d<D;
d(d<D) = 1/D*(D-d(i)).^2;
d(d>D) = 0;
d = Kd*d/D;

% Potential field
P = p + d;

hFig1 = OFig();
surf(X, Y, P, 'FaceAlpha', 0.9);
hFig1.sketch(); % xlabel('$x$'); ylabel('$y$'); zlabel('Potential field');
view(50, 30); axis([min(x), max(x), min(y), max(y)]); axis equal;

R = max(max(P));
paths = {'pathOK', 'pathTrap'};
for i = 1:length(paths)
    hFigi = OFig(2, 2, 1);
    contour(X, Y, P, 10);
    data = load(paths{i});
    plot(data.Q(:,1), data.Q(:,2), 'm', 'LineWidth', 2);
    plot3(data.Q(1,1), data.Q(1,2), R, 'ro');
    text(data.Q(1,1)-50, data.Q(1,2)-100, R+1, 'Start');
    plot3(goal(1), goal(2), R, 'g*');
    text(goal(1)-50, goal(2)+100, R+1, 'Goal');
    hFigi.sketch();
end