m = [0, 0; 5, 3; 1, 5; 2, 4].'; % Markers

% Measured distances to the markers
d = [3.2297, 3.0697, 2.7060, 1.4759];

% Trilateration: find the robot position from measured distances
N = size(m,2); A = zeros(N-1,2); b = zeros(N-1,1);
for i = 1:N-1
    A(i,:) = 2*[m(1,N)-m(1,i), m(2,N)-m(2,i)];
    b(i) = d(i)^2-d(N)^2-m(1,i)^2 + m(1,N)^2-m(2,i)^2 + m(2,N)^2;
end

r = A\b % Calculated position

hFig = OFig(); axis equal; axis([-2, 6, -0.5, 5.5]);
xlabel('$x$'); ylabel('$y$');

wmr = OWmr(); wmr.showWmr('y-'); wmr.setPose([r; 0]);
sty = {'m', 'c', 'g', 'b'};
for i = 1:size(m,2)
    mi = OMarker(); mi.showMarker(sprintf('%s-', sty{i}));
    mi.showDistance(sprintf('%s-', sty{i}));
    mi.showRange(sprintf('%s:', sty{i}));
    mi.setTex(sprintf('$M%d$', i)); mi.setPose(m(:,i));
    ai = atan2(r(2)-m(2,i), r(1)-m(1,i));
    wmri = OObject(); wmri.setPose(m(:,i)+d(i)*[cos(ai); sin(ai)]);
    mi.updateRange(wmri);
end
