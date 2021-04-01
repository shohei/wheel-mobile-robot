m = [0, 0; 5, 3; 1, 5].'; % Three markers
r0 = [2; 2.5; pi/6]; % True robot pose (x, y and fi), unknown.
hFig = OFig(); axis equal; axis([-2, 6, -0.5, 5.5]);
xlabel('$x$'); ylabel('$y$');

wmrs = cell(1,size(m,2));
sty = {'m-', 'c-', 'g-'};
for i = 1:size(m,2)
    wmrs{i} = OObject(); wmrs{i}.setPose(r0);
    wmrs{i}.showAngle(sty{i}, 1); wmrs{i}.showDistance(sty{i});
end
for i = 1:size(m,2)
    mi = OMarker(); mi.showMarker(sty{i});
    mi.setTex(sprintf('$M%d$', i)); mi.setPose(m(:,i));
    mi.updateRange(wmrs{i}); wmrs{i}.updateRange(mi);
end
wmr = OWmr(); wmr.showWmr('y-'); wmr.setPose(r0); wmr.showParticles('bo-');
wmp = OWmr(); wmp.showParticles('r.-');

% Measured angles
alpha = wrapToPi(atan2(m(2,:)-r0(2), m(1,:)-r0(1))-r0(3));

% Triangulation: compute robot pose from measured angles
% Method using geometry approach
%
% For algorithm details see the article: J.M. FOnt-Liagunes, J.A. Vettle,
% Consistent triangulation for mobile robot localization using
% discontinuous angular measurments. 2009.
f = atan2(m(2,3)-m(2,2), m(1,3)-m(1,2));
S = [cos(f) -sin(f); sin(f) cos(f)]; % Rotation for corrdinate frame in m2
m_ = S.'*(m - repmat(m(:,2),1,3)); % Transformed markers

cta = cot(alpha(2)-alpha(1));
ctb = cot(alpha(3)-alpha(2));
ni = (m_(1,3)-m_(1,1)-m_(2,1)*cta)/(m_(1,3)*ctb-m_(2,1)+m_(1,1)*cta);
p_ = m_(1,3)*(1-ni*ctb)/(1+ni^2)*[1; -ni];

% Solution
p = m(:,2) + S*p_ % Position
fi = wrapToPi(atan2(m(2,1)-p(2), m(1,1)-r0(1))-alpha(1)) % Orientation

wmr.showWmr('y-');
wmr.setPose([p; fi]);