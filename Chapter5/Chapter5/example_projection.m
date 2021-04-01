% Intrinsic camera parameters and image screen size
alphaF = 1000; % alpha*f, in px/m
s = [1024; 768]; % screen size, in px
c = s/2; % image centre, in px
S = [alphaF, 0, c(1); 0, alphaF, c(2); 0, 0, 1]; % Camera model

% Set of 3-D points in camera frame
pC = [-1 1 4; 1 1 5; 0 -1 4; -1 1 -4; 4 1 5].';

% Projection of points to image frame
pP = (S*pC)./repmat(pC(3,:), 3, 1)

hFig = OFig(2,2,2);
bb = [0, s(1), 0, s(2)]; hFig.axes(1); axis(bb); hFig.axes(2); axis(bb);
hFig.image0();

hFig.axes(1);
for i = 1:size(pP, 2);
    if pP(1,i)>=0 && pP(1,i)<=s(1) && pP(2,i)>=0 && pP(2,i)<=s(2)
        line(pP(1,i), pP(2,i), 1, 'Color', 'b', 'Marker', '.');
        text(pP(1,i)+0.03*s(1), pP(2,i), 2, sprintf('$p_{P,%d}$', i), 'HorizontalAlignment', 'left');
    end
end

hFig.axes(2);
for i = 1:size(pP, 2);
    if pP(1,i)>=0 && pP(1,i)<=s(1) && pP(2,i)>=0 && pP(2,i)<=s(2) && pC(3,i)>0
        line(pP(1,i), pP(2,i), 1, 'Color', 'b', 'Marker', '.');
        text(pP(1,i)+0.03*s(1), pP(2,i), 2, sprintf('$p_{P,%d}$', i), 'HorizontalAlignment', 'left');
    end
end