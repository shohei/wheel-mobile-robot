% Intrinsic camera parameters and image screen size
alphaF = 1000; % alpha*f, in px/m
c = [512; 384]; % image centre, in px
S = [alphaF, 0, c(1); 0, alphaF, c(2); 0, 0, 1]; % Camera model

b = 0.2; % Stereo camera baseline, in m

% Matched points
pP1 = [351; 522];
pP2 = [236; 522];

% 3-D reconstruction
d = pP1(1)-pP2(1);
pC1 = b/d*[pP1-c; alphaF]