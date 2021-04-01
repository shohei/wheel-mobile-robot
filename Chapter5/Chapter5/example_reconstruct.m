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

% Reconstruct
M = size(pairs, 2);
pC1 = zeros(3,M);
for i = 1:M
    a = pairs(1,i); b = pairs(2,i);
    c1 = S\pP1(:,a);
    c2 = -R*(S\pP2(:,b));
    psi = [c1, c2]\t;
    pC1(:,i) = psi(1)*c1;
end
pC2 = R.'*(pC1-repmat(t, 1, M));
pC1, pC2