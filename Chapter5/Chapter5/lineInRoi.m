function [x, y] = lineInRoi(l, roi)
% Find the segment of line in region of interest (ROI)
%
%in l   Line in the form $l=[a b c]$, $a x + b y + c = 0$.
%in roi Region of interest in the form ¨`roi = {xmin xmax ymin ymax}`.
%
%out x Horizontal coordinates of end segment points.
%out y Vertical coordinates of end segment points.

if length(roi)==2
    roi = [0, roi(1), 0, roi(2)];
end

a = l(1); b = l(2); c = l(3); % Line parameters
w = roi(1); W = roi(2); h = roi(3); H = roi(4); % Bounding box

% Constraints
C = [-(c+h*b+w*a)/a, (c+h*b+W*a)/a; ... % Bottom
     -(c+h*b+w*a)/b, (c+H*b+w*a)/b; ... % Left
     -(c+H*b+w*a)/a, (c+H*b+W*a)/a; ... % Top
     -(c+h*b+W*a)/b, (c+H*b+W*a)/b];    % Right
% Points
p = [-(c+h*b)/a, h; ... % Bottom
     w, -(c+w*a)/b; ... % Left
     -(c+H*b)/a, H; ... % Top
     W, -(c+W*a)/b].';  % Right
x = []; y = [];
B = all(C>=0, 2);
j = [1 3; 2 4; 1 2; 1 4; 2 3; 3 4]; % Order is important!
for i = 1:size(j, 1)
    if B(j(i,:))
        x = p(1,j(i,:)); y = p(2,j(i,:));
        break;
    end
end