im = imread('colour_orange_mask.bmp')>128; % Ensure binary image

[x, y] = meshgrid(1:size(im,2), 1:size(im,1));

% Raw moments
m00 = sum(sum( (x.^0).*(y.^0).*double(im) ));
m10 = sum(sum( (x.^1).*(y.^0).*double(im) ));
m01 = sum(sum( (x.^0).*(y.^1).*double(im) ));
m11 = sum(sum( (x.^1).*(y.^1).*double(im) ));
m20 = sum(sum( (x.^2).*(y.^0).*double(im) ));
m02 = sum(sum( (x.^0).*(y.^2).*double(im) ));
% m21 = sum(sum( (x.^2).*(y.^1).*double(im) ));
% m12 = sum(sum( (x.^1).*(y.^2).*double(im) ));
% m30 = sum(sum( (x.^3).*(y.^0).*double(im) ));
% m03 = sum(sum( (x.^0).*(y.^3).*double(im) ));

% Area, x and y
area = m00
x0 = m10/m00
y0 = m01/m00

% Central moments
u00 = m00;
% u10 = 0
% u01 = 0
u11 = m11-x0*m01; % u11 = m11-y*m10;
u20 = m20-x0*m10;
u02 = m02-y0*m01;
% u21 = m21-2*x*m11-y*m20+2*x^2*m01
% u12 = m12-2*y*m11-x*m02+2*y^2*m10
% u30 = m30-3*x*m20+2*x^2*m10
% u03 = m03-3*y*m02+2*y^2*m01

% Ellipse
v = eig([u20, u11; u11, u02]); % Eigenvalues
a = 2*sqrt(v(2)/u00) % Semi-major axis
b = 2*sqrt(v(1)/u00) % Semi-minor axis
theta = atan2(2*u11, u20-u02)/2 % Orientation

om = imread('colours.jpg');
[h, w, ~] = size(im);
hFig = OFig(); axis([0 w 0 h]); hFig.image0();
om = 255-((255-om).*repmat(uint8(~im), [1 1 3]));
image('CData', om);
line([x0 x0], [0 h], 'Color', 'g', 'LineWidth', 1, 'LineStyle', '--');
line([0 w], [y0 y0], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--');
line([x0 x0+a*cos(theta)], [y0 y0+a*sin(theta)], 'Color', 'r', 'LineWidth', 1);
line([x0 x0-b*sin(theta)], [y0 y0+b*cos(theta)], 'Color', 'g', 'LineWidth', 1);
t = linspace(0, 2*pi, 100);
ellipse = [cos(theta),-sin(theta); ...
           sin(theta), cos(theta)]*[a*cos(t); b*sin(t)];
line(x0+ellipse(1,:), y0+ellipse(2,:), 'Color', 'c', 'LineWidth', 3);
text(x0+a/2*cos(theta), y0+a/2*sin(theta), '$a$', 'Color', 'r', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
text(x0-b/2*sin(theta), y0+b/2*cos(theta), '$b$', 'Color', 'g', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
set(gca, 'XTick', [0, x0, w], 'XTickLabel', {0, '$\frac{m_{1,0}}{m_{0,0}}$', w}, ...
         'YTick', [0, y0, h], 'YTickLabel', {0, '$\frac{m_{0,1}}{m_{0,0}}$', h});
%Draw angle
f = linspace(0, theta, max([2, ceil(theta/pi*100)]));
patch(x0+[0, 0.5*a*cos(f), 0], y0+[0, 0.5*a*sin(f), 0], 'b', 'LineStyle', 'None', 'FaceAlpha', 0.1);
line(x0+0.5*a*cos(f), y0+0.5*a*sin(f), 'Color', 'b', 'LineStyle', '-');
text(x0+(0.5*a+10)*cos(theta/2), y0+(0.5*a+10)*sin(theta/2), '$\theta$', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Color', 'b');