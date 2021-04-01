script_laserscandata;
[x, y]; % Laser scan data
N = length(x);

dAlpha = pi/180; % Angle quant
nAlpha = round(2*pi/dAlpha);
nDist = nAlpha; % Use the same quantisation for distance
lutDist = zeros(N,nAlpha); % Distance look-up table
for i = 1:N % For each point compute lines for the range of alpha
    for j = 1:nAlpha
        alpha = (j-1)*dAlpha-pi;
        % Distance to the cordinate frame origin
        d = x(i)*cos(alpha)+y(i)*sin(alpha);
        if d<0
            if alpha>pi, alpha = alpha - pi;
            else         alpha = alpha + pi; end
            jj = round((alpha+pi)/dAlpha);
            lutDist(i,jj) = -d;
        else
            lutDist(i,j) = d;
        end
    end
end

% Find range for distance parameter
minLutDist = min(lutDist(:));
maxLutDist = max(lutDist(:));
dDist = (maxLutDist-minLutDist)/nDist; % Distance quant

% Realize accumulator of parameter space
A = zeros(nDist,nAlpha);
for i = 1:N % Go through all the points
    for j = 1:nAlpha
        k = round((lutDist(i,j)-minLutDist)/dDist)+1;
        if k>nDist, k = nDist; end
        A(k,j) = A(k,j)+1;
    end
end
H = A(2:nAlpha,:); % Accumulators

% Locate maximums in the accumulator (the most probable lines)
nLines = 7; % Requested number of top most probable lines
peaks  = houghpeaks(H, nLines, 'threshold', 3, 'NHoodSize', [31, 31]);
% Line parameters: distance from the origin and line angle
distAlpha = [peaks(:,1).'*dDist+minLutDist; peaks(:,2).'*dAlpha-pi];
% Line parameters of implicit form: ax+by+c=0
abc = [cos(distAlpha(2,:)); sin(distAlpha(2,:)); -distAlpha(1,:)];

% Draw acumulator
OFig(); xlabel('$\alpha$'); ylabel('$d$'); axis([0 nAlpha 0 nDist]);
image('CData', repmat(uint8(255-H/max(H(:))*255*10), [1, 1, 3]));
% Draw points and straight lines
roi = [floor(min(x)) ceil(max(x)) floor(min(y)) ceil(max(y))];
OFig(1.5,1,1); axis equal; axis(roi); xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
plot(x, y, 'k.');
for i = 1:size(abc,2)
    [lx, ly] = lineInRoi(abc(:,i), roi);
    line(lx, ly, 'Color', 'r', 'LineWidth', 1);
end