Ts = 0.03; % Sampling time
t = 0:Ts:30; % Simulation1 time
q = [0; 0; 0];% Initial pose
goal = [4; 4]; % Goal location
% Obstacles
obstacles{1} = flipud([-1 -1; 7 -1; 7 5; -1 5]);
obstacles{2} = [0.5 1; 4 1];
obstacles{3} = [3 3.5; 3 2.5; 5 2.5; 3 2.5];
obst = [];
for i = 1:length(obstacles)
    obst = [obst; obstacles{i}([1:end,1],:); nan(1,2)];
end

% Plot obstacles and Delaunay triangles
hFig = OFig();
for i = 1:length(obstacles)
    if ispolycw(obstacles{i}(:,1), obstacles{i}(:,2))
        line(obstacles{i}([1:end,1],1), obstacles{i}([1:end,1],2), 'Color', 'c', 'LineWidth', 4);
    else
        patch(obstacles{i}(:,1), obstacles{i}(:,2), [0.7 1 1], 'EdgeColor', 'c', 'LineWidth', 4);
    end
end
axis equal tight; bb = axis();

xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OMarker(); ref.setPose(goal); ref.setTxt('Goal', 0.2, 0, 'lb'); ref.showMarker('g:', 0.1);
wmr0 = OMarker(); wmr0.setPose(q); wmr0.setTxt('Start', 0.2, -pi/4, 'lt'); wmr0.showMarker('r:', 0.1);
wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); %wmr.showWmr('y-');

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

for k = 1:length(t)
    % Distance to the nearest obstacle and orientation of the segment
    [dObst, ~, z] = nearestSegment(q(1:2).', obst);
    phiObst = atan2(obst(z+1,2)-obst(z,2), obst(z+1,1)-obst(z,1));
    
    % Control algorithm goes here ...
    script_bug0;
    
    % Robot motion simulation
    dq = [v*cos(q(3)); v*sin(q(3)); w];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    wmr.setPose(q);
    sigU.plot(t(k), [v, w]);
    OFig.pause(Ts);
end

sigU.fin();