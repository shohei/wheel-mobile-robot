Ts = 0.03; % Sampling time
t = 0:Ts:30; % Simulation time
T = [3, 0; 6, 4; 3, 4; 3, 1; 0, 3].'; % Reference points of line segments
q = [5; 1; 0.6*pi]; % Initial pose

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
int = OObject(); int.showPath('r--', true); int.setPose(T(:,1)); int.setPose(T(:,2));
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4*3, 'rt'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

i = 1; % Index of the first point
for k = 1:length(t)
    % Reference segment determination
    dx = T(1,i+1) - T(1,i);
    dy = T(2,i+1) - T(2,i);

    v = [dx; dy]; % Direction vector of the current segment
    vN = [dy; -dx]; % Orthogonal direction vector of the current segment
    r = q(1:2) - T(:,i);
    u = v.'*r/(v.'*v);
    
    if u>1 && i<size(T,2)-1 % Switch to the next line segment
        i = i + 1;
        dx = T(1,i+1) - T(1,i);
        dy = T(2,i+1) - T(2,i);
        v = [dx; dy];
        vN = [dy; -dx];
        r = q(1:2) - T(:,i);
        
        int.setPose(T(:,i+1));
    end
    
    dn = vN.'*r/(vN.'*vN); % Normalized orthogonal distance
    
    phiLin = atan2(v(2), v(1)); % Orientation of the line segment
    phiRot = atan(5*dn); % If we are far from the line than we need
    % additional rotation to face towards the line. If we are on the left
    % side of the line we turn clockwise, otherwise counterclockwise.
    % Gain 5 increases the sensitivity ...
    
    phiRef = wrapToPi(phiLin + phiRot);

    % Orientation error for control
    ePhi = wrapToPi(phiRef - q(3));
    
    % Controller
    v = 0.4*cos(ePhi);
    w = 3*ePhi;
   
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