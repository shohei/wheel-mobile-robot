Ts = 0.03; % Sampling time
t = 0:Ts:30; % Simulation time
d = 0.1; % Distance between the front and the rear axis
q = [1.1; 0.8; 0]; % Initial robot pose

% Reference trajectory
freq = 2*pi/30;
xRef = 1.1 + 0.7*sin(freq*t);
yRef = 0.9 + 0.7*sin(2*freq*t);

hAni = OFig(); axis equal; axis([-0.1, 2.3, 0, 1.8]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OWmr(); ref.showPath('r--', true); % ref.showWmr('g:', 0.2, 0.04, 0.009);
% wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:', 0.2, 0.04, 0.009);
wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); % wmr.showWmr('y-', 0.2, 0.04, 0.009);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\alpha~[\mathrm{rad}]$'); sigU.sig('b-');

% Control gains
Kphi = 2;
Kv = 5;

upgradedControl = true; % This setting can be changed to false
for k = 1:length(t)    
    % Reference
    phiRef = atan2(yRef(k)-q(2), xRef(k)-q(1));
    qRef = [xRef(k); yRef(k); phiRef];
    
    % Error with respect to the (moving) reference
    e = qRef - q;          % Error on x, y and orientation
    e(3) = wrapToPi(e(3)); % Mapped to the [-pi, pi] interval

    % Control
    alpha = e(3)*Kphi;          % Orientation control (basic)
    v = sqrt(e(1)^2+e(2)^2)*Kv; % Forward-motion control (basic)
    if upgradedControl
        % If e(3) is not on the [-pi/2, pi/2], +/- pi should be added
        % to e(3), and negative velocity should be commanded
        v = v*sign(cos(e(3)));  % Changing sign of v if necessary
        e(3) = atan(tan(e(3))); % Mapped to the [-pi/2, pi/2] interval
        alpha = e(3)*Kphi;      % Orientation control (upgraded)
    end
    
    % Physical limitations of the vehicle
    if abs(alpha)>pi/4, alpha = pi/4*sign(alpha); end
    if abs(v)>0.8, v = 0.8*sign(v); end
    
    wmr.setPose(q); ref.setPose(qRef);
    sigU.plot(t(k), [v, alpha]);
    OFig.pause(Ts);

    % Robot motion simulation
    dq = [v*cos(q(3)); v*sin(q(3)); v/d*tan(alpha)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
end

OFig.pause(Ts);
sigU.fin();