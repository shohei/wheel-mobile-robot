r = 0.04; % Wheel radius
L = 0.08; % Axle length
Ts = 0.03; % Sampling time
t = 0:Ts:10; % Simulation time
q = [4; 0.5; pi/6]; % Initial pose

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Initial pose', 0.5, -pi/2, 'lt'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

for k = 1:length(t)
    wL = 12; % Left wheel velocity
    wR = 12.5; % Right wheel velocity
    v = r/2*(wR+wL); % Robot velocity
    w = r/L*(wR-wL); % Robot angular velocity
    dq = [v*cos(q(3)+Ts*w/2); v*sin(q(3)+Ts*w/2); w];
    q = q + Ts*dq; % Integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]    
    
    wmr.setPose(q);
    sigU.plot(t(k), [v, w]);
    OFig.pause(Ts);
end

sigU.fin();
