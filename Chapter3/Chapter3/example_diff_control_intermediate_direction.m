Ts = 0.03; % Sampling time
t = 0:Ts:15; % Simulation time
r = 0.2; % Distance parameter
qRef = [4; 4; 0]; % Reference pose
q = [1; 0; 100/180*pi]; % Initial pose

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
int = OMarker(); int.showMarker('r-', 0.05); int.setTxt('Guiding point', 0.1, pi*3/4, 'rb');
ref = OWmr(); ref.showWmr('g:'); ref.setPose(qRef); ref.setTxt('Goal', 0.5, pi/4, 'lb');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

for k = 1:length(t)
    % Compute intermidieate direction shift
    phiR = atan2(qRef(2)-q(2), qRef(1)-q(1));
    D = sqrt((qRef(1)-q(1))^2 + (qRef(2)-q(2))^2);
    
    alpha = wrapToPi(phiR - qRef(3));
    beta = atan(r/D);
    if alpha<0, beta = -beta; end
    
    % Controller
    if abs(alpha) < abs(beta)
        ePhi = wrapToPi(phiR - q(3) + alpha); % The second part
    else
        ePhi = wrapToPi(phiR - q(3) + beta); % The first part
        int.setPose(q);
    end
    v = D*0.8;
    w = ePhi*5;
    
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