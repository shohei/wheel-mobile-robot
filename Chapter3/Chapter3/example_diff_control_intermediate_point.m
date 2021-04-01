Ts = 0.03; % Sampling time
t = 0:Ts:15; % Simulation time
r = 0.5; % Distance parameter for the intermediate point
dTol = 0.05; % Tolerance distance (to the intermediate point) for switch
qRef = [4; 4; 0]; % Reference pose
q = [1; 0; 100/180*pi]; % Initial pose

% Intermediate point
xT = qRef(1) - r*cos(qRef(3));
yT = qRef(2) - r*sin(qRef(3));

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
int = OMarker(); int.showMarker('r-', dTol); int.setPose([xT; yT]); int.setTxt('Intermediate point', 0.1, pi*3/4, 'rb');
ref = OWmr(); ref.showWmr('g:'); ref.setPose(qRef); ref.setTxt('Goal', 0.5, pi/4, 'lb');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

state = 0; % State: 0 - go to intermediate point, 1 - go to reference point
for k = 1:length(t)
    D = sqrt((qRef(1)-q(1))^2 + (qRef(2)-q(2))^2);
    if D<dTol % Stop when close to the goal
        v = 0;
        w = 0;
    else
        if state==0
            d = sqrt((xT-q(1))^2+(yT-q(2))^2);
            if d<dTol, state = 1; end

            phiT = atan2(yT-q(2), xT-q(1));
            ePhi = phiT - q(3);
        else
            ePhi = qRef(3) - q(3);
        end
        ePhi = wrapToPi(ePhi);
        
        % Controller
        v = D*0.8;
        w = ePhi*5;
    end
    
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