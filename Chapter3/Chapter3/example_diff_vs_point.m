Ts = 0.03; % Sampling time
t = 0:Ts:15; % Simulation1 time
r = 0.5; % Distance parameter for the intermediate point
dTol = 1; % Tolerance distance (to the intermediate point) for switch
qRef = [4; 4; 0]; % Reference pose
q = [1; 0; 100/180*pi]; % Initial pose

% Camera
alphaF = 300; % alpha*f, in px/m
s = [1024; 768]; % screen size, in px
c = s/2; % image centre, in px
S = [alphaF, 0, c(1); 0, alphaF, c(2); 0, 0, 1]; % Internal camera model
RL2C = rotX(pi/2)*rotY(-pi/2)*rotX(pi/4); tL2C=[0;0;0.5]; % Camera mounting
% Camera simulation
p0P = S*RL2C.'*([0; 0; 0]-tL2C); p0P = p0P/p0P(3);
RW2L = rotZ(-q(3)); tW2L = [q(1:2); 0];
pP = S*RL2C.'*(RW2L.'*([qRef(1:2); 0]-tW2L)-tL2C); pP = pP/pP(3);

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OMarker(); ref.showMarker('g:', 0.5); ref.setPose(qRef); ref.setTxt('Goal', 0.6, 0, 'lb');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

hImg = OFig(); axis([0 s(1) 0 s(2)]); hImg.image0();
xlabel('$x~[\mathrm{px}]$'); ylabel('$y~[\mathrm{px}]$');
refi = OMarker(); refi.showMarker('g:', 1.2*max([dTol, 10]));  refi.showPath('b-', true); refi.setPose(pP(1:2));
wmri = OMarker(); wmri.showMarker('y-', max([dTol, 10])); wmri.setPose(p0P(1:2));

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

u = [0; 0];
for k = 1:length(t)
    if pP(1)<0 || pP(2)<0 || pP(1)>s(1) || pP(2)>s(2) % Invisible feature
        u = [0; 0]; % Lost tracked feature
    else
        D = sqrt(sum((pP(1:2)-p0P(1:2)).^2));
        if D<dTol % Stop when close to the goal
            u = [0; 0];
        else
            u = [0, 0.002; 0.005, 0]*(p0P(1:2)-pP(1:2));
        end
    end
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    % Camera simulation
    RW2L = rotZ(-q(3)); tW2L = [q(1:2); 0];
    pP = S*RL2C.'*(RW2L.'*([qRef(1:2); 0]-tW2L)-tL2C); pP = pP/pP(3);
    
    wmr.setPose(q); refi.setPose(pP(1:2));
    sigU.plot(t(k), u);
    OFig.pause(Ts);
end

sigU.fin();