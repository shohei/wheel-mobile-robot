Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time
q = [1.1; 0.8; 0]; % Initial robot pose

% Reference
freq = 2*pi/30;
  xRef = 1.1 + 0.7*sin(freq*t);    yRef = 0.9 + 0.7*sin(2*freq*t);
 dxRef = freq*0.7*cos(freq*t);    dyRef = 2*freq*0.7*cos(2*freq*t);
ddxRef =-freq^2*0.7*sin(freq*t); ddyRef =-4*freq^2*0.7*sin(2*freq*t);
qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
uRef = [vRef; wRef]; % Reference inputs

hAni = OFig(); axis equal; axis([-0.1, 2.3, 0, 1.8]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OWmr(); ref.showPath('r--', true); % ref.showWmr('g:', 0.2, 0.04, 0.009);
% wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:', 0.2, 0.04, 0.009);
wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); % wmr.showWmr('y-', 0.2, 0.04, 0.009);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('r--', 'b-');

for k = 1:length(t)
    e = [cos(q(3)), sin(q(3)), 0; ...
        -sin(q(3)), cos(q(3)), 0; ...
         0,         0,         1]*(qRef(:,k) - q); % Error vector
    e(3) = wrapToPi(e(3)); % Correct angle
    
    % Current reference inputs
    vRef = uRef(1,k);
    wRef = uRef(2,k);
  
    % Control
    eX = e(1); eY = e(2); ePhi = e(3);
    zeta = 0.9; % Experiment with this control design parameter
    g = 85;     % Experiment with this control design parameter
    Kx = 2*zeta*sqrt(wRef^2+g*vRef^2);
    Kphi = Kx;
    Ky = g;
    % Gains Kx and Kphi could also be constant.
    % This form is used to have the same damping of the transient
    % irrespective of ref. velocities.

    % Control: feedforward and feedback
    v = vRef*cos(e(3)) + Kx*eX;
    w = wRef + Ky*vRef*(cos(ePhi/2))^4*eY + Kphi*sin(ePhi);
    
    wmr.setPose(q); ref.setPose(qRef(:,k));
    sigU.plot(t(k), [vRef, wRef], [v, w]);
    OFig.pause(Ts);
    
    % Robot motion simulation
    dq = [v*cos(q(3)); v*sin(q(3)); w];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
end

OFig.pause(Ts);
sigU.fin();