Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time

% Reference
freq = 2*pi/30;
  xRef = 1.1 + 0.7*sin(freq*t);    yRef = 0.9 + 0.7*sin(2*freq*t);
 dxRef = freq*0.7*cos(freq*t);    dyRef = 2*freq*0.7*cos(2*freq*t);
ddxRef =-freq^2*0.7*sin(freq*t); ddyRef =-4*freq^2*0.7*sin(2*freq*t);
qRef = [xRef; yRef; atan2(dyRef, dxRef)];
uRef = [ddxRef; ddyRef];

q = [xRef(1)+.05; yRef(1)-0.1; 0]; % Initial robot pose
z1 = [q(1); dxRef(1)]; % Initial state [x, x']
z2 = [q(2); dyRef(1)]; % Initial state [y, y']
v = sqrt(z1(2)^2+z2(2)^2); % Initial state of velocity integrator

% Matrices of linearized system
A = [0, 1; 0, 0]; B = [0; 1]; C = [1, 0];
% State feedback controller
desPoles = [-2-1i; -2+1i]; % Desired poles (of the controller)
K = acker(A, B, desPoles); % Control gains obtained by pole placement
% Observer
H = place(A.', C.', [-15, -16]).'; % Obsrver gains (by pole placement)

hAni = OFig(); axis equal; axis([-0.1, 2.3, 0, 1.8]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OWmr(); ref.showPath('r--', true); % ref.showWmr('g:', 0.2, 0.04, 0.009);
% wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:', 0.2, 0.04, 0.009);
wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); % wmr.showWmr('y-', 0.2, 0.04, 0.009);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');

for k = 1:length(t)
    % Reference states
    zRef1 = [xRef(k); dxRef(k)];
    zRef2 = [yRef(k); dyRef(k)];
    
    % Error and control
    ez1 = zRef1 - z1;
    ez2 = zRef2 - z2;
    uu = [ddxRef(k); ddyRef(k)] + [K*ez1; K*ez2];
    
    % Compute inputs to the robot
    F = [cos(q(3)), -v*sin(q(3)); ...
         sin(q(3)),  v*cos(q(3))];
    vv = F\uu; % Translational acceleration and angular velocity
    v = v + Ts*vv(1); % Integrate translational acceleration
    u = [v; vv(2)]; % Robot input
    
    wmr.setPose(q); ref.setPose(qRef(:,k));
    sigU.plot(t(k), u);
    OFig.pause(Ts);
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    if false % Estimate states with observer
        yz1 = q(1); % Measurement
        yz2 = q(2); % Measurement
        yz1m = C*z1; % Estimate from model
        yz2m = C*z2; % Estimate from model
        
        dz1 = A*z1 + B*u(1) + H*(yz1-yz1m); % Observer
        z1 = z1 + dz1*Ts; % Euler integration
        dz2 = A*z2 + B*u(2) + H*(yz2-yz2m); % Observer
        z2 = z2 + dz2*Ts; % Euler integration
    else % Take known (measured) orientation and velocity to compute states
        z1 = [q(1); u(1)*cos(q(3))];
        z2 = [q(2); u(1)*sin(q(3))];
    end
end

OFig.pause(Ts);
sigU.fin();