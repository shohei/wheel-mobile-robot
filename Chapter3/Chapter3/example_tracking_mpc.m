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

for k = 1:length(t)-4
    e = [cos(q(3)), sin(q(3)), 0; ...
        -sin(q(3)), cos(q(3)), 0; ...
         0,         0,         1]*(qRef(:,k) - q); % Error vector
    e(3) = wrapToPi(e(3)); % Correct angle
    
    A0 = [1, Ts*uRef(2,k),   0;-Ts*uRef(2,k),   1, Ts*uRef(1,k);   0,0,1];
    A1 = [1, Ts*uRef(2,k+1), 0;-Ts*uRef(2,k+1), 1, Ts*uRef(1,k+1); 0,0,1];
    A2 = [1, Ts*uRef(2,k+2), 0;-Ts*uRef(2,k+2), 1, Ts*uRef(1,k+2); 0,0,1];
    A3 = [1, Ts*uRef(2,k+3), 0;-Ts*uRef(2,k+3), 1, Ts*uRef(1,k+3); 0,0,1];
    A4 = [1, Ts*uRef(2,k+4), 0;-Ts*uRef(2,k+4), 1, Ts*uRef(1,k+4); 0,0,1];
    B = [Ts, 0; 0, 0; 0, Ts];
    C = eye(3);
    
    Z = zeros(3,2);
    Hm = [C*A0*B,          Z,            Z,         Z; ...
          C*A0*A1*B,       C*A0*B,       Z,         Z; ...
          C*A0*A1*A2*B,    C*A0*A1*B,    C*A0*B,    Z; ...
          C*A0*A1*A2*A3*B, C*A0*A1*A2*B, C*A0*A1*B, C*A0*B];
    Fm = [C*A0*A1, C*A0*A1*A2, C*A0*A1*A2*A3, C*A0*A1*A2*A3*A4].';
    
    ar = 0.65;
    Ar = eye(3)*ar; % Reference error dynamics
    H = 0;
    Fr = [Ar^(H+1), Ar^(H+2), Ar^(H+3), Ar^(H+4)].';
    
    % Weight matrices
    Qt = diag(repmat([1; 40; 0.1], 4, 1));
    Rt = diag(repmat([0.001; 0.001], 4, 1));
    
    % Optimal control calculation
    KKgpc = (Hm.'*Qt*Hm + Rt)\(Hm.'*Qt*(-Fm));
    KK = KKgpc(1:2,:); % Take current control gains
  
    v = -KK*e;
    uF = [uRef(1,k)*cos(e(3)); uRef(2,k)];
    u = v + uF;
   
    vMAX = 1; wMAX = 15; % Max velocities
    if abs(u(1))>vMAX, u(1) = sign(u(1))*vMAX; end
    if abs(u(2))>wMAX, u(2) = sign(u(2))*wMAX; end
    
    wmr.setPose(q); ref.setPose(qRef(:,k));
    sigU.plot(t(k), uF, u);
    OFig.pause(Ts);
  
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
end

OFig.pause(Ts);
sigU.fin();