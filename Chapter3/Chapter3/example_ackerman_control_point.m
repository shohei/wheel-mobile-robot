Ts = 0.03; % Sampling time
t = 0:Ts:30; % Simulation time
d = 0.1; % Distance between axes
xyRef = [4; 4]; % Reference position
q = [1; 0; -pi]; % Initial state

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OMarker(); ref.showMarker('g:', 0.5); ref.setPose(xyRef); ref.setTxt('Goal', 0.6, 0, 'lb');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, -pi/4, 'rb'); wmr0.showWmr('r:'); 
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\alpha~[\mathrm{rad}]$'); sigU.sig('b-');
OFig.pause(Ts);

for k = 1:length(t)
    phi_ref = atan2(xyRef(2)-q(2), xyRef(1)-q(1)); %Reference orientation
    qRef = [xyRef; phi_ref];
    
    e = qRef - q; %Position end orientation error
    
    % Controller
    v     = 0.3*sqrt(e(1)^2+e(2)^2);
    alpha = 0.2*e(3);
  
    % Physical constraints
    if abs(alpha)>pi/4, alpha = pi/4*sign(alpha); end
    if abs(v)>0.8, v = 0.8*sign(v); end
    
    % Robot motion simulation
    dq = [v*cos(q(3)); v*sin(q(3)); v/d*tan(alpha)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    
    wmr.setPose(q);
    sigU.plot(t(k), [v, alpha]);
    OFig.pause(Ts);
end

sigU.fin();