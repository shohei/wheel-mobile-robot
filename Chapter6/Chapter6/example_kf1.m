% Linear model of the system in state-space
Ts = 0.1; % Sampling time
A = [1 0; 0 1];
B = [Ts 0; 0 Ts];
C = [1 0; 0 1];
F = B; % Noise is added directly to the input.

xTrue = [0; 0]; % True initial state
x = [3; 3]; % Initial state estimate
P = diag([10 10]); % Variance of the initial state estimate
Q = diag([1 1]/10); % Noise variance of the movement actuator
R = diag([10 10]); % Noise vairace of the GPS measurements

% Figures
hAni = OFig(); axis equal; axis([-4, 20, -9, 9]);
xlabel('$x$'); ylabel('$y$');

sigX = OSig(2, 2); sigX.axes('$t~[\mathrm{s}]$', '$x~[\mathrm{m}]$', '$y~[\mathrm{m}]$');
sigX.sig('c.:', 'r-', 'b--'); sigX.plot(0, [], x, xTrue);
sigP = OSig(2, 2); sigP.axes('$t~[\mathrm{s}]$', '$var(x)~[\mathrm{m^2}]$', '$var(y)~[\mathrm{m^2}]$');
sigP.sig('b-');

% Animation
hAni.axes();
mea = OMarker(); mea.showPath('c.:');
est = OMarker(); est.showMarker('r:', 0.9); est.showPath('r.-');
                 est.setPose(x);
wmr = OMarker(); wmr.showMarker('y-', 0.9); wmr.showPath('b--');
                 wmr.setPose(xTrue);

OFig.pause(Ts);

% Loop
N = 150;
for k = 1:N
    u = [1; 0]; % Movement command

    % Simulation of the true mobile robot position and measurement
    xTrue = A*xTrue + B*u + F*sqrt(Q)*randn(2, 1);
    zTrue = C*xTrue + sqrt(R)*randn(2, 1);

    % Position estimate based on known inputs and measurements
    %%% Prediction
    xPred = A*x + B*u;
    PPred = A*P*A.' + F*Q*F.';
    
    %%% Correction
    K = PPred*C.'/(C*PPred*C.' + R);
    x = xPred + K*(zTrue - C*xPred);
    P = PPred - K*C*PPred;
    
    % Animation
    wmr.setPose(xTrue);
    est.setPose(x);
    mea.setPose(zTrue);

    % Signals
    sigX.plot(k*Ts, zTrue, x, xTrue);
    sigP.plot(k*Ts, diag(P));

    OFig.pause(Ts);
end

% Signals
sigX.fin([1 2]); sigP.fin([1 2]);