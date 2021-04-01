Ts = 0.1; % Sampling time
xTrue = [1; 2; pi/6]; % True initial pose
x = [3; 0; 0]; % Initial pose estimate
P = diag([9 9 0.6]); % Initial covariance matrix of the pose estimate
Q = diag([0.1 0.1]); % Noise covariance matrix of movement actuator
R = diag([0.5 0.3]); % Noise covariance matrix of distance and
                     % angle measurement
enableNoise = 1; % Enable noise: 0 or 1
N = 300; % Number of simulation sample times

% Figures
hAni = OFig(); axis equal; axis([-2, 6, -0.5, 5.5]);
xlabel('$x$'); ylabel('$y$');

sigX = OSig(2); sigX.axes('$t~[\mathrm{s}]$', '$x~[\mathrm{m}]$', '$y~[\mathrm{m}]$', '$\varphi~[\mathrm{rad}]$');
sigX.sig('r-', 'b--'); sigX.plot(0, x, xTrue);
sigY = OSig(2, 2); sigY.axes('$t~[\mathrm{s}]$', '$d~[\mathrm{m}]$', '$\alpha~[\mathrm{rad}]$');
sigY.sig('b-', 'r--');
sigP = OSig(2, 3); sigP.axes('$t~[\mathrm{s}]$', '$var(x)~[\mathrm{m^2}]$', '$var(y)~[\mathrm{m^2}]$', ...
    '$\varphi~[\mathrm{rad^2}]$');
sigP.sig('b-');
sigI = OSig(2); sigI.axes('$t~[\mathrm{s}]$', 'Innovation');
sigI.sig('b-', 'r--', 'g-.', 'm:');

% Animation
hAni.axes();
est = OWmr(); est.showWmr('r:'); est.showDistance('m:');
              est.showAngle('m:', 1); est.showPath('r.-');
              est.setPose(x);
wmr = OWmr(); wmr.showWmr('y-'); wmr.showDistance('m-');
              wmr.showAngle('m-', 1); wmr.showPath('b--');
              wmr.setPose(xTrue);
m = OMarker(); m.showMarker('m-'); m.setTex('$M$');

OFig.pause(Ts);

% Loop
for k = 1:N
    u = [0.5; 0.5]; % Movement command (translational and angular velocity)
    uNoisy = u + sqrt(Q)*randn(2, 1)*enableNoise;

    % Simulation of the true mobile robot state (pose)
    xTrue = xTrue + Ts*[uNoisy(1)*cos(xTrue(3)); ...
                        uNoisy(1)*sin(xTrue(3)); ...
                        uNoisy(2)];
    xTrue(3) = wrapToPi(xTrue(3));

    % Simulation of the true noisy measurements (distance and angle)
    zTrue = [sqrt(xTrue(1)^2 + xTrue(2)^2); ...
             atan2(0-xTrue(2), 0-xTrue(1))-xTrue(3)] + ...
             sqrt(R)*randn(2, 1)*enableNoise;
    zTrue(1) = abs(zTrue(1));
    zTrue(2) = wrapToPi(zTrue(2));

    %%% Prediction (pose and speed estimation based on known inputs)
    xPred = x + Ts*[u(1)*cos(x(3)); ...
                    u(1)*sin(x(3)); ...
                    u(2)];
    xPred(3) = wrapToPi(xPred(3));

    % Jacobian matrices
    A = [1 0 -Ts*u(1)*sin(x(3)); ...
         0 1  Ts*u(1)*cos(x(3)); ...
         0 0  1];
    F = [Ts*cos(x(3)) 0; ...
         Ts*sin(x(3)) 0; ...
         0            Ts];
    PPred = A*P*A.' + F*Q*F.';


    % Estimated measurements
    z= [sqrt(xPred(1)^2 + xPred(2)^2); ...
        atan2(0-xPred(2), 0-xPred(1)) - xPred(3)];
    z(2) = wrapToPi(z(2));

    %%% Correction
    d = sqrt(xPred(1)^2 + xPred(2)^2);
    C = [xPred(1)/d   xPred(2)/d    0; ...
        -xPred(2)/d^2 xPred(1)/d^2 -1];
    K = PPred*C.'/(C*PPred*C.' + R);
    inov = zTrue - z;

    % Selection of appropriate innovation due to noise and angle wrapping
    inov(2) = wrapToPi(inov(2));

    x = xPred + K*inov;
    P = PPred - K*C*PPred;
    
    % Animation
    wmr.setPose(xTrue);
    est.setPose(x);
    wmr.updateRange(m);
    est.updateRange(m);

    % Signals
    sigX.plot(k*Ts, x, xTrue);
    tmp = {zTrue(1:2), zTrue(3:end)};
    sigY.plot(k*Ts, tmp{:});
    sigP.plot(k*Ts, diag(P));
    tmp = num2cell(inov);
    sigI.plot(k*Ts, tmp{:});

    OFig.pause(Ts);
end

% Signals
sigX.fin([1 2]); sigY.fin(); sigP.fin([1 2]); sigI.fin();