Ts = 0.1; % Sampling time
xTrue = [1; 2; pi/6]; % True initial pose
x = [3; 0; 0]; % Initial pose estimate
P = diag([9 9 0.6]); % Initial covariance matrix of the pose estimate
Q = diag([0.1 0.1]); % Noise covariance matrix of movement actuator
R = diag([0.5 0.3]); % Noise covariance matrix of distance and
                     % angle measurement
enableNoise = 1; % Enable noise: 0 or 1
N = 300; % Number of simulation sample times
marker = [0 0; 5 5]; % Positions of markers
R = R(1,1); % Only distance measurement

% Particle initialization
nParticles = 500;
xP = repmat(xTrue, 1, nParticles) + diag([4 4 1])*randn(3, nParticles);
W = ones(nParticles, 1)/nParticles; % All particles have equal probability

% Figures
hAni = OFig(); axis equal; axis([-2, 6, -0.5, 5.5]);
xlabel('$x$'); ylabel('$y$');

sigX = OSig(2); sigX.axes('$t~[\mathrm{s}]$', '$x~[\mathrm{m}]$', '$y~[\mathrm{m}]$', '$\varphi~[\mathrm{rad}]$');
sigX.sig('r-', 'b--'); sigX.plot(0, x, xTrue);

% Animation
hAni.axes();
esta = OWmr(); esta.showDistance('c:'); esta.setPose(x);
est = OWmr(); est.showWmr('r:'); est.showDistance('m:');
              est.showPath('r.-'); est.showParticles('go-');
              est.setPose(x);
wmra = OWmr(); wmra.showDistance('c-'); wmra.setPose(xTrue);
wmr = OWmr(); wmr.showWmr('y-'); wmr.showDistance('m-');
              wmr.showPath('b--'); wmr.setPose(xTrue);
m1 = OMarker(); m1.showMarker('m-'); m1.setTex('$M1$');
                m1.setPose(marker(1,:));
m2 = OMarker(); m2.showMarker('c-'); m2.setTex('$M2$');
                m2.setPose(marker(2,:));

OFig.pause(Ts);

% Loop
for k = 1:N
    u = [0.5; 0.5]; % Movement command (translational and angular velocity)
    u_sum = u + sqrt(Q)*randn(2, 1)*enableNoise;

    % Simulation of the true mobile robot state (pose)
    xTrue = xTrue + Ts*[u_sum(1)*cos(xTrue(3)); ...
                        u_sum(1)*sin(xTrue(3)); ...
                        u_sum(2)];
    xTrue(3) = wrapToPi(xTrue(3));

    % Simulation of the true noisy measurements (distance)
    zTrue = [];
    for m = 1:size(marker, 1)
        dist = sqrt((marker(m,1)-xTrue(1))^2 + (marker(m,2)-xTrue(2))^2);
        zz = [dist] + sqrt(R)*randn(1, 1)*enableNoise;
        zz(1) = abs(zz(1));
        zTrue = [zTrue; zz];
    end

    % Prediction
    for p = 1:nParticles
        % Particles are moved according to the noise model
        un = u + sqrt(Q)*randn(2, 1)*1;
        xP(:,p) = xP(:,p) + Ts*[un(1)*cos(xP(3,p)); ...
                                un(1)*sin(xP(3,p)); ...
                                un(2)];
        xP(3,p) = wrapToPi(xP(3,p));
    end

    % Correction
    for p = 1:nParticles
        % Estimated measurement for every particle
        z = [];
        for m = 1:size(marker, 1)
            dist = sqrt((marker(m,1)-xP(1,p))^2 + (marker(m,2)-xP(2,p))^2);
            zz = [dist];
            zz(1) = abs(zz(1));
            z = [z; zz];
        end

        Innov = zTrue - z; % Determine innovation

        % Determine particle weights (particle probability)
        % Measurement covariance matrix
        RR = diag(repmat(diag(R), size(marker, 1), 1));
        W(p) = exp(-0.5*Innov.'*inv(RR)*Innov) + 0.0001;
    end

    iNextGeneration = obtainNextGenerationOfParticles(W, nParticles);
    xP = xP(:,iNextGeneration);

    % The new state estimate is the average of all the particles
    x = mean(xP, 2);
    x(3) = wrapToPi(x(3));
    % For robot orientation use the most likely particle instead of the
    % average angle of all the particles.
    [gg, ggi] = max(W);
    x(3) = xP(3,ggi);
    
    % Animation
    wmr.setPose(xTrue);
    wmra.setPose(xTrue);
    est.setPose(x);
    esta.setPose(x);
    wmr.updateRange(m1);
    wmra.updateRange(m2);
    est.updateRange(m1);
    esta.updateRange(m2);
    est.updateParticles(xP);

    % Signals
    sigX.plot(k*Ts, x, xTrue);

    OFig.pause(Ts);
end

% Signals
sigX.fin([1 2]);