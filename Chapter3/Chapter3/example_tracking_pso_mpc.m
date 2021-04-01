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

vMax = 1; wMax = 15; % Valocity constraints

% Swarm initialization
iterations = 20; % Number of iterations
omega = 0.5*0.5; % Inertia
c1 = 0.5*1; % Correction factor
c2 = 0.5*1; % Global correction factor
N = 25; % Swarm size
swarm = zeros([2,N,4]);
uBest = [0; 0];

H = 3; % Prediction horizon

for k = 1:length(t)-H
    % Initial swarm position
    swarm(:,:,1) = repmat(uBest, 1, N) + diag([0.1; 3])*randn(2,N);
    swarm(:,:,2) = 0; % Initial particle velocity
    swarm(1,:,4) = 1000; % Best value so far
    
    for iter = 1:iterations % PSO iteratively find best solution
        % Evaluate particles parameters
        for i = 1:N
            % Compute new predicted pose of the robot using i-th particle
            % parameters (input velocities) and compare obtained predicted
            % pose to the reference pose.
            vwi = swarm(:,i,1); 
            ui = vwi + uRef(:,k); % Feedback and feedforward
            qk = q; % Current robot pose
            % Predict robot pose using particle parameters (velocities)
            J = 0;
            for h = 1:H
                qk = qk + Ts*[cos(qk(3)), 0; sin(qk(3)), 0; 0, 1]*ui;
                qk(3) = wrapToPi(qk(3)); % Correct angle range
                e = [cos(qk(3)), sin(qk(3)), 0; ...
                    -sin(qk(3)), cos(qk(3)), 0; ...
                     0,          0,          1]*(qRef(:,k+h)-qk); % Error
                e(3) = wrapToPi(e(3)); % Correct angle range
                Qt = diag([4; 80; 0.1]); Rt = diag([1; 1]*0.0001);% Weights
                J = J + e.'*Qt*e + vwi.'*Rt*vwi; % Cost function
            end
            if J<swarm(1,i,4) % if new parameter is better, update:
                swarm(:,i,3) = swarm(:,i,1); % param values (v and w)
                swarm(1,i,4) = J;            % and best critiria value.
            end
        end
        [~, gBest] = min(swarm(1,:,4)); % Global best particle parameters
        
        % Updating parameters by velocity vectors
        a = omega*swarm(:,:,2) + ...
            c1*rand(2,N).*(swarm(:,:,3) - swarm(:,:,1)) + ...
            c2*rand(2,N).*(repmat(swarm(:,gBest,3), 1, N) - swarm(:,:,1));
        % Max param increment, acceleration: aMax=3 ==> 3*Ts=0.1
        a(1,a(1,:)>0.1) = 0.1; a(1,a(1,:)<-0.1) = -0.1;
        % Max param increment, angula acceleration: aMax=60 ==> 60*Ts=2
        a(2,a(1,:)>2) = 2;     a(2,a(1,:)<-2) = -2;
        
        v = swarm(:,:,1) + a; % Update velocity
        % Limit velocity to preserve curvature ...
        [m, ii] = max([v(1,:)/vMax; v(2,:)/wMax; ones(1,N)]);
        i = ii==1; v(1,i) = sign(v(1,i))*vMax;
                     v(2,i) = v(2,i)./m(i);
        i = ii==2; v(2,i) = sign(v(2,i))*wMax;
                     v(1,i) = v(1,i)./m(i);
        
        swarm(:,:,2) = a; % Updated particle velocities (acceleration)
        swarm(:,:,1) = v; % Updated particle positions (velocities)
    end
    
    % Take the best particle to get robot command velocities
    uBest = swarm(:,gBest,1);
    u = uBest + uRef(:,k); % Feedback and feedforward
    
    % Velocity constraints
    if abs(u(1))>vMax, u(1) = sign(u(1))*vMax; end
    if abs(u(2))>wMax, u(2) = sign(u(2))*wMax; end
    
    wmr.setPose(q); ref.setPose(qRef(:,k));
    sigU.plot(t(k), uRef(:,k), u);
    OFig.pause(Ts);
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
end

OFig.pause(Ts);
sigU.fin();