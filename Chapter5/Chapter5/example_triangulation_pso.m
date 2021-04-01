m = [0, 0; 5, 3; 1, 5].'; % Markers
r0 = [2; 2.5; pi/6]; % True robot pose (x, y and fi), unknown.

% Measured angles
alpha = wrapToPi(atan2(m(2,:)-r0(2), m(1,:)-r0(1))-r0(3));

% Method using PSO
iterations = 50; % Number of iterations
omega = 0.5; % Inertia
c1 = 0.5; % Correction factor
c2 = 0.5; % Global correction factor
N = 25; % Swarm size

% Initial swarm position
swarm = zeros([3,N,4]);
swarm(1,:,1) = 3 + randn(1,N); % Init of x
swarm(2,:,1) = 3 + randn(1,N); % Init of y
swarm(3,:,1) = 0 + randn(1,N); % Init of fi
swarm(:,:,2) = 0; % Initial velocity
swarm(1,:,4) = 1000; % Best value so far

hFig = OFig(); axis equal; axis([-2, 6, -0.5, 5.5]);
xlabel('$x$'); ylabel('$y$');

wmrs = cell(1,size(m,2));
sty = {'m-', 'c-', 'g-'};
for i = 1:size(m,2)
    wmrs{i} = OObject(); wmrs{i}.setPose(r0);
    wmrs{i}.showAngle(sty{i}, 1); wmrs{i}.showDistance(sty{i});
end
for i = 1:size(m,2)
    mi = OMarker(); mi.showMarker(sty{i});
    mi.setTex(sprintf('$M%d$', i)); mi.setPose(m(:,i));
    mi.updateRange(wmrs{i}); wmrs{i}.updateRange(mi);
end
wmr = OWmr(); wmr.showWmr('y-'); wmr.setPose(r0); wmr.showParticles('bo-');
wmp = OWmr(); wmp.showParticles('r.-');
wmp.updateParticles(swarm(:,:,1));

for iter = 1:iterations % PSO iteratively find best solution
    % Evaluate particles parameters
    for i = 1:N
        % Compute predicted angle measurments using i-th particle
        pEst = swarm(:,i,1); % Estimated particle parameters (x, y, fi)
        
        % Compare obtained predicted angles to the true angle measurments
        alphaEst = wrapToPi(atan2(m(2,:)-pEst(2), m(1,:)-pEst(1))-pEst(3));
        
        % Compute cost function
        cost = (alphaEst-alpha)*(alphaEst-alpha).';
        if cost<swarm(1,i,4) % If new parameter is better, update:
            swarm(:,i,3) = swarm(:,i,1); % param values (x, y and fi)
            swarm(1,i,4) = cost;         % and best criteria value.
        end
    end
    [~, gBest] = min(swarm(1,:,4)); % Global best particle parameters
    
    % Updating parameters by velocity vectors
    swarm(:,:,2) = omega*swarm(:,:,2) + ...
            c1*rand(3,N).*(swarm(:,:,3)-swarm(:,:,1)) + ...
            c2*rand(3,N).*(repmat(swarm(:,gBest,3), 1, N) - swarm(:,:,1));
    swarm(:,:,1) = swarm(:,:,1) + swarm(:,:,2);
    
    wmr.updateParticles(swarm(:,:,1));
    drawnow; pause(0.05);
end

r = swarm(:,gBest,1) % Solution, best pose estimate

wmr.setPose(r);