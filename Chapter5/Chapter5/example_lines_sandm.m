script_laserscandata;
X = [x, y]; % Laser scan data
[N, M] = size(X);

% Init
C = 50; % Maximum number of clusters
clusters = 1; % Last active cluster
dMin = 0.06; % Threshold distance to split cluster

sizeOfCluster = zeros(C,1); % Number of points in clusters
clusterBounds = zeros(C,2); % Indexes of bounary points in clusters
clusterParams = zeros(C,M+1); % Cluster parameters
splitCluster = zeros(C,1); % Split flag

% At the begining add all points to a single cluster
sizeOfCluster(clusters,1) = N;
clusterBounds(clusters,:)= [1, N]; % Points are ordered
splitCluster(clusters,1) = 1; % Initial cluster can be split

exit = false;
while ~exit
    exit = true;
    tmpLastCluster = clusters;
    for i = 1:tmpLastCluster
        if splitCluster(i)
            p0 = clusterBounds(i,1); % First point in the cluster
            p1 = clusterBounds(i,2); % Last point in the cluster
            
            % Estimation of cluster params in the least-square sense (LSQ)
            Psi = [X(p0:p1,:), ones(p1-p0+1,1)];
            [~, ~, V] = svd(Psi);
            thetaEst = V(:,3);
            % Transform line ax+by+c=0 to normal form
            s = -sign(thetaEst(3)); if s==0, s = 1; end
            mi = 1/sqrt(thetaEst(1)^2+thetaEst(2)^2)*s;
            Theta = thetaEst*mi;
            
            % Estimation of simple line parameters (through the first and
            % the last point). These params are used when the split point
            % is at the cluster boundary.
            if abs(X(p1,1)-X(p0,1))<100*eps % Vertical line
                a = 1; b = 0; c = -X(1,1);
            else
                a = (X(p1,2)-X(p0,2))/(X(p1,1)-X(p0,1));
                b = -1;
                c = -a*X(p0,1) + X(p0,2);
            end
            % Transform line ax+by+c=0 to normal form
            thetaEst = [a; b; c];
            s = -sign(thetaEst(3)); if s==0, s = 1; end
            mi = 1/sqrt(thetaEst(1)^2+thetaEst(2)^2)*s;
            Theta0 = thetaEst*mi;
            
            % Store LSQ params
            clusterParams(i,:) = Theta.';
            ind = p0:p1;
            XX = X(ind,:);
            
            % Calculate distance obtained from the first and the last
            % cluster sample (simple line)
            dik = [XX, ones(size(XX,1),1)]*Theta0;
            [dd0, iii] = max(abs(dik)); ii0 = ind(iii);
            
            % Calculate distance from line obtained in the LSQ sense
            dik = [XX, ones(size(XX,1),1)]*Theta;
            [dd, iii] = max(abs(dik)); ii = ind(iii);
            
            % Cluster splitting
            doSplit = 0;
            if dd>dMin && (ii-p0)>=2 && (p1-ii)>=1 % Use LSQ line
                if clusters<C
                    iiFin = ii; % Split location
                    doSplit = 1;
                    clusterParams(i,:) = Theta.';
                end
            elseif dd0>dMin && (ii0-p0)>=2 && (p1-ii0)>=1 % Use simple line
                if clusters<C
                    iiFin = ii0; % Split location
                    doSplit = 1;
                    clusterParams(i,:) = Theta0.';
                end
            else
                splitCluster(i) = 0;
            end
            
            
            if doSplit==1 && clusters<C
                % Split the cluster to cluster A and B
                clusters = clusters + 1; % New cluster
                % First and last point in cluster A
                clusterBounds(i,1);
                clusterBounds(i,2) = iiFin-1;
                splitCluster(i) = 1;
                % First and last point in cluster B
                clusterBounds(clusters,1) = iiFin+1;
                clusterBounds(clusters,2) = p1;
                splitCluster(clusters) = 1;

                exit = false;
            end
        end
    end
end

roi = [floor(min(x)) ceil(max(x)) floor(min(y)) ceil(max(y))];
OFig(1.5,1,1); axis equal; axis(roi); xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
plot(x, y, 'k.');
for i = 1:clusters
    bp = clusterBounds(i,:);
    cx = x(bp); cy = y(bp);
    a = clusterParams(i,1); b = clusterParams(i,2); c = clusterParams(i,3);
    if abs(a)>abs(b)
        cx = -(b*cy+c)/a;
    else
        cy = -(a*cx+c)/b;
    end
    line(cx, cy, 'Color', 'r', 'LineWidth', 1);
 end