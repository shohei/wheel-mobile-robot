script_laserscandata;
X = [x, y]; % Laser scan data
Kapa_Max=7; 

[n,m] = size(X); dimension = m;
%%%%%% init
Nr_cloud_max = 20; Current_clust = 1;

Nr_points_in_cloud = zeros(Nr_cloud_max,1); % vector specifying the number 
% of data points in the cloud where rows correspond to different clouds
M_of_clouds = zeros(Nr_cloud_max, dimension); % matrix - rows correspond to 
% different clouds and columns to elements of the input vector 
V_of_clouds = zeros(Nr_cloud_max, dimension); 
VarD_of_cloud = zeros(Nr_cloud_max,1); % distance variance of points 
% from cluster
M_dist = zeros(Nr_cloud_max,1); % distances among points in cluster
Eig_of_clouds = zeros(Nr_cloud_max, dimension);
EigLat_of_cloud = zeros(Nr_cloud_max, dimension);
Points_in_buffer=zeros(6,dimension);  % stores up to 6 samples
StartEndX_points_in_cloud = zeros(Nr_cloud_max,2);

M_old=0;V_old=0;

%%%%%%%%%%%%% init %%%%%%%%%%%%%%%%% 
% first cluster receives 3 points (supposing that are on the line)
Nr_points_in_cloud(Current_clust,1)=3; 
Nr_points_in_buffer=0; 

N=3;
XX=X(1:N,:);
M = sum(XX)/N;
Mold = M;

V=[0, 0];
sumD=0;
for i=1:N
    V=V+(XX(i,:)-M).*(XX(i,:)-M);    % variance of cluster points 
end
V=V/(N-1);
Vold = V;

% covariance matrix of samples
Vmat=(XX-repmat(M,N,1))' * (XX-repmat(M,N,1)) / (N-1);

p = [sqrt(V(1,1)) sqrt(V(1,2))];   
signC=sign(X(2,:)-X(1,:)); 
p = p.*signC / sqrt(p*p'); % 1. eigen vector, normalised 
pL = [p(1,2) , -p(1,1)];  % 2. eigen vector, normalised

% distance from line 
sumD=0;
for i=1:N
   sumD=sumD + ( ((X(i,:) - M)*pL') /(pL*pL') )^2;   
end
varD=sumD/(N-1);
varDold=varD;

% mean distance between points
d1= sqrt( (X(1,:)-X(2,:))*(X(1,:)-X(2,:))' );
d2= sqrt( (X(3,:)-X(2,:))*(X(3,:)-X(2,:))' );
d_med_toc=(d1+d2)/2;    

% storing
M_of_clouds( Current_clust,:)=M;
V_of_clouds( Current_clust,:)=V;
Eig_of_clouds (Current_clust,:)=p;
EigLat_of_cloud( Current_clust,:)=pL;
VarD_of_cloud(Current_clust,1) = varD; 

M_dist(Current_clust,1) =d_med_toc;
StartEndX_points_in_cloud(Current_clust,1)=1;
StartEndX_points_in_cloud(Current_clust,2)=3;


for k=4:n   % loop through all samples
    % compute distance to current sample
    pL=EigLat_of_cloud( Current_clust,:); 
    M=M_of_clouds( Current_clust,:);   
    V=V_of_clouds( Current_clust,:);
    varD=VarD_of_cloud(Current_clust,1);
    
    d=abs( (X(k,:) - M)*pL');
    
   if(Nr_points_in_cloud(Current_clust)>1)
       d_med_toc=sqrt((X(k,:) - X(k-1,:))*(X(k,:) - X(k-1,:))'); 
   end
   
   % new point belongs to cluster if its distance to cluster is small
   % enough and sample are close enough
   if (d<Kapa_Max*sqrt(varD) & d_med_toc < 2*M_dist(Current_clust,1))  
       StartEndX_points_in_cloud(Current_clust,2)=k;
       Nr_points_in_buffer=0;  
       j=Nr_points_in_cloud(Current_clust)+1;   % increase cluster samples
       M=Mold+(X(k,:)-Mold)/j; % recursive mean 
       V=Vold*(j-2)/(j-1)+j*(M-Mold).^2;   % recursive variance
       
       % covariance matrix of data
       Vmat=Vmat*(j-2)/(j-1) + 1/j*(X(k,:)-Mold)'*(X(k,:)-Mold);
       
       Mold=M;
       Vold=V;
       
       p = [sqrt(V(1,1)) sqrt(V(1,2))];       
       signC=sign(X(k,:)-X(StartEndX_points_in_cloud(Current_clust,1),:));
       
       p = p.*signC / sqrt(p*p'); % 1. eigen vector, normalised 
       pL = [p(1,2) , -p(1,1)];  % 2. eigen vector, normalised
              
       % recursive distance variance 
       d=abs( (X(k,:) - M)*pL') ;     
       varD=varDold*(j-2)/(j-1)+d^2/j;      
       varDold=varD;
       
       % storing
       M_of_clouds( Current_clust,:)=M;
       V_of_clouds( Current_clust,:)=V;
       Eig_of_clouds (Current_clust,:)=p;
       EigLat_of_cloud( Current_clust,:)=pL;
       Nr_points_in_cloud(Current_clust)=j; 
       VarD_of_cloud(Current_clust,1) = varD;  

    else   % new point does not belong to cluster, make new cluster
        
        Nr_points_in_buffer= Nr_points_in_buffer+1;
        Points_in_buffer(Nr_points_in_buffer, :)=X(k,:);
        
        % new cluster need to have 3 consistent samples
        if(Nr_points_in_buffer>=3)   

            XX=Points_in_buffer(1:Nr_points_in_buffer,:);
            M = sum( XX )/Nr_points_in_buffer;
            
            d_min_toc=0;
            V=[0, 0];
            for i=1:Nr_points_in_buffer               
                d_tmp=(XX(i,:)-M).*(XX(i,:)-M);
                V=V+d_tmp;
                if (d_tmp>d_min_toc), d_min_toc=d_tmp; end
            end
            V=V/(Nr_points_in_buffer-1);
            
            Mold = M;
            Vold = V;
         
            p = [sqrt(V(1,1)) sqrt(V(1,2))]; 
            signC=sign(XX(Nr_points_in_buffer,:)-XX(1,:)); 
            p = p.*signC / sqrt(p*p'); 
            pL = [p(1,2) , -p(1,1)]; 
 
            Vmat=(XX-repmat(M,N,1))' * (XX-repmat(M,N,1)) / (N-1);

            % test consistency of samples 
            sumD=0;
            for i=1:N
               d=abs((XX(i,:) - M)*pL') ;
               sumD=sumD+ d^2;   
            end
            varD=sumD/(N-1);
            varDold=varD;
            
            d1= sqrt( (XX(1,:)-XX(2,:))*(XX(1,:)-XX(2,:))' );
            d2= sqrt( (XX(3,:)-XX(2,:))*(XX(3,:)-XX(2,:))' );

            if ( d<Kapa_Max/2*sqrt(varD) & abs(d2-d1)<min(d1,d2) )  
                % samples are consistent and correctly spaced  
                Current_clust = Current_clust+1;
                M_of_clouds( Current_clust,:)=M;
                V_of_clouds( Current_clust,:)=V;
                Eig_of_clouds (Current_clust,:)=p;
                EigLat_of_cloud( Current_clust,:)=pL;
                Nr_points_in_cloud(Current_clust)=2; 
                VarD_of_cloud(Current_clust,1) = varD;   

                StartEndX_points_in_cloud(Current_clust,1)=k-2;
                StartEndX_points_in_cloud(Current_clust,2)=k;

                d_med_toc=(d1+d2)/2;  
                M_dist(Current_clust,1) = d_med_toc;
            end   
            Nr_points_in_buffer=0;    % clear buffer
        
        else    % wait until buffer has 3 samples 
          %  
        end
   end     
end         % end: loop through all samples 

% drawing 
roi = [floor(min(x)) ceil(max(x)) floor(min(y)) ceil(max(y))];
OFig(1.5,1,1); axis equal; axis(roi); xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
plot(X(:,1), X(:,2), 'k.');
for i=1:Current_clust
    S=10;
    
    Pcur = Eig_of_clouds (i,:);
    Mcur = M_of_clouds(i,:);
    d_med_toc=M_dist(i,1);

    p1=-EigLat_of_cloud( i,1);  p2=-EigLat_of_cloud( i,2);
    
    mx=Mcur(1,1); my=Mcur(1,2);
   
    x1=X(StartEndX_points_in_cloud(i,1),1);
    x2=X(StartEndX_points_in_cloud(i,2),1);

    y1=(p2*my-p1*(x1-mx))/p2; y2=(p2*my-p1*(x2-mx))/p2;
    line([x1 x2], [y1 y2], 'Color', 'r', 'LineWidth',1);
end