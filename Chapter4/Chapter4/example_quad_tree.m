function example_quad_tree

enableAnimation = 0; % With or without animation

xMax = 15;      yMax = 10;      % Environment dimension
xObstMax = 1.5; yObstMax = 1.5; % Max dimension of an obstacle
nObst = 15;                     % Number of obstacles
maxVel = 15;                    % Maximum length of the cell edge
minVel = .5;                    % Minimum length of the cell edge

% Randomly defined obstacles
% x, y, fi, a, b  (rectangle centre, orientation, border size, border size)
obst = zeros(nObst,13);
obst(:,1) = xMax*rand(nObst,1);
obst(:,2) = yMax*rand(nObst,1);
obst(:,3) = 2*pi*rand(nObst,1);
obst(:,4) = xObstMax*(rand(nObst,1)+0.0);
obst(:,5) = yObstMax*(rand(nObst,1)+0.0);
% Vertices
xo = [-obst(:,4)/2, obst(:,4)/2, obst(:,4)/2,-obst(:,4)/2];
yo = [-obst(:,5)/2,-obst(:,5)/2, obst(:,5)/2, obst(:,5)/2];
for i = 1:size(obst,1)
    obst(i,6:9)   = obst(i,1)*ones(1,4) + ...
        xo(i,:)*cos(obst(i,3))-yo(i,:)*sin(obst(i,3));
    obst(i,10:13) = obst(i,2)*ones(1,4) + ...
        xo(i,:)*sin(obst(i,3))+yo(i,:)*cos(obst(i,3));
end

drawQT([], obst); axis([0 xMax 0 yMax]);
title('Obstacles in the environment');

% Draw quadtree
maxVel = maxVel;     % Max cell size
minVel = maxVel/2^4; % Min cell size

% Calculate quadtree
if enableAnimation
    h = msgbox('Quadtree. Animation by steps.', ' '); uiwait(h);
    [tree] = quadTree(obst, minVel,maxVel, 0, xMax, 0, yMax, 1, 1);
else
    [tree] = quadTree(obst, minVel, maxVel, 0, xMax, 0, yMax, 0, 0);
    drawQT(tree, obst);
end

end

function drawQT(drevo, Ovire)
figure, axis equal

sizeDrevo=size(drevo,2);
leafs=[];   % array of free leaf nodes
indL=1;
for i=1:sizeDrevo
   if( drevo(i).type ==0 ) 
    x=[drevo(i).boundingPoints(1) drevo(i).boundingPoints(2) drevo(i).boundingPoints(2) drevo(i).boundingPoints(1) drevo(i).boundingPoints(1)];
    y=[drevo(i).boundingPoints(3) drevo(i).boundingPoints(3) drevo(i).boundingPoints(4) drevo(i).boundingPoints(4) drevo(i).boundingPoints(3)];
    line(x,y);
    
    if(drevo(i).free ==1 )
        
        patch(x,y,'y');  
        
        
%         for n=drevo(i).visibleNodes
%             %plot([drevo(i).center(1),drevo(n).center(1)],[drevo(i).center(2),drevo(n).center(2)] )
%             line([drevo(i).center(1),drevo(n).center(1)],[drevo(i).center(2),drevo(n).center(2)],'LineWidth',2);
%         end
       % text(   drevo(i).center(1),...
       %         drevo(i).center(2), num2str(i));
        %
    end
   end
end

for i=1:size(Ovire,1)
    poza=Ovire(i,1:3);
    xo=[-Ovire(i,4)/2,Ovire(i,4)/2,Ovire(i,4)/2,-Ovire(i,4)/2,-Ovire(i,4)/2];
    yo=[-Ovire(i,5)/2,-Ovire(i,5)/2,Ovire(i,5)/2,Ovire(i,5)/2,-Ovire(i,5)/2];
    %% narisem sceno
    x=poza(1)*ones(1,5)+xo*cos(poza(3))-yo*sin(poza(3));
    y=poza(2)*ones(1,5)+xo*sin(poza(3))+yo*cos(poza(3));
    patch(x,y,'c')
end

end

%function quadTreeDekompozicija(minDim, xMin,xMax,yMin,yMax, anim, animNum)
function [drevo] = quadTree(ovire, minDim, RequredMaxDim, xMin,xMax,yMin,yMax, anim, animNum)
%
% Delitev prostora z stiriskim drevesom
% Ovire .... spisek vseh ovir v formatu 
% MinDim .... minimalna velikost polja  
% RequiredMaxDim .... maksimalna velikost polja
% Xmin,Xmax,Ymin,Ymax ..... koordinate okolice
% anim ..... shows steb by step tree construction (0 no animation, 1 step by step, <0 result, >0 time )
% animNum .... show cell id

%global leafs drevo %ovire


minDim=minDim*.5;

id=1;
idCurrent=id;


leafs=[];
indLeafs=1;

if(anim) drawEnvironment(ovire); end


% root node
drevo(id).type = 0;                          % 1 -node, 0 - leaf
drevo(id).id =1;                             % id of the node in the array
drevo(id).boundingPoints = [xMin,xMax,yMin,yMax];      % xmin, xmax, ymin, ymax
drevo(id).center =[(xMin+xMax)/2, (yMin+yMax)/2]; % center of the cell
drevo(id).parent =0;                         % id of the parent (0 - root)
drevo(id).branches = [0 0 0 0];              % childs of the node (if type=1)
drevo(id).free = 0;                          % is the cell occupacted
drevo(id).visibleNodes=[];
drevo(id).distVisibleNodes=[];
id=id+1;


if (anim) animateQT(drevo, idCurrent, anim,animNum); end

maxDim=min([xMax-xMin, yMax-yMin]);
dim = maxDim;

maxDimMax=max([xMax-xMin, yMax-yMin]);



%while (dim > minDim )
while (dim > minDim || maxDimMax>RequredMaxDim)
         
    occupied=isCellOccupied(idCurrent,drevo, ovire);
    
    if(occupied==0 && idCurrent==0) break; end
    
    
    ddx=(drevo(idCurrent).boundingPoints(2)-drevo(idCurrent).boundingPoints(1));
    ddy=(drevo(idCurrent).boundingPoints(4)-drevo(idCurrent).boundingPoints(3));
    
    maxDim=min([ddx, ddy]);
    dim = maxDim;


    dim=dim/2;
      
    maxDimMax=max([ddx, ddy]);
    

%    if( occupied && dim > minDim)    % split the node to 4 children
    if( occupied && dim > minDim || maxDimMax>RequredMaxDim)    % split the node to 4 children
        drevo(idCurrent).type = 1;
        drevo(idCurrent).branches = [id id+1 id+2 id+3]; 
        drevo(idCurrent).free = 0;

        dx=ddx/2;
        dy=ddy/2;
       
        min_x=drevo(idCurrent).boundingPoints(1);
        max_x=drevo(idCurrent).boundingPoints(2);
        min_y=drevo(idCurrent).boundingPoints(3);
        max_y=drevo(idCurrent).boundingPoints(4);
        
        
        
        drevo(id).type = 0;                          % 1 -node, 0 - leaf
        drevo(id).id =id;                             % id of the node in the array
        drevo(id).boundingPoints = [min_x,min_x+dx,min_y,min_y+dy];      % xmin, xmax, ymin, ymax
        drevo(id).center=[(drevo(id).boundingPoints(1)+drevo(id).boundingPoints(2))/2,(drevo(id).boundingPoints(3)+drevo(id).boundingPoints(4))/2];         
        drevo(id).parent =idCurrent;                         % id of the parent (0 - root)
        drevo(id).branches = [0 0 0 0];              % childs of the node (if type=1)
        drevo(id).free = 0;
        if (anim) animateQT(drevo,id, anim,animNum); end
        id=id+1;
       
            
        drevo(id).type = 0;                          % 1 -node, 0 - leaf
        drevo(id).id =id;                             % id of the node in the array
        drevo(id).boundingPoints = [min_x+dx,max_x,min_y,min_y+dy];      % xmin, xmax, ymin, ymax
        drevo(id).center=[(drevo(id).boundingPoints(1)+drevo(id).boundingPoints(2))/2,(drevo(id).boundingPoints(3)+drevo(id).boundingPoints(4))/2];         
        drevo(id).parent =idCurrent;                         % id of the parent (0 - root)
        drevo(id).branches = [0 0 0 0];              % childs of the node (if type=1)
        drevo(id).free = 0;
        if (anim) animateQT(drevo,id, anim,animNum); end
        id=id+1;       

        
        drevo(id).type = 0;                          % 1 -node, 0 - leaf
        drevo(id).id =id;                             % id of the node in the array
        drevo(id).boundingPoints = [min_x,min_x+dx,min_y+dy,max_y];      % xmin, xmax, ymin, ymax
        drevo(id).center=[(drevo(id).boundingPoints(1)+drevo(id).boundingPoints(2))/2,(drevo(id).boundingPoints(3)+drevo(id).boundingPoints(4))/2];                
        drevo(id).parent =idCurrent;                         % id of the parent (0 - root)
        drevo(id).branches = [0 0 0 0];              % childs of the node (if type=1)
        drevo(id).free = 0;
        if (anim) animateQT(drevo,id, anim,animNum); end
        id=id+1;
  
        
        drevo(id).type = 0;                          % 1 -node, 0 - leaf
        drevo(id).id =id;                             % id of the node in the array
        drevo(id).boundingPoints = [min_x+dx,max_x,min_y+dy,max_y];      % xmin, xmax, ymin, ymax
        drevo(id).center=[(drevo(id).boundingPoints(1)+drevo(id).boundingPoints(2))/2,(drevo(id).boundingPoints(3)+drevo(id).boundingPoints(4))/2];         
        drevo(id).parent =idCurrent;                         % id of the parent (0 - root)
        drevo(id).branches = [0 0 0 0];              % childs of the node (if type=1)
        drevo(id).free = 0;
        if (anim) animateQT(drevo,id, anim,animNum); end
        id=id+1;       
               
        
    elseif (occupied==0)
        drevo(idCurrent).free = 1; % type=0 -> leaf
        
       leafs(indLeafs)=idCurrent;         % zapomni si array listov
       indLeafs=indLeafs+1;

        
        if (anim) animateQT(drevo,idCurrent, anim,animNum); end
    else
        % Done for this branch!
        % break;
    end
      
    idCurrent =drevo(idCurrent).id+1;
    
    
    
end  % while




    % chech for leaves if they are free
%    for i=idCurrent-1:id-1
    for i=idCurrent:id-1
       occupied=isCellOccupied(i,drevo, ovire);
       if( occupied==0 )    
            drevo(i).free = 1;

           leafs(indLeafs)=i;
           indLeafs=indLeafs+1;

            if (anim) animateQT(drevo,i, anim,animNum); end
       end
    end  % for

  
  
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  
    % create visibility graph for all leaves
    drevo=createVisibilityGrapf(drevo,minDim,leafs);  
  
  

  
end % main fuction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawEnvironment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function drawEnvironment(ovire)
    %global ovire
    % izrisemo sceno
    figure, axis equal
    for i=1:size(ovire,1)
        poza=ovire(i,1:3);
        xo=[-ovire(i,4)/2,ovire(i,4)/2,ovire(i,4)/2,-ovire(i,4)/2,-ovire(i,4)/2];
        yo=[-ovire(i,5)/2,-ovire(i,5)/2,ovire(i,5)/2,ovire(i,5)/2,-ovire(i,5)/2];
        %% narisem sceno
        x=poza(1)*ones(1,5)+xo*cos(poza(3))-yo*sin(poza(3));
        y=poza(2)*ones(1,5)+xo*sin(poza(3))+yo*cos(poza(3));

        patch(x,y,'c')
        %line(x,y);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% animateQT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function animateQT(drevo,ii, anim,animNum)
   % global drevo
    x=[drevo(ii).boundingPoints(1) drevo(ii).boundingPoints(2) drevo(ii).boundingPoints(2) drevo(ii).boundingPoints(1) drevo(ii).boundingPoints(1)];
    y=[drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(4) drevo(ii).boundingPoints(4) drevo(ii).boundingPoints(3)];
    line(x,y);
    
    ss='';
    if (drevo(ii).free == 1) 
        ss='F'; 
        patch(x,y,'y');
    end
    
    if(animNum)
        text(   drevo(ii).boundingPoints(1)+(drevo(ii).boundingPoints(2)-drevo(ii).boundingPoints(1))/2,...
                drevo(ii).boundingPoints(3)+(drevo(ii).boundingPoints(4)-drevo(ii).boundingPoints(3))/2, strcat(num2str(ii),ss))
    end
    
    if(anim==1)
        pause;    
    elseif(anim>0)
        pause(anim);
    end
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% isCellOccupied
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function occupied=isCellOccupied(ii, drevo, ovire)
    %global drevo ovire
   
    % check approximate overlaping between cell and objects
    occupied=0;
    for i=1:size(ovire,1)
        
%         xo=[-ovire(i,4)/2,ovire(i,4)/2,ovire(i,4)/2,-ovire(i,4)/2];
%         yo=[-ovire(i,5)/2,-ovire(i,5)/2,ovire(i,5)/2,ovire(i,5)/2];
%         
%         x=ovire(i,1)*ones(1,4)+xo*cos(ovire(i,3))-yo*sin(ovire(i,3));
      
        x=ovire(i,6:9);
        
        xx=[drevo(ii).boundingPoints(1) drevo(ii).boundingPoints(2) min(x) max(x)];
        [gg,ggg]=sort(xx);
        
        if( ~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4))  )      % overlaping in x exist
            % check for ovelapping in y
%              y=ovire(i,2)*ones(1,4)+xo*sin(ovire(i,3))+yo*cos(ovire(i,3));
              y=ovire(i,10:13);
            
            yy=[drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(4) min(y) max(y)];
            [gg,ggg]=sort(yy);
            if(~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4)))        % overlaping in y exist
                               
                  
                % check if all points of obstacle (ovire) is outside of the
                % node
                ind2=[2 3 4 1];
                notN=[1 1 1 1]; %ones(4);

                xnnn=[drevo(ii).boundingPoints(1) drevo(ii).boundingPoints(2) drevo(ii).boundingPoints(2) drevo(ii).boundingPoints(1)];
                ynnn=[drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(4) drevo(ii).boundingPoints(4)];

                    % preverim vse tocke ovire zunaj meja noda
                 for(n=1:4)
                    % splosna enacba premice Ax+By+C=0
                        A=ynnn(ind2(n))-ynnn(n);
                        B=xnnn(n)-xnnn(ind2(n));
                        C=xnnn(n)*(ynnn(n)-ynnn(ind2(n)))+ynnn(n)*(xnnn(ind2(n))-xnnn(n));                     
                   
                    for(o=1:4)
                        d=A*x(o)+B*y(o)+C;
                        notN(o)=notN(o) && d<=0;    

                    end   
                   
                 end % for(n=ind1)
                
                 if(sum(notN)>0)
                     occupied=1;
                     return;
                 end
                    
                  
 
                  % check if all points of node is outside of the
                  % obstacle 
                  notO=[1 1 1 1];

                  for(o=1:4)
                        % splosna enacba premice Ax+By+C=0
                        A=y(ind2(o))-y(o);
                        B=x(o)-x(ind2(o));
                        C=x(o)*(y(o)-y(ind2(o)))+y(o)*(x(ind2(o))-x(o));                     
                        for(n=1:4)
                           xnn=xnnn(n);  
                           ynn=ynnn(n); 
                           d=A*xnn+B*ynn+C;
                           notO(n)=notO(n) && d<=0;    
                        end
                  end %(o=ind1)

                  if(sum(notO)>0)
                          occupied=1;
                          return;
                  end





                  

            
        % preverimo se ali so preseciscas premic znotraj objektov

    for(n=1:4)
    
        for(o=1:4)
            
                         A1=y(ind2(o))-y(o);
                         B1=x(o)-x(ind2(o));
                         C1=x(o)*(y(o)-y(ind2(o)))+y(o)*(x(ind2(o))-x(o));                     
   
                         A2=ynnn(ind2(n))-ynnn(n);
                         B2=xnnn(n)-xnnn(ind2(n));
                         C2=xnnn(n)*(ynnn(n)-ynnn(ind2(n)))+ynnn(n)*(xnnn(ind2(n))-xnnn(n));                     

                         D=A1*B2-A2*B1;
                         if(abs(D)>0)
                            
                             xp=(B1*C2-B2*C1)/D;
                             yp=(C1*A2-C2*A1)/D;
                             
                             if(xp>=xnnn(1)&&xp<=xnnn(2)&&yp>=ynnn(2)&& yp<=ynnn(3))
                               %  )
                                  %  occupied=1;
                                  %  return;
                                 if(xp>=min(x)&&xp<=max(x)&&yp>=min(y)&&yp<=max(y))
                                    occupied=1;
                                    return;
                                 end    

                             end
                             
                             xgg=min(x);
                             xgg=min(x);
                             

                         end
                            
            
        end
    
    end
    


% % 
%                           occupied=1;
%                           return;


            end % if overlap y
            
        end % if overlap x
        
        
        
    end % for
      
end
 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% createVisibilityGrapf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function drevo=createVisibilityGrapf(drevo, minDim, leafs)  
%function drevo=createVisibilityGrapf(minDim)  
   % global leafs drevo
  % global leafs drevo

% 
% sizeDrevo=size(drevo,2);
% leafs=[];   % array of free leaf nodes
% indL=1;
% for i=1:sizeDrevo
%    if( drevo(i).type ==0 && drevo(i).free ==1 ) 
%     leafs(indL)=i;
%     indL=indL+1;
%    end
% end


%for each leaf check neigborhood of QT

for i=leafs
    open=[];
    indOpen=0;
    close=[];
    indClose=0;
    
    xNmin=drevo(i).boundingPoints(1)-minDim/2; % neigborhood definition
    xNmax=drevo(i).boundingPoints(2)+minDim/2;
    yNmin=drevo(i).boundingPoints(3)-minDim/2;
    yNmax=drevo(i).boundingPoints(4)+minDim/2;
    
    % quarry QT for neigbors
    ii=1; % root node
                          
    xx=[drevo(ii).boundingPoints(1) drevo(ii).boundingPoints(2) xNmin xNmax];
    [gg,ggg]=sort(xx);
    if( ~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4))  )      % overlaping in x exist
            % check for ovelapping in y
            yy=[drevo(ii).boundingPoints(3) drevo(ii).boundingPoints(4) yNmin yNmax];
            [gg,ggg]=sort(yy);
            if(~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4)))        % overlaping in y exist              
                % overlapping exists between node area and searched area
                indOpen=indOpen+1;
                open(indOpen)=ii;
                %break;
            end           
    end
        
    while(indOpen>0)
 
        % check all the node branches in list open:
        %   if it has no branchec - and it is free leaf  put it on list close
        %   put the branches id on list open if they overlap 
        %   clear node from list open when all its branches are checked
        index=open(1);
        if(drevo(index).type ==0 )
            % put it on list close
             if(drevo(index).free ==1 && i~=index)   
                 
                 % preveri se ce ni diagonalen 26.8.09
                    x1min=drevo(i).boundingPoints(1);
                    x1max=drevo(i).boundingPoints(2);
                    y1min=drevo(i).boundingPoints(3);
                    y1max=drevo(i).boundingPoints(4);

                    x2min=drevo(index).boundingPoints(1);
                    x2max=drevo(index).boundingPoints(2);
                    y2min=drevo(index).boundingPoints(3);
                    y2max=drevo(index).boundingPoints(4);


                    if ~((x1max==x2min && y1min==y2max) ||...   % ne dotika se le v ogliscu 
                         (x1max==x2min && y1max==y2min) ||...     
                         (x1min==x2max && y1max==y2min) ||...
                         (x1min==x2max && y1min==y2max))
                       indClose=indClose+1;
                       close(indClose)=index; 
                    end
                   % konec: preverbe diagonalnosti 26.8.09      
%                       indClose=indClose+1;
%                       close(indClose)=index; 
%        
                
                
%                indClose=indClose+1;
%                close(indClose)=index; 
             end
        else   % it is node - check its branches
            for(b=1:4)
                indB=drevo(index).branches(b);  
                
                xx=[drevo(indB).boundingPoints(1) drevo(indB).boundingPoints(2) xNmin xNmax];
                [gg,ggg]=sort(xx);
                if( ~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4))  )      % overlaping in x exist
                    % check for ovelapping in y
                    yy=[drevo(indB).boundingPoints(3) drevo(indB).boundingPoints(4) yNmin yNmax];
                    [gg,ggg]=sort(yy);
                    if(~((ggg(1)==1 && ggg(2)==2) || (ggg(1)==3 && ggg(2)==4)))        % overlaping in y exist              
                        % overlapping exists between node area and searched area
                        indOpen=indOpen+1;
                        open(indOpen)=indB;
                        %break;
                    end           
                end
                
            end %for(b=1:4)
            
        end % if
        
        open(1)=[];
        indOpen=indOpen-1;
    end % while
    
    % put visible nodes in the node array
    drevo(i).visibleNodes=close;
    % calculate distances to neigbours
    itmp=1;
    for n=drevo(i).visibleNodes         
       % calculate distances
       drevo(i).distVisibleNodes(itmp)=sqrt((drevo(i).center(1)-drevo(n).center(1))^2 +....
                                            (drevo(i).center(2)-drevo(n).center(2))^2);    
       itmp=itmp+1;                                  
    end
    
 end % for

end
