% Control based on distance to the obstacle 
if dObst>0.2 % Drive towards the goal
    phiRef = atan2(goal(2)-q(2), goal(1)-q(1));
    ePhi = wrapToPi(phiRef - q(3));
    dGoal = sqrt(sum((goal-q(1:2)).^2));
    g = [dGoal/2, 1]; % Control gains
else % Drive right arround the obstacle
    phiRef = wrapToPi(phiObst + pi*0);  % Add pi to go always left
    ePhi = wrapToPi(phiRef-q(3));
    g = [0.4, 5]; % Control gains
end
% Simple control of differential drive
v = g(1)*abs(cos(ePhi));
w = g(2)*ePhi;
v = min([v, 0.5]);