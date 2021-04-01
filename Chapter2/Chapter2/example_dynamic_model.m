Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time

% Reference
freq = 2*pi/30;
   xRef = 1.1 + 0.7*sin(freq*t);     yRef = 0.9 + 0.7*sin(2*freq*t);
  dxRef = freq*0.7*cos(freq*t);     dyRef = 2*freq*0.7*cos(2*freq*t);
 ddxRef =-freq^2*0.7*sin(freq*t);  ddyRef =-4*freq^2*0.7*sin(2*freq*t);
dddxRef =-freq^3*0.7*cos(freq*t); dddyRef =-8*freq^3*0.7*cos(2*freq*t);
qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
dvRef = (dxRef.*ddxRef+dyRef.*ddyRef)./vRef;
dwRef = (dxRef.*dddyRef-dyRef.*dddxRef)./vRef.^2 - 2.*wRef.*dvRef./vRef;

q = [qRef(:,1); vRef(1); wRef(2)]; % Inital robot state
m = 0.75; J = 0.001; L = 0.075; r = 0.024; d = 0.01; % Robot parameters

hAni = OFig(); axis equal; axis([-0.1, 2.3, 0, 1.8]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OWmr(); ref.showPath('r--', true); % ref.showWmr('g:', 0.2, 0.04, 0.009);
% wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:', 0.2, 0.04, 0.009);
wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); % wmr.showWmr('y-', 0.2, 0.04, 0.009);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$T_r~[\mathrm{Nm}]$', '$T_l~[\mathrm{Nm}]$'); sigU.sig('b-');

for k = 1:length(t)
    % Calculate torques from the trajectory and inverse model
    v = vRef(k); w = wRef(k); dv = dvRef(k); dw = dwRef(k);
    tau = [(r*(dv*m-d*w*m*w))/2 + (r*(dw*(m*d^2+J) + d*w*m*v))/L; ...
           (r*(dv*m-d*w*m*w))/2 - (r*(dw*(m*d^2+J) + d*w*m*v))/L];
    
    wmr.setPose(q(1:3)); ref.setPose(qRef(:,k));
    sigU.plot(t(k), tau);
    OFig.pause(Ts);
     
    % Robot motion simulation using kinematic and dynamic model
    phi = q(3); v = q(4); w = q(5);
    F = [v*cos(phi) - d*w*sin(phi); ...
         v*sin(phi) + d*w*cos(phi); ...
         w; ...
         d*w^2; ...
       -(d*w*v*m)/(m*d^2 + J)];
    G = [0,                    0; ...
         0,                    0; ...
         0,                    0; ...
         1/(m*r),              1/(m*r); ...
         L/(2*r*(m*d^2 + J)), -L/(2*r*(m*d^2 + J))];
    dq = F + G*tau; % State space model
    q = q + dq*Ts; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
end

sigU.fin();