biasA = [1; 1; 1]*0.02; biasW = [1; 1; 1]*0.04; % Sensor bias
SigmaA = 0.1; SigmaW = 0.05; % Sensor noise

nSteps = 6000; dT = 0.001; t = 0; % Time samples, step size

AccStore = nan(3,nSteps); GyroStore = nan(3,nSteps);
XTrueStore = nan(7,nSteps); TStore = nan(1,nSteps);
XEstStore = nan(7,nSteps);

for k = 1:nSteps
    % INS motion simulation: get true robot pose, acceleration and rotation
    x = [cos(t); sin(2*t); 0];    % Position 
    v = [-sin(t); 2*cos(2*t); 0]; % Velocity
    a = [-cos(t);-4*sin(2*t); 0]; % Acceleration

    fi = [0; 0; atan2(v(2), v(1))]; % Euler angles (from world to INS) 
    dfi = [0; 0; (v(1)*a(2) - v(2)*a(1))/(v(1)^2 + v(2)^2)]; % Derivative  
    Rx = rotX(fi(1)); Ry = rotY(fi(2)); Rz = rotZ(fi(3));
    dRx = [0, 0, 0; 0,-sin(fi(1)), cos(fi(1)); 0,-cos(fi(1)),-sin(fi(1))];
    dRy = [-sin(fi(2)), 0,-cos(fi(2)); 0, 0, 0; cos(fi(2)), 0,-sin(fi(2))];
    dRz = [-sin(fi(3)), cos(fi(3)), 0;-cos(fi(3)),-sin(fi(3)), 0; 0, 0, 0];
    R = Rx*Ry*Rz;
    dR = dRx*Ry*Rz*dfi(1) + Rx*dRy*Rz*dfi(2) + Rx*Ry*dRz*dfi(3);
    q = dcm2quat(R).'; % Quaterion from world to INS
    
    % Gyro measurements
    Omega = dR*R.'; % Skew-symmetric form of the angular rate vector wb
    wb = -[Omega(3,2); Omega(1,3); -Omega(1,2)]; % Angular rates of the INS 
    
    % Accelometer measurements
    agDyn = a; % Dynamic acceleration in world coordinates
    agGrav = [0; 0; 9.81]; % Gravitation
    Rearth = eye(3); % Assume world and Earth frames are the same
    wg = R.'*wb; % True angular rates in world frame
    wgSkew = [0 -wg(3) wg(2); wg(3) 0 -wg(1); -wg(2) wg(1) 0];
    vg = v; % Velocity in world frame

    % Measured angular rates and accelerations
    wbMea = wb + biasW + randn(3,1)*SigmaW;
    abMea = R*(agDyn + Rearth*agGrav + wgSkew*vg) +biasA+randn(3,1)*SigmaA;
    
    % Inertial navigation
    if k==1 % Initialization
        qEst = q; xEst = x; vEst = v; % Init true initial values
    else % Update
        % Gyro
        wx = wbMea(1); wy = wbMea(2); wz = wbMea(3);
        OMEGA = [ 0 -wx -wy -wz; ...
                 wx   0  wz -wy; ...
                 wy -wz   0  wx; ...
                 wz  wy -wx   0];
        dQest = 0.5*OMEGA*qEst;
        qEst = qEst + dQest*dT; % Quaterninon integration
        qEst = qEst/norm(qEst); % Quaternion normalization
        % Acceleration
        agGrav = [ 0; 0; 9.81]; % Gravity
        R_ = quat2dcm(qEst.'); % Ratation from world to INS
        Rearth = eye(3); % Earth frame is the same as world frame 
        wg_ = R_.'*[wx; wy; wz]; % Angular rates in world frame
        wgSkew_ = [0 -wg_(3) wg_(2); wg_(3) 0 -wg_(1); -wg_(2) wg_(1) 0];
        Aest = R_.'*abMea - Rearth*agGrav - wgSkew_*vEst; % Estimated a
        vEst = vEst + Aest*dT; % Estimated velocity
        xEst = xEst + vEst*dT; % Estimated position
    end
    
    % Store values
    AccStore(:,k) = abMea; GyroStore(:,k) = wbMea;
    XTrueStore(:,k) = [x; q]; TStore(:,k) = t;
    XEstStore(:,k) = [xEst; qEst];
    
    t = t + dT;
end
hFig1 = OFig(); view(-40, 30); axis equal;
plot3(XEstStore(1,:), XEstStore(2,:), XTrueStore(3,:), '--', 'Color', [1 1 1]*0.6);
plot3(XTrueStore(1,:), XTrueStore(2,:), XTrueStore(3,:), 'b-', ...
      XEstStore(1,:), XEstStore(2,:), XEstStore(3,:), 'r--');
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$'); zlabel('$z~[\mathrm{m}]$');
axis([-2 1.5 -2 1.5 0 3]);

hFig2 = OFig(3,2,4);
hFig2.xlabel('$t~[\mathrm{s}]$'); hFig2.ylabel({'$q_0$', '$q_1$', '$q_3$', '$q_4$'});
hFig2.axes(1); plot(TStore, XTrueStore(4,:), 'b-', TStore, XEstStore(4,:), 'r--'); ylim([-1 1]);
hFig2.axes(2); plot(TStore, XTrueStore(5,:), 'b-', TStore, XEstStore(5,:), 'r--'); ylim([-1 1]);
hFig2.axes(3); plot(TStore, XTrueStore(6,:), 'b-', TStore, XEstStore(6,:), 'r--'); ylim([-1 1]);
hFig2.axes(4); plot(TStore, XTrueStore(7,:), 'b-', TStore, XEstStore(7,:), 'r--'); ylim([-1 1]);

hFig3 = OFig(3,2);
hFig3.xlabel('$t~[\mathrm{s}]$'); hFig3.ylabel({'$a_x~[\mathrm{m/s^2}]$', '$\omega_x~[\mathrm{s^{-2}}]$', '$a_y~[\mathrm{m/s^2}]$', '$\omega_y~[\mathrm{s^{-2}}]$', '$a_z~[\mathrm{m/s^2}]$', '$\omega_z~[\mathrm{s^{-2}}]$'});
hFig3.axes(1); plot(TStore, AccStore(1,:), 'b-'); ylim([-10 10]);
hFig3.axes(3); plot(TStore, AccStore(2,:), 'b-'); ylim([-10 10]);
hFig3.axes(5); plot(TStore, AccStore(3,:), 'b-'); ylim([0 20]);
hFig3.axes(2); plot(TStore, GyroStore(1,:), 'b-'); ylim([-10 10]);
hFig3.axes(4); plot(TStore, GyroStore(2,:), 'b-'); ylim([-10 10]);
hFig3.axes(6); plot(TStore, GyroStore(3,:), 'b-'); ylim([-10 10]);
