Ts = 0.03; % Sampling time
t = 0:Ts:15; % Simulation time
r = 0.2; % Distance parameter
qRef = [4; 4; 0]; % Reference pose
q = [1; 0; 100/180*pi]; % Initial pose

aMax = 5; % Maximum acceleration
vMax = 0.4; % Maximum speed
accuracy = vMax*Ts; % Accuracy
curveZone = 0.6; % Radius
Rr = 0.99*curveZone/2; % Radius
slowDown = false; v = 0; vDir = 1; w = 0; % Initial states
X = [0, 1; -1, 0]; % Support matrix: a.'*X*b = a(1)*b(2) - a(2)*b(1)

hAni = OFig(); axis equal; axis([-1, 7, -1, 5]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');
ref = OWmr(); ref.showWmr('g:'); ref.setPose(qRef); ref.setTxt('Goal', 0.5, pi/4, 'lb');
wmr0 = OWmr(); wmr0.setPose(q); wmr0.setTxt('Start', 0.5, pi/4, 'rb'); wmr0.showWmr('r:');
wmr = OWmr(); wmr.showWmr('y-'); wmr.showPath('b-', true); wmr.setPose(q);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v~[\mathrm{m/s}]$', '$\omega~[\mathrm{rad/s}]$'); sigU.sig('b-');
OFig.pause(Ts);

for k = 1:length(t)
    fin = [cos(qRef(3)); sin(qRef(3))];
    D = qRef(1:2); % Destination point
    S = q(1:2); % Robot position
    M = (D + S)/2;
    Ov = [cos(q(3)); sin(q(3))]; % Orientation vector
    SDv = D - S; % SD vector
    l2 = norm(SDv); % Distance
    
    if slowDown
        v = v - aMax*Ts; if v < 0, v = 0; end
        w = 0;
    else
        if fin.'*X*SDv > SDv.'*X*fin
            Ps = D - Rr*X.'*fin; % Circle centre
        else
            Ps = D - Rr*X*fin; % Circle centre
        end
        
        l = norm(Ps-S);
        if l < curveZone/2
            Dv = fin;
        else
            d = sqrt(sum((S-Ps).^2) - Rr^2);
            alpha = atan(Rr/d);
            phi = wrapTo2Pi(atan2(Ps(2)-S(2), Ps(1)-S(1)));
            U1 = S + d*[cos(phi+alpha); sin(phi+alpha)];
            U2 = S + d*[cos(phi-alpha); sin(phi-alpha)];
            if ((U1 - S).'*X*(Ps - U1)) * (fin.'*X*(Ps - D)) >= 0
                D = U1;
            else
                D = U2;
            end
            M = (D + S)/2;
            SDv = D - S;
            Dv = SDv/(norm(SDv)+eps);
        end
        
        if l2 > accuracy % If the position is not reached
            v = v + aMax*Ts; if v > vMax, v = vMax; end
            
            Ev = X*(D-S);
            DTv = X*Dv;
            if abs(DTv.'*X*Ev) < 0.000001 % Go on a straight line
                gamma = 0;
                Sv = SDv/(norm(SDv)+eps);
            else % Go on a circle
                C = DTv * Ev.'*X*(D - M)/(DTv.'*X*Ev) + D; % Circle centre
                if SDv.'*X*Dv > 0, a = 1; else a = -1; end
                Sv = a*X*(C-S);
                Sv = Sv/(norm(Sv)+eps);
                gamma = a*acos(Dv.'*Sv);
                if a*Sv.'*X*Dv < 0, gamma = a*2*pi - gamma; end
                l = abs(gamma*norm(S-C)); % Curve length
            end
            
            if v > eps
                if Ov.'*Sv < 0, vDir = -1; else vDir = 1; end % Direction
                ePhi = acos(vDir*Sv.'*Ov); % Angular error
                if vDir*Ov.'*X*Sv < 0, ePhi = -ePhi; end
                dt = l/v; if dt < 0.00001, dt = 0.00001; end
                w = gamma/dt + ePhi/dt*10*(1-exp(-l2/0.1)); % Angular speed
            else
                w = 0;
            end
        else
            slowDown = true;
        end
    end
    u = [vDir*v; w]; % Tangential and angular velocity
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.00; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    wmr.setPose(q);
    sigU.plot(t(k), u);
    OFig.pause(Ts);
end

sigU.fin();