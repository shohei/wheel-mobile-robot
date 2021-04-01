% Define trajectory with function handles
  x = @(u) cos(u);    y = @(u) sin(2*u);    % Path
 dx = @(u) -sin(u);  dy = @(u) 2*cos(2*u);  % First derivative
ddx = @(u) -cos(u); ddy = @(u) -4*sin(2*u); % Second derivative
v = @(u) sqrt(dx(u).^2 + dy(u).^2); % Tangential velocity
w = @(u) (dx(u).*ddy(u)-dy(u).*ddx(u))./(dx(u).^2+dy(u).^2); % Angular vel.
kappa = @(u) w(u)./v(u); % Curvature

u = 0:0.001:2*pi; % Time
arMax = 4; atMax = 2; % Acceleration constraints
vSP = 0.2; vEP = 0.1; % Initial and final velocity requirements
uSP = u(1); uEP = u(end); % Start point and end point
uTP = []; % Turn points
for i = 2:length(u)-1 % Determine turn points
    if all(abs(kappa(u(i))) > abs(kappa(u([i-1, i+1]))))
        uTP = [uTP, u(i)];
    end
end
up0 = sqrt(arMax./abs(v(uTP).*w(uTP))) ; % Derivative in turn points

velCnstr = false; % Enable velocity contraints (disabled)
if velCnstr
    vMax = 1.5*inf; %Velocity constraints
    for i = 1:length(uTP) % Make uTP in accordance with velocity constraint
       vvu = v(uTP(i)); vvt = vvu*up0(i);
       if abs(vvt) > vMax, up0(i) = abs(vMax/vvu); end
    end
    % Add requirements for initial and final velocity
    uTP  = [uSP, uTP, uEP]; up0 = [vSP/v(uSP), up0, vEP/v(uEP)];
end

Ts = 0.001; % Simulation sampling time
N = length(uTP); ts = cell(1,N); us = cell(1,N); ups = cell(1,N);  
for i = 1:N % Loop through all turn points
    uB = uTP(i); upB = up0(i); tB = 0;
    uF = uTP(i); upF = up0(i); tF = 0;
    uBs =[]; upBs = []; tBs = []; uFs = []; upFs =[]; tFs = []; % Storage
    goB = true; goF = true;
    
    while goB || goF
        % Integrate back from the turn point
        if uB > uSP && goB
            dxT = dx(uB);   dyT = dy(uB);
            ddxT = ddx(uB); ddyT = ddy(uB);
            vT = v(uB)*upB; wT = w(uB)*upB; kappaT = kappa(uB);
            arT = vT*wT; atT = atMax*sqrt(1 - (arT/arMax)^2);
            
            if velCnstr && abs(vT) > vMax
                upB = vMax/v(uB); upp = 0;
            elseif abs(arT)-arMax > 0.001
                arT = arMax; atT = 0; upp = 0; goB = false;
            else
                atT = -real(atT);
                upp = real(-atMax*sqrt(1/(dxT^2 + dyT^2) - ...
                    (dxT^2 + dyT^2)*kappaT^2*upB^4/arMax^2) - ...
                    (dxT*ddxT + dyT*ddyT)/(dxT^2 + dyT^2) * upB^2);
            end
            
            uBs = [uBs; uB]; upBs = [upBs; upB]; tBs = [tBs; tB]; % Store
            tB = tB + Ts;
            uB = uB - upB*Ts; % Euler integration
            upB = upB - upp*Ts; % Euler integration
        else
            goB = false;
        end
        
        % Integrate forward from the turn point
        if uF < uEP && goF
            dxT = dx(uF);   dyT = dy(uF);
            ddxT = ddx(uF); ddyT = ddy(uF);
            vT = v(uF)*upF; wT = w(uF)*upF; kappaT = kappa(uF);
            arT = vT*wT; atT = atMax*sqrt(1 - (arT/arMax)^2);
            
            if velCnstr && abs(vT) > vMax
                upF = vMax/v(uF); upp = 0;
            elseif abs(arT)-arMax > 0.001
                arT = arMax; atT = 0; upp = 0; goF = false;
            else
                atT = real(atT);
                upp = real(+atMax*sqrt(1/(dxT^2 + dyT^2) - ...
                    (dxT^2 + dyT^2)*kappaT^2*upF^4/arMax^2) - ...
                    (dxT*ddxT + dyT*ddyT)/(dxT^2 + dyT^2) * upF^2);
            end
            
            uFs = [uFs; uF]; upFs = [upFs; upF]; tFs = [tFs; tF]; % Store
            tF = tF + Ts;
            uF = uF + upF*Ts; % Euler integration
            upF = upF + upp*Ts; % Euler integration
        else
            goF = false;
        end
    end

    ts{i} = [tBs; tB+tFs(2:end)];
    us{i} = [flipud(uBs); uFs(2:end)];
    ups{i} = [flipud(upBs); upFs(2:end)];
end

OFig(1.5,1,1); xlabel('$u~[\mathrm{1}]$'); ylabel('$\dot{u}~[\mathrm{1/s}]$'); xlim([uSP, uEP]);
for i = 1:N
    plot(us{i}, ups{i}, 'k-', uTP(i), up0(i), 'ko');
    if velCnstr && i==1
        text(uTP(i)+0.08, up0(i)+0.04, 10, '$SP$', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    elseif velCnstr && i==N
        text(uTP(i)-0.08, up0(i)+0.04, 10, '$EP$', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
    else
        text(uTP(i), up0(i)+0.04, 10, sprintf('$TP_%d$', i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    
end

% Find minimum of all profiles ups (shedule derivative)
usOrig = us;
for i = 1:N-1
    d = ups{i+1} - interp1(us{i}, ups{i}, us{i+1});
    j = find(d(1:end-1).*d(2:end)<0, 1); % Where ups{i} is approx. ups{i+1}
    % Find more exact u where profiles ups{i} and ups{i+1} are equal
    uj = us{i+1}(j) + (us{i+1}(j+1)-us{i+1}(j))/(d(j+1)-d(j))*(0-d(j));
    rob = interp1(us{i}, ups{i}, uj);
    
    keep = us{i} < uj;
    us{i} = [us{i}(keep); uj];     ups{i} = [ups{i}(keep); rob];
    keep = us{i+1} > uj;
    us{i+1} = [uj; us{i+1}(keep)]; ups{i+1} = [rob; ups{i+1}(keep)];
end

% Construct final solution profile
tt = interp1(usOrig{1}, ts{1}, us{1}); uu = us{1}; uup = ups{1};
for i = 2:N
    ti = interp1(usOrig{i},ts{i},us{i});
    tt = [tt; ti + tt(end) - ti(1)];
    uu = [uu; us{i} + uu(end) - us{i}(1)];
    uup = [uup; ups{i}];
end
vv = v(uu).*uup;

plot(uu, uup, 'b-', 'linewidth', 2);

OFig(1.5,1,1); xlabel('$t~[\mathrm{s}]$'); ylabel('$u~[\mathrm{1}]$'); xlim([min(tt), max(tt)]);
plot(tt, uu, 'b-');

hFig = OFig(2,1); hFig.xlabel({'$t~[\mathrm{s}]$', '$u~[\mathrm{1}]$'}); hFig.ylabel('$v~[\mathrm{m/s}]$');
hFig.axes(1); xlim([min(tt), max(tt)]); plot(tt, vv, 'b-');
hFig.axes(2); xlim([uSP, uEP]); plot(uu, vv, 'b-');