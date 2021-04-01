xi = [5, 5]; % Initial configuration
D = 0.2; % Distance to new node
maxIter = 1000;
M = [xi]; % Map

hFig = OFig(2, 3, 1); hFig.sketch(); axis square equal; axis([0, 10, 0, 10]);

j = 1;
while j < maxIter
    xRand = 10*rand(1,2); % Random configuration
    dMin = 100; iMin = 1; % Search for the closest point in the map M
    for i = 1:size(M,1)
        d = norm(M(i,:)-xRand);
        if d<dMin
            dMin = d;
            iMin = i;
        end
    end

    xNear = M(iMin,:);
    v = xRand - xNear;
    xNew = xNear + v/norm(v)*D; % Calculate new point

    con = [xNear; xNew];
    M = [M; xNew];
    j = j + 1;
    
    line(con(:,1), con(:,2), 'Color', 'b');
end