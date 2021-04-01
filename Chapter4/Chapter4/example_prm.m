D = 1; % Distance parameter
maxIter = 200;
M = []; % Map

hFig = OFig(1.5, 2, 1); hFig.sketch(); axis square equal; axis([0, 10, 0, 10]);

j = 1;
while j <= maxIter
    xRand = 10*rand(1,2); % Random configuration
    M = [M; xRand];
    con = []; % Connectins
    for i = 1:size(M,1) % Search connections to the neighbour nodes
        d = norm(M(i,:)-xRand);
        if d<D && d>eps % Add connections from xRand to neighbor
            con = [con; xRand, M(i,:)];
        end
    end
    j = j + 1;

    line(xRand(1), xRand(2), 'Color', 'r', 'Marker', '.');
    for i = 1:size(con,1)
        line(con(i,[1,3]), con(i,[2,4]), 'Color', 'b');
    end
end