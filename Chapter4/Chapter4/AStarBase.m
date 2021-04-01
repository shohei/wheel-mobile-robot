classdef AStarBase < handle
properties
    map = []; % Map: 0 - free space, 1 - obstacles
    open = []; closed = []; start = []; goal = []; act = []; path = [];
    hFig = [];
    hMap;
    hPath;
    showMode = 0;
    showTs = 0.1;
end
    
methods
    function path = find(obj, start, goal) % start=[is; js], goal=[ig; jg]
        obj.start = start; obj.goal = goal; obj.path = [];
        obj.closed = []; % Empty closed list
        obj.open = [];
        obj.update();
        obj.open = struct('id', start, 'src', [0; 0], 'g', 0, ...
                          'h', obj.heuristic(start)); % Initial open list

        if obj.map(start(1), start(2))~=0 || obj.map(goal(1), goal(2))~=0
            path = []; return; % Path not feasible!
        end
        
        while true % Search loop
            obj.update();
            
            if isempty(obj.open), break; end % No path found :(
            
            obj.act = obj.open(1); % Get node from the ordered open list,
            obj.closed = [obj.closed, obj.act]; % add it to the closed list
            obj.open = obj.open(2:end); % and remove it from the open list.

            if obj.act.id(1)==obj.goal(1) && obj.act.id(2)==obj.goal(2)
                % Path found :) Get the path from the closed list ...
                p = obj.act.id; obj.path = [p]; ids = [obj.closed.id];
                while sum(abs(p-start))~=0 % Follow src nodes to the start
                    p = obj.closed(ids(1,:)==p(1) & ids(2,:)==p(2)).src;
                    obj.path = [p, obj.path];
                end
                break;
            end
            
            neighbours = obj.getNodeNeighbours(obj.act.id);
            for i = 1:size(neighbours, 2)
                n = neighbours(:,i);
                % Add neighbour to the open list if it is not in the closed
                % list and it is not an obstacle.
                ids = [obj.closed.id]; z = ids(1,:)==n(1) & ids(2,:)==n(2);
                if isempty(find(z, 1)) && ~obj.map(n(1), n(2))
                    obj.addNodeToOpenList(n);
                end
            end
        end
        
        obj.update();
        path = obj.path;
    end

    function addNodeToOpenList(obj, i)
        g = obj.act.g + obj.cost(i); % Path cost
        % Check if the node is in already the open list
        ids = [obj.open.id]; s = [];
        if ~isempty(ids)
            s = sum(abs(ids-repmat(i, 1, size(ids, 2))))==0;
        end
        if isempty(find(s, 1)) % Add new node to the open list
            node = struct('id', i, 'src', obj.act.id, ...
                          'g', g, 'h', obj.heuristic(i));
            obj.open = [obj.open, node];
        else % Update the node in the open list if it has better score
            if g<obj.open(s).g
                obj.open(s).g = g;
                obj.open(s).src = obj.act.id;
            end
        end
        % Sort open list
        [~,i] = sortrows([[obj.open.g]+[obj.open.h]; obj.open.h].', [1,2]);
        obj.open = obj.open(i);
    end

    function n = getNodeNeighbours(obj, a)
        n = [a(1)-1, a(1), a(1), a(1)+1; a(2), a(2)-1, a(2)+1, a(2)];
        [h, w] = size(obj.map);
        n = n(:, n(1,:)>=1 & n(1,:)<=h & n(2,:)>=1 & n(2,:)<=w); % Bounds
    end
    
    function g = cost(obj, a)
        g = sum(abs(a-obj.act.id)); % Manhattan distance
    end

    function h = heuristic(obj, a)
        h = sqrt(sum((a-obj.goal).^2)); % Euclidean distance
    end
    
    function update(obj)
        if obj.showMode==0, return; end
        if isempty(obj.hFig)
            obj.hFig = OFig(2, 2, 1);
            hAxe = obj.hFig.axes(); hold on;
            set(hAxe, 'XTick', [], 'YTick', [], 'YDir', 'reverse', 'Position', [0.05 0.05, 0.9 0.9]);
            [h, w] = size(obj.map);
            obj.hMap = cell(h, w);
            for i=1:h
                for j=1:w
                    c = 'w';
                    if obj.map(i,j)~=0
                        c = 'k';
                    end
                    p.tile = patch([-0.5 0.5 0.5 -0.5 -0.5]+j, [-0.5 -0.5 0.5 0.5 -0.5]+i, c, 'EdgeColor', ones(1,3)*0);
                    obj.hMap{i,j} = p;
                end
            end
            obj.hPath = line(nan, nan, 'Color', [1, 0.5, 0.5], 'LineStyle', '-', 'LineWidth', 3);
            for i=1:h
                for j=1:w
                    p = obj.hMap{i,j};
                    if obj.showMode>1
                        p.line = line(nan, nan, 'Color', [1 1 1], 'LineWidth', 2);
                        p.dir = text(0.45+j, 0.45+i, '', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'FontWeight', 'bold');
                    end
                    if obj.showMode>2
                        p.g = text(-0.45+j, -0.15+i, '', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 8);
                        p.h = text(-0.45+j, 0.5+i, '', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 8);
                        p.f = text(j, i, '', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 10);
                        p.i = text(0.45+j, -0.15+i, '', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 8);
                        if i==obj.start(1) && j==obj.start(2)
                            set(p.i, 'String', 'S');
                        elseif i==obj.goal(1) && j==obj.goal(2)
                            set(p.i, 'String', 'G');
                        end
                    end
                    obj.hMap{i,j} = p;
                end
            end
            axis equal; axis([0.5 w+0.5 0.5 h+0.5]);
        end

        lists = {obj.closed, obj.open};
        for l=1:length(lists)
            list = lists{l};
            for k=1:length(list)
                node = list(k); ij = node.id; i = ij(1); j = ij(2);
                g = node.g;
                h = node.h;
                if obj.showMode>1 && node.src(1)~=0 && node.src(2)~=0
                    dm = ij-node.src;
                    dd = '';
                    if dm(1)==1 && dm(2)==0
                        dd = '$\uparrow$';
                    elseif dm(1)==-1 && dm(2)==0
                        dd = '$\downarrow$';
                    elseif dm(1)==0 && dm(2)==1
                        dd = '$\leftarrow$';
                    elseif dm(1)==0 && dm(2)==-1
                        dd = '$\rightarrow$';
                    elseif dm(1)==1 && dm(2)==1
                        dd = '$\nwarrow$';
                    elseif dm(1)==1 && dm(2)==-1
                        dd = '$\nearrow$';
                    elseif dm(1)==-1 && dm(2)==1
                        dd = '$\swarrow$';
                    elseif dm(1)==-1 && dm(2)==-1
                        dd = '$\searrow$';
                    end
                    set(obj.hMap{i,j}.dir, 'String', dd);
                end
                if obj.showMode>2
                    set(obj.hMap{i,j}.g, 'String', sprintf('%.2g', g));
                    set(obj.hMap{i,j}.h, 'String', sprintf('%.2g', h));
                    set(obj.hMap{i,j}.f, 'String', sprintf('%.2g', g+h));
                end
                if ~isempty(obj.act) && ij(1)==obj.act.id(1) && ij(2)==obj.act.id(2)
                    c = [1, 0.5, 0.5];
                elseif l==1
                    c = [1, 0.8, 0.8];
                else
                    c = [0.9, 1, 1];
                end
                set(obj.hMap{i,j}.tile, 'FaceColor', c);
            end
        end

        if ~isempty(obj.path)
%             set(obj.hMap{obj.path(1,1),obj.path(2,1)}.tile, 'FaceColor', 'r');
%             set(obj.hMap{obj.path(1,end),obj.path(2,end)}.tile, 'FaceColor', 'g');
            set(obj.hPath, 'XData', obj.path(2,:), 'YData', obj.path(1,:));
        end
        OFig.pause(obj.showTs);
    end
end
end