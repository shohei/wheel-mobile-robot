classdef OObject < handle
    properties (Access = 'protected')
        o;
        oAxes;
        oAxesX;
        oAxesY;
        oLabel;
        oPath;
        oRange;
        oDistance;
        oAngle;
        oAngel;
        absAngle;
        count;
        pPathTop = false;
    end
    
    methods
        function obj = OObject()
            obj.count = length(findobj(gcf, 'Type', 'hgtransform', 'Tag', 'OObject'));
            obj.o = hgtransform('UserData', obj, 'Tag', 'OObject', 'Matrix',  makehgtform('translate', [0, 0, obj.count]));
            obj.oAxes = hggroup('Parent', obj.o, 'Visible', 'off');
            obj.oPath = line(nan, nan, nan, 'Visible', 'off', ...
                'Parent', ancestor(obj.o, 'axes'));
            obj.oRange = line(nan, nan, nan, 'Color', 'm', ...
                'LineStyle', ':', 'Visible', 'off', 'Parent', obj.o);
            obj.oDistance = line(nan, nan, nan, 'Color', 'm', ...
                'LineStyle', '-', 'Visible', 'off', 'Parent', obj.o);
            obj.oAngle = patch([0 0], [nan nan], -60.2*ones(1,2), 'w', 'Visible', 'off', ...
                'Parent', obj.o, 'LineStyle', 'none'); % 'FaceAlpha', 0.1,
            obj.oAngel = line(nan, nan, -60.1, 'Visible', 'off', ...
                'Parent', obj.o);
            obj.oAxesX = line([0, 1], [0, 0], [-20, -20], 'Color', 'r', ...
                'LineWidth', 2, 'Parent', obj.oAxes);
            obj.oAxesY = line([0, 0], [0, 1], [-20, -20], 'Color', 'g', ...
                'LineWidth', 2, 'Parent', obj.oAxes);
            obj.oLabel = text(0, 0, 20, '', 'Parent', obj.o, ...
                'HorizontalAlignment', 'center');
            obj.absAngle = true;
        end
        
        function setParent(obj, parent)
            set(obj.o, 'Parent', parent.o);
        end
        
        function setPose(obj, q)
            if length(q)<3
                q(3) = 0;
            end
            set(obj.o, 'Matrix', makehgtform( ...
                'translate', [q(1), q(2), obj.count], ...
                'zrotate', q(3)));
            
            os = findobj(obj.o, 'Type', 'hgtransform', 'Tag', 'OObject');
            for i=1:length(os)
                o = get(os(i), 'UserData');
                o.updatePath();
            end
        end
        
        function updatePath(obj)
            if strcmp(get(obj.oPath, 'Visible'), 'on')
                q = obj.getAbsPose();
                set(obj.oPath, ...
                    'XData', [get(obj.oPath, 'XData'), q(1)], ...
                    'YData', [get(obj.oPath, 'YData'), q(2)], ...
                    'ZData', [get(obj.oPath, 'ZData'), obj.pPathTop*30-20+0.1]);
            end
        end
        
        function showAxes(obj, size, lw)
            if nargin<2 || isempty(size)
                size = 1;
            end
            if nargin<3 || isempty(lw)
                lw = 2;
            end
            if size<eps
                set(obj.oAxes, 'Visible', 'off');
            else
                set(obj.oAxes, 'Visible', 'on');
                set(obj.oAxesX, 'XData', [0, size], 'LineWidth', lw);
                set(obj.oAxesY, 'YData', [0, size], 'LineWidth', lw);
            end
        end
        
        function setTxt(obj, label, r, fi, align)
            obj.setTex(label, r, fi, align);
            set(obj.oLabel, 'Interpreter', 'none');
        end
        
        function setTex(obj, label, r, fi, align)
            if nargin<3 || isempty(r)
                r = 0;
            end
            if nargin<4 || isempty(fi)
                fi = 0;
            end
            if nargin<5 || isempty(align)
                align = '';
            end
            set(obj.oLabel, 'String', label);
            obj.setTPos(r, fi, align, true);
        end
        
        function setTPos(obj, r, fi, align, force)
            if nargin<2 || isempty(r)
                r = 0;
            end
            if nargin<3 || isempty(fi)
                fi = 0;
            end
            if nargin<4 || isempty(align)
                align = '';
            end
            if nargin<5 || isempty(force)
                force = false;
            end
            if ~isempty(align) || force
                ha = 'center';
                va = 'middle';
                for i=1:length(align)
                    if strcmp(align(i), 'l')
                        ha = 'left';
                    elseif strcmp(align(i), 'c')
                        ha = 'center';
                    elseif strcmp(align(i), 'r')
                        ha = 'right';
                    elseif strcmp(align(i), 't')
                        va = 'top';
                    elseif strcmp(align(i), 'm')
                        va = 'middle';
                    elseif strcmp(align(i), 'b')
                        va = 'bottom';
                    end
                end
                set(obj.oLabel, ...
                    'HorizontalAlignment', ha, 'VerticalAlignment', va);
            end
            set(obj.oLabel, 'Position', [r*cos(fi), r*sin(fi), 20]);
        end
        
        function showPath(obj, show, top)
            if nargin<3 || isempty(top)
                top = false;
            end
            obj.pPathTop = top;
            OObject.showObj(obj.oPath, show);
        end
        
        function showAngle(obj, show, rel)
            if nargin<3 || isempty(rel)
                rel = false;
            end
            obj.absAngle = ~rel;
            OObject.showObj(obj.oAngle, show, 'f', 0.1);
            OObject.showObj(obj.oAngel, show);
        end
        
        function showRange(obj, show)
            OObject.showObj(obj.oRange, show);
        end
        
        function showDistance(obj, show)
            OObject.showObj(obj.oDistance, show);
        end
        
        function updateRange(obj, p)
            if isobject(p) && isa(p, 'OObject')
                p = p.getAbsPose();
            end
            p0 = obj.getAbsPose();
            dp = p(1:2)-p0(1:2);
            r = sqrt(dp.'*dp);
            fi = wrapToPi(atan2(dp(2), dp(1))-p0(3));
            t = linspace(0, 2*pi, 96+1);
            a = 0.7;
            if ~obj.absAngle
                xi = fi;
            else
                xi = -p0(3);
            end
            u = sign(xi)*linspace(0, abs(xi), ceil(48/pi*abs(xi)));
            if strcmp(get(obj.oRange, 'Visible'), 'on')
                set(obj.oRange, ...
                    'XData', r*cos(t), ...
                    'YData', r*sin(t), ...
                    'ZData', -40.1*ones(size(t)));
            end
            if strcmp(get(obj.oDistance, 'Visible'), 'on')
                set(obj.oDistance, ...
                    'XData', [0, r*cos(fi)], ...
                    'YData', [0, r*sin(fi)], ...
                    'ZData', -40.2*ones(1,2));
            end
            if strcmp(get(obj.oAngle, 'Visible'), 'on')
                set(obj.oAngle, ...
                    'XData', [0, a*cos(u), 0], ...
                    'YData', [0, a*sin(u), 0], ...
                    'ZData', -60.2*ones(1,length(u)+2));
            end
            if strcmp(get(obj.oAngel, 'Visible'), 'on')
                set(obj.oAngel, ...
                    'XData', a*cos(u), ...
                    'YData', a*sin(u), ...
                    'ZData', -60.1*ones(1,length(u)));
            end
        end
        
        function T = getAbsTransform(obj)
            o = obj.o;
            T = get(o, 'Matrix');
            p = get(o, 'Parent');
            while p~=0 && ~strcmp(get(p, 'Type'), 'axes')
                o = p;
                if strcmp(get(p, 'Type'), 'hgtransform')
                    T = get(o, 'Matrix')*T;
                end
            end
        end
        
        function q = getAbsPose(obj, vct)
            if nargin<2 || isempty(vct)
                vct = [1; 0];
            end
            p = obj.getAbsTransform()*[0 vct(1); 0 vct(2); 0 0; 1 1];
            q = [p(1,1); p(2,1); atan2(p(2,2)-p(2,1), p(1,2)-p(1,1))];
        end
    end
    
    methods (Static)
        function showObj(h, show, mode, desat)
            if nargin<3 || isempty(mode)
                mode = 'scm';
            end
            if nargin<4 || isempty(desat)
                desat = 0.5;
            end
            if nargin<2 || isempty(show)
                set(h, 'Visible', 'off');
            else
                [s, c, m] = colstyle(show);
                if isempty(s), s = 'none'; end
                if isempty(c), c = 'none'; end
                if isempty(m), m = 'none'; end
                if strfind(mode, 's')
                    set(h, 'LineStyle', s);
                end
                if strfind(mode, 'c')
                    set(h, 'Color', c);
                end
                if strfind(mode, 'f')
                    set(h, 'FaceColor', OObject.desatColor(c, desat));
                end
                if strfind(mode, 'e')
                    set(h, 'EdgeColor', c);
                end
                if strfind(mode, 'm')
                    set(h, 'Marker', m);
                end
                set(h, 'Visible', 'on');
            end
        end
        
        function out = desatColor(c, s)
            if nargin<2 || isempty(s)
                s = 0.5;
            end
            c = OObject.parseColor(c);
            c = rgb2hsv(c);
            c(2) = c(2)*s;
            out = hsv2rgb(c);
        end
        
        function out = parseColor(c)
            out = [0 0 0];
            if ~ischar(c) && length(c)==3
                out = c;
            else
                if strcmp(c, 'y')
                    out = [1 1 0];
                elseif strcmp(c, 'm')
                    out = [1 0 1];
                elseif strcmp(c, 'c')
                    out = [0 1 1];
                elseif strcmp(c, 'r')
                    out = [1 0 0];
                elseif strcmp(c, 'g')
                    out = [0 1 0];
                elseif strcmp(c, 'b')
                    out = [0 0 1];
                elseif strcmp(c, 'w')
                    out = [1 1 1];
                end
            end
        end
    end
end