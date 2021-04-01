classdef OSig < handle
    properties (Access = 'protected')
        oFig;
        oSignals;
    end
    
    methods
        function obj = OSig(m, n)
            if nargin<1
                m = [];
            end
            if nargin<2
                n = [];
            end
            obj.oFig = OFig(m, n, 1);
        end
        
        function fin(obj, xy)
            if nargin<2
                xy = [];
            end
            ax = obj.oFig.getAxes();
            linkaxes(ax, 'x');
            n = length(xy);
            lims = zeros(n,2);
            if n>1
                for i=xy
                    lims(i,:) = get(ax(i), 'YLim');
                end
                m = [min(lims(:,1)), max(lims(:,2))];
                for i=xy
                    set(ax(i), 'YLim', m);
                end
                linkaxes(ax(xy), 'y');
            end
            t = get(obj.oSignals(1,1), 'XData');
            xlim(ax(1), [0, max(t)]);
        end
        
        function axes(obj, t, varargin)
            n = length(varargin);
            obj.oFig.layout(obj.oFig.mnk(1), obj.oFig.mnk(2), n);
            obj.oFig.xlabel(t);
            obj.oFig.ylabel(varargin);
        end
        
        function sig(obj, varargin)
            n = length(obj.oFig.getAxes());
            p = length(varargin);
            for j = 1:p
                [s, c, m] = colstyle(varargin{j});
                if isempty(s), s = 'none'; end
                if isempty(c), c = 'none'; end
                if isempty(m), m = 'none'; end
                sigs = zeros(n,1);
                for i = 1:n
                    sigs(i) = line(nan, nan, 'Parent', obj.oFig.axes(i));
                    set(sigs(i), 'LineStyle', s, 'Color', c, 'Marker', m);
                end
                obj.oSignals = [obj.oSignals, sigs];
            end
        end
        
        function plot(obj, t, varargin)
            p = min([length(varargin), size(obj.oSignals, 2)]);
            for j = 1:p
                y = varargin{j};
                n = min([length(y), size(obj.oSignals, 1)]);
                for i = 1:n
                    set(obj.oSignals(i,j), ...
                        'XData', [get(obj.oSignals(i,j), 'XData'), t], ...
                        'YData', [get(obj.oSignals(i,j), 'YData'), y(i)]);
                end
            end
        end
    end
end