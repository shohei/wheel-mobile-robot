classdef OFig < handle
    properties (GetAccess = 'public', SetAccess = 'protected')
        mnk = [1, 1, 1];
    end
    
    properties (Access = 'protected')
        paperSize = [4, 3]*33.3/25.4;
        screenSize = [768, 576]; %[768 576] [560 420] %
        margin = [0.12 0.12 0.06 0.06]; % left bottom right top
        hAxes = [];
        mode = '';
        hFig;
    end
    
    methods
        function obj = OFig(m, n, k)
            SPPI = get(0, 'ScreenPixelsPerInch');
            obj.screenSize = obj.paperSize*SPPI;
            if nargin<1
                m = [];
            end
            if nargin<2
                n = [];
            end
            if nargin<3
                k = [];
            end
            obj.hFig = figure('Units', 'normalized', ...
                'PaperUnits', 'inches', ...
                'IntegerHandle', 'on', ...
                'DefaultTextFontSize', 10, ...
                'DefaultTextInterpreter', 'latex');
            try
                set(obj.hFig, 'DefaultAxesTickLabelInterpreter', 'latex', ...
                    'DefaultColorbarTickLabelInterpreter', 'latex', ...
                    'DefaultLegendInterpreter', 'latex', ...
                    'DefaultTextarrowshapeInterpreter', 'latex', ...
                    'DefaultTextboxshapeInterpreter', 'latex');
            catch
            end
            p = get(obj.hFig, 'Position');
            set(obj.hFig, 'Position', [0.05 0.05 p(3:4)]);
            set(obj.hFig, 'Units', 'pixels');
            p = get(obj.hFig, 'Position');
            set(obj.hFig, ...
                'Position', [p(1:2) obj.screenSize], ...
                'PaperPosition', [0 0 obj.paperSize], ...
                'PaperSize', obj.paperSize);
            obj.layout(m, n, k);
            obj.axes();
        end
        
        function layout(obj, m, n, k)
            if isempty(m)
                m = 1;
            end
            if nargin<3 || isempty(n)
                n = 1;
            end
            if nargin<4 || isempty(k)
                k = n*m;
            end
            obj.mnk = [m, n, k];
            nn = min(k, n);
            mm = ceil(k/n);
            nm = [nn/n, mm/m];
            ps = obj.paperSize.*nm;
            ss = obj.screenSize.*nm;
            set(obj.hFig, 'Units', 'pixels', 'PaperUnits', 'inches');
            p = get(obj.hFig, 'Position');
            set(obj.hFig, 'Position', [p(1:2) ss], ...
                'PaperPosition', [0 0 ps], 'PaperSize', ps);
            
            gm = m/mm;
            gn = n/nn;
            for i=1:k
                x = mod(i-1, n);
                y = ceil(i/n)-1;
                x = x/nn + obj.margin(1)*gn;
                y = 1-(y+1)/mm + obj.margin(2)*gm;
                w = (1-sum(obj.margin([1,3]))*gn*nn)/nn;
                h = (1-sum(obj.margin([2,4]))*gm*mm)/mm;
                if i>length(obj.hAxes)
                    obj.hAxes(i) = axes('Parent', obj.hFig, ...
                        'Units', 'normalized', 'Box', 'on');
                end
                set(obj.hAxes(i), 'Position', [x y w h]);
                if strcmp(obj.mode, 'image')
                    set(obj.hAxes(i), 'XTick', [], 'YTick', [], 'YDir', 'reverse', 'XAxisLocation', 'top');
                    axis(obj.hAxes(i), 'image');
                elseif strcmp(obj.mode, 'image0')
                    bb = axis(obj.hAxes(i));
                    set(obj.hAxes(i), 'YDir', 'reverse', 'XAxisLocation', 'top', 'XLimMode', 'manual', 'YLimMode', 'manual', 'XTick', [bb(1), bb(2)], 'YTick', [bb(3), bb(4)]);
                    axis(obj.hAxes(i), 'image');
                    axis(obj.hAxes(i), bb+[-1 1 -1 1]*eps);
                elseif strcmp(obj.mode, 'sketch')
                    set(obj.hAxes(i), 'XTick', [], 'YTick', [], 'ZTick', [], 'XTickLabel', '', 'YTickLabel', '', 'ZTickLabel', '');
                    axis(obj.hAxes(i), 'equal');
                end
            end
        end
        
        function h = axes(obj, i)
            if nargin<2 || isempty(i)
                i = 1;
            end
            h = obj.hAxes(i);
            axes(h);
            hold(h, 'on');
        end
        
        function h = getAxes(obj)
            h = obj.hAxes;
        end
        
        function xlabel(obj, a, label)
            if nargin<3
                label = a;
                a = 1:length(obj.hAxes);
            end
            if ~iscell(label)
                l = label;
                label = cell(size(a));
                for i=1:length(a)
                    label{i} = l;
                end
            end
            for i=1:length(a)
                xlabel(obj.hAxes(a(i)), label{i});
            end
        end
        
        function ylabel(obj, a, label)
            if nargin<3
                label = a;
                a = 1:length(obj.hAxes);
            end
            if ~iscell(label)
                l = label;
                label = cell(size(a));
                for i=1:length(a)
                    label{i} = l;
                end
            end
            for i=1:length(a)
                ylabel(obj.hAxes(a(i)), label{i});
            end
        end
        
        function image(obj)
            obj.mode = 'image';
            obj.margin = [0.06 0.06 0.06 0.06];
            obj.layout(obj.mnk(1), obj.mnk(2), obj.mnk(3));
        end
        
        function image0(obj)
            obj.mode = 'image0';
            obj.margin = [0.12 0.06 0.06 0.12];
            obj.layout(obj.mnk(1), obj.mnk(2), obj.mnk(3));
        end
        
        function sketch(obj)
            obj.mode = 'sketch';
            obj.margin = [0.01 0.01 0.01 0.01];
            obj.layout(obj.mnk(1), obj.mnk(2), obj.mnk(3));
        end
    end
        
    methods (Static)
        function pause(dt)
            drawnow;
            if dt==0
                pause;
            else
                pause(dt);
            end
        end
    end
end