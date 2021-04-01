classdef OMarker < OObject
    properties (Access = 'protected')
        oMarker;
    end
    
    methods
        function obj = OMarker()
            obj = obj@OObject;
            obj.oMarker = patch([0 0], [nan nan], ones(1,2)*0.1, 'y', 'Parent', obj.o);
        end
        
        function showMarker(obj, show, r)
            OObject.showObj(obj.oMarker, show, 'sfm');
            if nargin<3 || isempty(r)
                r = 0.3;
            end
            t = linspace(0, 2*pi, 49);
            set(obj.oMarker, ...
                'XData', r*cos(t), ...
                'YData', r*sin(t), ...
                'ZData', 0.1*ones(size(t)));
        end
    end
end