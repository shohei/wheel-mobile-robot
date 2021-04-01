classdef OWmr < OObject
    properties (Access = 'protected')
        oWmr;
        oWmrAxis;
        oWmrWheelL;
        oWmrWheelR;
        oWmrBody;
        oWmrDir;
        oParticles;
        oParticleDirs;
    end
    
    methods
        function obj = OWmr()
            obj = obj@OObject;
            obj.oWmr = hggroup('Parent', obj.o);
            obj.oWmrAxis = line([0, 0], nan(1,2), ones(1,2)*0.1, ...
                'LineWidth', 2, 'Parent', obj.oWmr);
            obj.oWmrWheelL = patch(nan(1,5), nan(1,5), ones(1,5)*0.2, ...
                'k', 'Parent', obj.oWmr);
            obj.oWmrWheelR = patch(nan(1,5), nan(1,5), ones(1,5)*0.2, ...
                'k', 'Parent', obj.oWmr);
            obj.oWmrBody = patch(nan(1,5), nan(1,5), ones(1,5)*0.3, ...
                'r', 'Parent', obj.oWmr);
            obj.oWmrDir = line(nan(1,2), [0, 0], ones(1,2)*0.4, ...
                'LineWidth', 2, 'Parent', obj.oWmr);
            obj.oParticles = line(nan, nan, 10+0.1, 'LineStyle', 'none', 'Parent', ancestor(obj.o, 'axes'));
            obj.oParticleDirs = line(nan, nan, 10+0.1, 'Parent', ancestor(obj.o, 'axes'));
        end
        
        function showWmr(obj, show, L, r, dr)
            OObject.showObj(obj.oWmrBody, show, 'sfm');
            if nargin<3 || isempty(L)
                L = 0.7;
            end
            if nargin<4 || isempty(r)
                r = 0.15;
            end
            if nargin<5 || isempty(dr)
                dr = 0.03;
            end
            set(obj.oWmrAxis, 'YData', [0.5, -0.5]*L);
            set(obj.oWmrWheelL, ...
                'XData', 2*r*[-0.5, 0.5, 0.5, -0.5, -0.5], ...
                'YData', 0.5*L+dr*[-1, -1, 1, 1, -1]);
            set(obj.oWmrWheelR, ...
                'XData', 2*r*[-0.5, 0.5, 0.5, -0.5, -0.5], ...
                'YData',-0.5*L+dr*[-1, -1, 1, 1, -1]);
            set(obj.oWmrBody, ...
                'XData', [-0.4, 0.4, 0.4,-0.4,-0.4]*L, ...
                'YData', [-0.4,-0.4, 0.4, 0.4,-0.4]*L);
            set(obj.oWmrDir, 'XData', [0.4, 1]*L);
        end
        
        function showParticles(obj, show)
            OObject.showObj(obj.oParticles, show, 'cm');
            OObject.showObj(obj.oParticleDirs, show, 'sc');
        end
        
        function updateParticles(obj, q)
            n = size(q,2);
            r = 0.2;
            set(obj.oParticles, ...
                'XData', q(1,:), ...
                'YData', q(2,:), ...
                'ZData', 10+0.1*ones(1, n));
            set(obj.oParticleDirs, ...
                'XData', reshape([q(1,:); q(1,:)+r*cos(q(3,:)); nan(1,n)], 1, []), ...
                'YData', reshape([q(2,:); q(2,:)+r*sin(q(3,:)); nan(1,n)], 1, []), ...
                'ZData', reshape([10+0.1*ones(1, n); 10+0.1*ones(1, n); 10+0.1*ones(1, n)], 1, []));
        end
    end
end