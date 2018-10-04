classdef figure8ReferenceControl < handle
    %robotModel A convenience class for storing robot physical 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the class name with robotModel.W2 for
    % example because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are referenced from the class name as well.
    properties(Constant)
    end
    properties(Access = private)
        Ks;
        Kv;
        tPause;
        v;
        sf;
        kk;
        kth;
    end
    
    methods(Access = public)
        
        function obj = figure8ReferenceControl(Ks, Kv, tPause)
            obj.Ks = Ks;
            obj.Kv = Kv;
            obj.tPause = tPause;
            obj.v  = 0.2;
            obj.sf = 1;
            obj.kk = 15.1084;
            obj.kth = 2*pi/obj.sf;
        end
        function [V, w] = computeControl(obj,timeNow)
               if timeNow < 0
               V = 0;
               w = 0;
               else
               t = (obj.Kv/obj.Ks)*timeNow;
               s = obj.v*t;
               k = (obj.kk/obj.Ks)*sin(obj.kth*s);
               V = obj.Kv * obj.v;
               w = k*V;
               end
        end
        function duration = getTrajectoryDuration(obj)
        % Return the total time required for motion and for the
        % initial and terminal pauses.
        duration = (obj.Ks/obj.Kv)*(obj.sf/obj.v);
        end
        function v = getAbsV(obj)
        v = abs(obj.v);
        end
    end
end