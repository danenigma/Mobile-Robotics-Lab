classdef trajectoryFollower < handle
    %pose A layer over a column vector that provides access methods and
    % associated homogeneous transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered to be that of frame b
    % relative to frame a.
    
    properties(Constant)

    end
    
    
    properties(Access = public)
    numSamples;
    init_dist;
    init_pose;
    dt;
    refControl;
    velArray  = [];
    wArray    = [];
    poseArray = [];
    timeArray = [];
    distArray = [];
    xArray = [];
    yArray = [];
    thArray = [];
    
    end
            
    methods(Access = public)
        
        function obj = trajectoryFollower()

        end
        function pos = getPoseAtTime(obj,t)
            t_dt = t/obj.dt;
            x  = interp1(obj.xArray,t_dt);
            y  = interp1(obj.yArray,t_dt);
            th = interp1(obj.thArray,t_dt);            
            pos = pose(x, y, th);            
        end

    end
end