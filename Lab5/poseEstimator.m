classdef poseEstimator < handle
    %pose A layer over a column vector that provides access methods and
    % associated homogeneous transforms. For the purpose of naming the
    % homogeneous transforms, the pose is considered to be that of frame b
    % relative to frame a.
    
    properties(Constant)

    end
    
    
    properties(Access = public)

        initPose;
        robot;
        robotMdl;
        sl;
        sr;
        sl_bias;
        sr_bias;
        x;y;th;
    end
            
    methods(Access = public)
        
        function obj = poseEstimator(robot, robotMdl, initPose, sl_bias, sr_bias)
            obj.robot = robot;
            obj.robotMdl = robotMdl;
            obj.initPose = initPose;
            obj.sl = 0;
            obj.sr = 0;
            obj.sl_bias = sl_bias;
            obj.sr_bias = sr_bias;
            initPoseVec =  initPose.getPoseVec();
            obj.x = initPoseVec(1);
            obj.y = initPoseVec(2);
            obj.th = initPoseVec(3);
            
        end
        function currentPose = update(obj)
            dsl = obj.robot.encoders.LatestMessage.Vector.X - obj.sl - obj.sl_bias;
            dsr = obj.robot.encoders.LatestMessage.Vector.Y - obj.sr - obj.sr_bias;
            obj.sl  = obj.sl + dsl;
            obj.sr  = obj.sr + dsr;
            ds  = (dsl  + dsr)/2;
            dth = (dsr  - dsl)/obj.robotMdl.my_W;
            obj.th =  obj.th + dth;    
            obj.x  =  obj.x  + cos(obj.th)*ds;
            obj.y  =  obj.y  + sin(obj.th)*ds;
            currentPose =  pose(obj.x, obj.y, obj.th);
            
        end    
    end
end