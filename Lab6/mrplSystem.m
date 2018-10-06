classdef mrplSystem < handle
     
    properties(Constant)
    NUM_SAMPLES = 4001; 
    end
    properties(Access = public)
    robot;
    end
    methods(Access = public)
        
        function obj = mrplSystem(robot)
            obj.robot = robot;
        end
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn)
            curve = cubicSpiralTrajectory.planTrajectory(x,y,th,sgn);
            curve.planVelocities(robotModel.maxWheelVelocity-0.1); 
            obj.executeTrajectory(curve);
        end
        function executeTrajectory(obj, curve)
            firstIteration = false;
            tf  = curve.getTrajectoryDuration();
            while true
                if(firstIteration== false)
                    startTic = tic();
                    timePrev = toc(startTic);
                    firstIteration= true;
                end
                timeNow = toc(startTic);
                dt = timeNow - timePrev;
                timePrev = timeNow;
                if timeNow > tf
                    obj.robot.stop()
                    break;
                end
                V  = curve.getVAtTime(timeNow);
                w  = curve.getwAtTime(timeNow);
                [vl , vr] = robotModel.My_VwTovlvr(V, w);
                obj.robot.sendVelocity(vl, vr);
                pause(0.005);
            end
            %obj.robot.stop();
        end 
    end
end