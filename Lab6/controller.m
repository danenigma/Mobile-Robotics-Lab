classdef controller < handle
    properties(Constant)

    end
    
    properties(Access = public)
    traj;
    p_gain;
    d_gain;
    i_gain;
    controlMat;
    tau;
    mode;
    prevError;
    integralError;
    t_prev;
    end
            
    methods(Access = public)
        
        function obj = controller(traj, tau, mode)
            obj.traj = traj;
            obj.mode = mode;
            obj.tau = tau;
            kx = 1/tau;
            ky = 2/(tau^2*abs(traj.refControl.getAbsV()));
            kth = 1/tau;
            obj.controlMat = [kx, 0, 0; 0, ky, kth];
            obj.prevError = zeros(3,1);
            obj.integralError = zeros(3,1);
            obj.t_prev = 0.0;
            
        end
        function [V, w, error] = pidCorrect(obj, t, actualPose)
            refPose = obj.traj.getPoseAtTime(t);
            actualPoseVec  = actualPose.getPoseVec();
            actTh = actualPoseVec(3);
            refPoseVec  = refPose.getPoseVec();
            refth = refPoseVec(3);
            
            refPoseVec(3) = 1;
            error = actualPose.aToB()*refPoseVec;
            errorPos = error(1:2);
            errorTh  = refth - actTh; 
            errorTh  = atan2(sin(errorTh), cos(errorTh));
            
            error = [errorPos', errorTh]';
            dt = t-obj.t_prev;
            obj.t_prev = t;
            errorDervative = (error-obj.prevError)/dt;
            obj.prevError  = error;
            
            obj.integralError = obj.integralError + error;
            
            feedBackControl = obj.controlMat*error;
            
            [Vf, wf] = obj.traj.getRefVelAtTime(t);
            
            finalControl = [Vf; wf] + obj.mode*feedBackControl;
            
            V = finalControl(1);
            w = finalControl(2);
            
        end

    end
end