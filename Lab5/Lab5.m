%% figure 8 oop

close all; clear;clc;
robot = raspbot();

Ks = 3.;
Kv = 1.;
tPause = 0.0;
fig8Traj = figure8ReferenceControl(Ks, Kv, tPause);
robMdl = robotModel();
tf = fig8Traj.getTrajectoryDuration();

firstIteration = false;

while true
    
    if(firstIteration== false)
    startTic = tic();
    timePrev = toc(startTic);
    firstIteration= true;
    end
    timeNow = toc(startTic);
    if timeNow > tf
        robot.stop()
        break;
    end
    dt = timeNow - timePrev;
    [V, w] = fig8Traj.computeControl(timeNow);
    [vl , vr] = robMdl.VwTovlvr(V, w);
    robot.sendVelocity(vl, vr);
    pause(0.005);
    
end