%% figure 8 oop

close all; clear;clc;
robot = raspbot('sim');

Ks = 3.;
Kv = 1.;
tPause = 0.0;
W = 0.09;
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
    [V, w] = fig8Traj.computeControl(timeNow);
    [vl , vr] = robMdl.My_VwTovlvr(V, w);
    robot.sendVelocity(vl, vr);

    
    pause(0.005);
    
end
robot.stop()
robot.shutdown();
pause(5.);
close all;
%% 3.3
close all; clear;clc;
robot = raspbot('sim');
Ks = 3.;
Kv = 1.;
tPause = 0.0;
fig8 = figure8ReferenceControl(Ks, Kv, tPause);
robMdl = robotModel();
tf     = fig8.getTrajectoryDuration();
dt = 0.001;
numSamples = tf/dt;
init_dist = 0;
init_x = 0;init_y = 0;init_th=0;
init_pose = pose(init_x, init_y, init_th);

refControl = fig8;
robTraj = robotTrajectory(numSamples, init_dist, init_pose, dt, refControl);
timeArray(1) = 0;
xArray(1) = init_x;
yArray(1) = init_y;

distanceArray(1)= init_dist;

trajPlot = plot(xArray(1), yArray(1), 'b-', 'DisplayName', 'Integrated distance');
xlabel('x');
ylabel('y');
title('Figure 8 trajectory');
xlim([-0.5 0.5])
ylim([-0.5 0.5])

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
    
    [V, w]    = robTraj.getRefVelAtTime(timeNow);
    [vl , vr] = robMdl.My_VwTovlvr(V, w);
    robot.sendVelocity(vl, vr);
    
    newPose = robTraj.getPoseAtTime(timeNow);

    
    set(trajPlot, 'xdata', [get(trajPlot,'xdata') newPose.x],...
    'ydata', [get(trajPlot,'ydata') newPose.y]);
    pause(0.001);
    
end
pause(1.)
robot.stop()
robot.shutdown();
pause(5.);
close all;
%% trapezoidal

close all; clear;clc;
robot = raspbot();
Ks = 3.;
Kv = 1.;
tPause = 0.0; 
dist = 1.; 
amax = 3*0.25; 
vmax = 0.25;
sgn=1;
trapizod = trapezoidalStepReferenceControl(dist, amax, vmax, sgn, tPause);
robMdl = robotModel();
tf     = trapizod.getTrajectoryDuration();
dt = 0.001;
numSamples = int64(tf/dt)+1;
init_dist = 0;
init_x = 0;init_y = 0;init_th=0;
init_pose = pose(init_x, init_y, init_th);

refControl = trapizod;
robTraj = robotTrajectory(numSamples, init_dist, init_pose, dt, refControl);
timeArray(1) = 0;
xArray(1) = init_x;
yArray(1) = init_y;

distanceArray(1)= init_dist;

trajPlot = plot(xArray(1), yArray(1), 'b-', 'DisplayName', 'Integrated distance');
xlabel('x');
ylabel('y');
title('Figure 8 trajectory');
xlim([-1. 1])
ylim([-1 1])

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
    [V, w] = robTraj.getRefVelAtTime(timeNow);
    [vl , vr] = robMdl.VwTovlvr(V, w);

    robot.sendVelocity(vl, vr);
    
    newPose = robTraj.getPoseAtTime(timeNow);
    newPose.x
    newPose.y

    set(trajPlot, 'xdata', [get(trajPlot,'xdata') newPose.x],...
    'ydata', [get(trajPlot,'ydata') newPose.y]);
    pause(0.001);
    
end
%% challenge
close all; clear;clc;
robot = raspbot();
Ks = 3.;
Kv = 1.;
tPause = 0.0;
fig8 = figure8ReferenceControl(Ks, Kv, tPause);
robMdl = robotModel();
tf     = fig8.getTrajectoryDuration();
dt = 0.001;
numSamples = tf/dt;
init_dist = 0;
init_x = 0;init_y = 0;init_th=0;
init_pose = pose(init_x, init_y, init_th);
refControl = fig8;

FORWARD = 0;
BOTH = 1.;
tau = 0.6;

robTraj = robotTrajectory(numSamples, init_dist, init_pose, dt, refControl);
myController = controller(robTraj, tau, BOTH);
timeArray(1) = 0;
%plotting arrays
xArray(1) = init_x;
yArray(1) = init_y;
thArray(1)= 0;
distanceArray(1)= init_dist;
xHatArray(1) = 0;
yHatArray(1) = 0;
thHatArray(1) = 0;

errorXArray(1) = 0;
errorYArray(1) = 0;
errorThArray(1) = 0;


sl_bias = robot.encoders.LatestMessage.Vector.X;
sr_bias = robot.encoders.LatestMessage.Vector.Y;
sl = 0;
sr = 0;
x = 0;
y = 0;
th = 0;
firstIteration = false;
tick = 1;
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
        robot.stop()
        break;
    end
    
    
    dsl = robot.encoders.LatestMessage.Vector.X - sl - sl_bias;
    dsr = robot.encoders.LatestMessage.Vector.Y - sr - sr_bias;
    
    sl  = sl + dsl;
    sr  = sr + dsr;
    ds  = (dsl  + dsr)/2;
    dth = (dsr  - dsl)/robMdl.my_W;
    th =  th + dth;    
    x  =  x  + cos(th)*ds;
    y  =  y  + sin(th)*ds;
    actualPose = pose(x, y, th);
    [V, w, error] = myController.pidCorrect(timeNow, actualPose);
    if abs(w)>1.2
        fprintf('I found one: %.2f \n', w);
        w = w/abs(w);
        tick
    end
    [vl , vr] = robMdl.My_VwTovlvr(V, w);
    if timeNow > tf && timeNow < tf + 1
        robot.stop();
    else
        robot.sendVelocity(vl, vr);       
    end
    
 
    newPose = robTraj.getPoseAtTime(timeNow);
    timeArray(tick) = timeNow;
    xArray(tick) = newPose.x;
    yArray(tick) = newPose.y;
    thArray(tick) = newPose.th;
    xHatArray(tick)   = actualPose.x;
    yHatArray(tick)   = actualPose.y;
    thHatArray(tick)  = actualPose.th;
    errorXArray(tick) = error(1);
    errorYArray(tick) = error(2);
    errorThArray(tick) = error(3);
    tick = tick + 1;
    %set(XPlot, 'xdata', [get(XPlot,'xdata') timeNow],...
    %'ydata', [get(XPlot,'ydata') newPose.x]);
    %set(YPlot, 'xdata', [get(YPlot,'xdata') timeNow],...
    %'ydata', [get(YPlot,'ydata') newPose.y]);

    %set(trajPlot, 'xdata', [get(trajPlot,'xdata') newPose.x],...
    %'ydata', [get(trajPlot,'ydata') newPose.y]);
    
    %set(actualTrajPlot, 'xdata', [get(actualTrajPlot,'xdata') x],...
    %'ydata', [get(actualTrajPlot,'ydata') y]);
    %errorMag = sqrt(error(1)^2 + error(2)^2);
    %set(errorPlot, 'xdata', [get(errorPlot,'xdata') timeNow],...
    %'ydata', [get(errorPlot,'ydata') error(1)]);
    pause(0.005);
    

end

robot.stop()
robot.shutdown();
x
y
th

figure;
XPlot = plot(timeArray, xArray, 'r-', 'DisplayName', 'x');
hold on;
YPlot = plot(timeArray, yArray, 'g-', 'DisplayName', 'y');
hold on;
ThPlot = plot(timeArray, thArray, 'b-', 'DisplayName', 'th');
hold on;
XHatPlot = plot(timeArray, xHatArray, 'c-', 'DisplayName', 'xhat');
hold on;
YHatPlot = plot(timeArray, yHatArray, 'm-', 'DisplayName', 'yhat');
hold on;
ThHatPlot = plot(timeArray, thHatArray, 'k-', 'DisplayName', 'thhat');
title('reference trajectory (x,y,th) and actual trajectory(xhat yhat thhat) ');
%ylim([-1 1])
xlabel('time')
legend('show')

figure;
actualTrajPlot = plot(xHatArray, yHatArray, 'b-', 'DisplayName', 'Actual trajectory');
hold on;
trajPlot = plot(xArray, yArray, 'r-', 'DisplayName', 'reference trajectory');
xlabel('x');
ylabel('y');
title('Figure 8 trajectory');
%xlim([-0.5 0.5])
%ylim([-0.5 0.5])
legend('show')
figure;
errorXPlot = plot(timeArray, errorXArray, 'r-', 'DisplayName', 'X-Error');
hold on;
errorYPlot = plot(timeArray, errorYArray, 'g-', 'DisplayName', 'Y-Error');
hold on;
errorThPlot = plot(timeArray, errorThArray, 'b-', 'DisplayName', 'Th-Error');
hold on;
xlabel('time')
ylabel('error(m for x and y, rad for th)')
%ylim([-0.05 0.05])
legend('show')
%pause(5.);
%close all;
%% with Traj follower
close all; clear;clc;
robot = raspbot('sim');
robMdl = robotModel();
pause(2.)
Ks = 3.;
Kv = 1.;
tPause = 0.0;
fig8 = figure8ReferenceControl(Ks, Kv, tPause);
tf   = fig8.getTrajectoryDuration();
dt = 0.001;
numSamples = tf/dt;
init_dist = 0;
init_x = 0; init_y = 0;init_th=0;
initPose = pose(init_x, init_y, init_th);

FORWARD = 0;
BOTH = 1.;
tau = 0.8;

fig8Traj = robotTrajectory(numSamples, init_dist, initPose, dt, fig8);
myController = controller(fig8Traj, tau, BOTH);
fig8Follower = trajectoryFollower(robot, robMdl, fig8Traj, myController);
timeArray(1) = 0;
%plotting arrays
xArray(1) = init_x;
yArray(1) = init_y;
thArray(1)= 0;
distanceArray(1)= init_dist;
xHatArray(1) = 0;
yHatArray(1) = 0;
thHatArray(1) = 0;

errorXArray(1) = 0;
errorYArray(1) = 0;
errorThArray(1) = 0;


sl_bias = robot.encoders.LatestMessage.Vector.X;
sr_bias = robot.encoders.LatestMessage.Vector.Y;
myPoseEstimator = poseEstimator(robot, robMdl, initPose, sl_bias, sr_bias);
sl = 0;
sr = 0;
x = 0;
y = 0;
th = 0;
firstIteration = false;
tick = 1;

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
        robot.stop()
        break;
    end
    actualPose = myPoseEstimator.update();
    error = fig8Follower.update(timeNow, actualPose);
    newPose = fig8Follower.trajGenerator.getPoseAtTime(timeNow);
    
    timeArray(tick) = timeNow;
    xArray(tick) = newPose.x;
    yArray(tick) = newPose.y;
    thArray(tick) = newPose.th;
    xHatArray(tick)   = actualPose.x;
    yHatArray(tick)   = actualPose.y;
    thHatArray(tick)  = actualPose.th;
    errorXArray(tick) = error(1);
    errorYArray(tick) = error(2);
    errorThArray(tick) = error(3);
    tick = tick + 1;
    
    pause(0.005);
    

end

robot.stop()
robot.shutdown();
x
y
th

figure;
XPlot = plot(timeArray, xArray, 'r-', 'DisplayName', 'x');
hold on;
YPlot = plot(timeArray, yArray, 'g-', 'DisplayName', 'y');
hold on;
ThPlot = plot(timeArray, thArray, 'b-', 'DisplayName', 'th');
hold on;
XHatPlot = plot(timeArray, xHatArray, 'c-', 'DisplayName', 'xhat');
hold on;
YHatPlot = plot(timeArray, yHatArray, 'm-', 'DisplayName', 'yhat');
hold on;
ThHatPlot = plot(timeArray, thHatArray, 'k-', 'DisplayName', 'thhat');
title('reference trajectory (x,y,th) and actual trajectory(xhat yhat thhat) ');
%ylim([-1 1])
xlabel('time')
legend('show')

figure;
actualTrajPlot = plot(xHatArray, yHatArray, 'b-', 'DisplayName', 'Actual trajectory');
hold on;
trajPlot = plot(xArray, yArray, 'r-', 'DisplayName', 'reference trajectory');
xlabel('x');
ylabel('y');
title('Figure 8 trajectory');
%xlim([-0.5 0.5])
%ylim([-0.5 0.5])
legend('show')
figure;
errorXPlot = plot(timeArray, errorXArray, 'r-', 'DisplayName', 'X-Error');
hold on;
errorYPlot = plot(timeArray, errorYArray, 'g-', 'DisplayName', 'Y-Error');
hold on;
errorThPlot = plot(timeArray, errorThArray, 'b-', 'DisplayName', 'Th-Error');
hold on;
xlabel('time')
ylabel('error(m for x and y, rad for th)')
%ylim([-0.05 0.05])
legend('show')
%pause(5.);
%close all;