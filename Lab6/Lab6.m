%% 3.1 Warm Up Exercise 1: Integrate Some Cubic Spirals
idealCurve = cubicSpiralTrajectory([-3.0 1.55 3.0],10001);
idealPose  = idealCurve.getFinalPose();
fprintf('x:%f mm y:%f mm t:%f\n',idealPose(1)*1000,idealPose(2)*1000,idealPose(3));
curve = cubicSpiralTrajectory([-3.0 1.55 3.0], 4001);
pose  = curve.getFinalPose();
fprintf('x-error:%f mm y error:%f mm t:%f\n',(idealPose(1)-pose(1))*1000,...
                                             (idealPose(2)-pose(2))*1000,...
                                             (idealPose(3)-pose(3)));
%% 3.2 Making a Lookup Table
close all; clear; clc;
num_samples = 4001;
scale = 10;
cubicSpiralTrajectory.makeLookupTable(scale)
%% 3.3 Driving Positively Anywhere in Style
clear;clc;
x = 1.2; y = 0.7; th=0; sgn = 1;
curve = cubicSpiralTrajectory.planTrajectory(x,y,th,sgn);
pose  = curve.getFinalPose();
fprintf('x:%f y :%f t:%f\n',pose(1),...
                            pose(2),...
                            pose(3));
%% 3.3.2 Velocity Planning
clear;clc;
x = 2.5; y = 0; th = 0; sgn = 1;
curve = cubicSpiralTrajectory.planTrajectory(x,y,th,sgn);
curve.planVelocities(0.3);

%% Challenge Task: Plan and Execute Arbitrary Motions
close all; clear; clc;
robot = raspbot('sim');
pause(2.);
mrpl = mrplSystem(robot);
xf = 0.3048; yf = 0.3048; thf = 0.0;
mrpl.executeTrajectoryToRelativePose(xf, yf, thf,1)
pause(1.)
xf = -0.6096; yf = -0.6096; thf = -pi()/2.0;
mrpl.executeTrajectoryToRelativePose(xf, yf, thf,1)
pause(1.)
xf = -0.3048; yf = 0.3048; thf = pi()/2.0;
mrpl.executeTrajectoryToRelativePose(xf, yf, thf,1)
robot.shutdown()
hold on;
plot(0.3048, 0.3048, 'rx');
plot(-0.3048, -0.3048, 'gx');
plot(-0.6096, 0.0, 'bx');
