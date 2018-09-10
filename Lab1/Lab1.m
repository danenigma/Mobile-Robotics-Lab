%%
robot = raspbot();

%% Lab 1 Task 1 Move the Robot
tic
while 1
    robot.sendVelocity(.050, .050)
    if toc > 4
        break
    end
    pause(.05)
end

pause(.05)

tic
while 1
    robot.sendVelocity(-.050, -.050)
    if toc > 4
        break
    end
    pause(0.05)
end

%% Lab 1 Task 2: Basic Simulation
v = 50;% 5cm/sec = 50 mm/sec
leftStart = 6934;
rightStart = 4396;

leftEncoder = leftStart;
rightEncoder = rightStart;


time_elapsed = 0;
while 1
    tic
    leftEncoder  = leftEncoder  + time_elapsed*v;
    rightEncoder = rightEncoder + time_elapsed*v;
    signedDistance = ((leftEncoder - leftStart)+(rightEncoder - rightStart))/2;
    % 12 inch = 0.3048 meter
    if signedDistance > 304.8
        break
    end
    pause(0.001);
    time_elapsed = toc;
end


 %% Lab 1 Task 4: Basic Plotting and Real Time Plotting
v = 50;
leftStart = 6934;
rightStart = 4396;

leftEncoder = leftStart;
rightEncoder = rightStart;

timeArray = [];
leftArray = [];
rightArray = [];

time_elapsed = 0;
tickCounter = 1;
time = 0;
while 1
    tic
    leftEncoder  = leftEncoder  + time_elapsed*v;
    rightEncoder = rightEncoder + time_elapsed*v;
    time = time + time_elapsed;
    timeArray(tickCounter) = time;
    leftArray(tickCounter) = leftEncoder;
    rightArray(tickCounter) = rightEncoder;
    
    signedDistance = ((leftEncoder - leftStart)+(rightEncoder - rightStart))/2;
    % 12 inch = 0.3048 meter
    if signedDistance > 304.8
        break
    end
    pause(0.05);
    time_elapsed = toc;
    tickCounter = tickCounter +1
    
    plot(timeArray,leftArray, timeArray, rightArray); 
end
leftArray  = leftArray - leftStart;
rightArray = rightArray - rightStart;

%% Lab1 challenge
close all; clear; clc;
robot = raspbot();
robot.forksDown()
v = .050;
% Forward
move(robot, v, 12, 'Moving Forward');
% Backward
pause(1.)
move(robot, -v, 12, 'Moving Backward');
%% with function 
