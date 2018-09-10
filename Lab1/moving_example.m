%% Connect to the Robot

robot = raspbot();%Make sure you are connected to Raspbot-X WiFi network.


%% Move the robot forward and backward
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
%robot.sendVelocity(-.050, -.050)

%pause(0.5)

% Look at the encoder data of the left wheel (in meters)

%robot.encoders.LatestMessage.Vector.X


%% Laser

% spin up the laser, wait for it to start spewing data

robot.startLaser()

pause(3)

% look at the ranges (there are 360 of them)

robot.laser.LatestMessage.Ranges