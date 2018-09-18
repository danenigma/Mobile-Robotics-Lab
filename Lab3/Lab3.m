%% CASE I
% lineObject instances represent bodies.
walls = lineObject();
% Specify points as an n x 2 array.
% Simulator length units: m.
walls.lines = [1.5 0; 3 1.5; 1.5 3; 0 1.5; 1.5 0];
obstacle = lineObject();
obstacle.lines = [1.6 1.4; 1.6 1.6; 1.4 1.6; 1.4 1.4; 1.6 1.4];

% Environments are specified as lineMap instances.
% lineMap constructor takes an array of lineObject instances as input.
map = lineMap([walls obstacle]);
% Show the map.
hf = map.plot();

%% Fire simulator.
% Close map plot.
if ishandle(hf)
    close(hf);
end

% By default robot starts at pose x = 0, y = 0, theta = 0.
%rob = raspbot('sim');
% Start at desired pose.
robot = raspbot('sim',[1; 1.5; 0]);
%% Add map to simulator.
% neato.genMap takes an array of lineObject instances as input. 
robot.genMap(map.objects);
%%
%robot =  raspbot();

%robot.encoders.NewMessageFcn = @encoderEventListener;
previousLeftEncoder  = robot.encoders.LatestMessage.Vector.X;
previousRightEncoder = robot.encoders.LatestMessage.Vector.Y;
tic;
tickCounter = 1;
time = 0;
timeArray = [];
leftArray = [];
cmdV = .050; 
while 1
    
    robot.sendVelocity(cmdV, cmdV)
    pause(.05);
    
    leftEncoder  = robot.encoders.LatestMessage.Vector.X;
    rightEncoder = robot.encoders.LatestMessage.Vector.Y;
    dT  = toc;
    
    dL = leftEncoder  - previousLeftEncoder;
    dR = rightEncoder - previousRightEncoder;
    
    previousLeftEncoder  = leftEncoder;
    previousRightEncoder = rightEncoder;
    
    time = time + dT
    V   = 0.5*(dL+dR)/dT;
    
    timeArray(tickCounter) = time;
    velocityArray(tickCounter) = V;
    
    tickCounter = tickCounter + 1;
    if time > 2
        robot.stop()
        break
    end
    tic
    
end 
figure;
plot(timeArray, velocityArray);
xlabel('Time (sec)')
ylabel('Velocity m/sec')
title('Velocity vs Time');
 %% CASE II
robot =  raspbot();

%robot.encoders.NewMessageFcn = @encoderEventListener;
tic;

while 1
    
    dL  = robot.encoders.LatestMessage.Vector.X;
    dR  = robot.encoders.LatestMessage.Vector.Y;
    dT  = toc;
    V   = 0.5*(dL+dR)/dT; 
    tic
    pause(0.005);

end 
          