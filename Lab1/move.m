function [] = move(robot, v, distance, direction)


leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;

timeArray = [];
leftArray = [];
rightArray = [];

time_elapsed = 0;
tickCounter = 1;
time = 0;
distance = 0.0254*distance;
tic
while 1
   
    robot.sendVelocity(v, v)
    pause(.05);
    
    timeArray(tickCounter)  = toc;
    leftArray(tickCounter)  = robot.encoders.LatestMessage.Vector.X;
    rightArray(tickCounter) = robot.encoders.LatestMessage.Vector.Y;
    
    
    signedDistance = abs((leftArray(tickCounter) - leftStart)+(rightArray(tickCounter) - rightStart))/2;
    % 12 inch = 0.3048 meter
    
    if signedDistance > distance
        robot.stop();
        break
    end
    plot(timeArray,(leftArray-leftStart)*100, timeArray, 100*(rightArray-rightStart)); 
    xlabel('time (sec)');
    ylabel('Encoder Distance (cm)');
    title(direction);
    tickCounter = tickCounter +1;
    
end


end