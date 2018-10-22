clear; clc;
robot = raspbot();
robot.startLaser();
pause(5)

R = robot.laser.LatestMessage.Ranges;
robot.stopLaser();

x = zeros(360,1);
for i = 1:length(R)
    r = R(i);
    [x(i), y(i), b] = irToXy(i, r);
end

plot([-1 5],[0 0])
hold on
pl = plot(x, y, '*');
pl.Color = [0 0 0];