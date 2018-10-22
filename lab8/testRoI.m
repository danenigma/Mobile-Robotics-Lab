robot.startLaser();
pause(5)
R = robot.laser.LatestMessage.Ranges;
robot.stopLaser();
l = length(R);
xy = zeros(2, l);
for j = 1:l
    [xy(1,j),xy(2,j)] = irToXy(j,R(j));
end
%save('lalala4', 'xy')

pl = plot(xy(1,:), xy(2,:),'*');
axis equal

hold on
plot([0 0],[-6,6])
hold on
plot([-6,6],[0,0])

exp_pt = [0.6 0];
r = 0.5;
hold on
viscircles(exp_pt,[r])
pts = rOfi(exp_pt, xy, r);
hold on
if ~isempty(pts)
pl = plot(pts(1,:), pts(2,:),'*r');
end
