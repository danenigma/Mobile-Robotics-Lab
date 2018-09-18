%% TASK 1 
close all; clear; clc;

robot = raspbot();
robot.stopLaser();
pause(1);
robot.startLaser();
pause(1)

% look at the ranges (there are 360 of them)
i = 1:360;
min_dist = 0.06;
max_object_range = 1;
max_bearing = pi/2;

f = figure;
while 1
   r =  robot.laser.LatestMessage.Ranges;
   valid_r    = r <1 & r > 0.06; %less than one meters and > 2cm 
   [x_disp, y_disp, th_disp] = irToXy(i(valid_r),r(valid_r));
   %[x_disp, y_disp, th_disp] = irToXy(i, r);
   
   [x_min, y_min, th_min, distance] = find_nearest_object(r, min_dist, max_object_range, max_bearing)
   disp(th_min*180/pi)
   clf(f);
   plot(x_disp, y_disp, 'gx');
   hold on;
   plot(0,0,'ro');
   hold on;
   line([0,0], [0, 0.5])
   hold on;
   line([0,0],[0,-0.5])
   hold on;
   line([0,0.06],[0,0])
   %plot(0.06,0, 'g^')
   hold on;
   plot(x_min, y_min, 'b^');
   %hold on;
   %plot([0, x_min],[0, y_min]);
   pause(.2);
   xlim([-1 1])
   ylim([-1 1])
end

%% TASK 3 
close all; clear; clc;
robot = raspbot();
ideal_object_range = 0.5;
prop_gain = 0.8;
pause_time = 0.001;
follow_nearest_object(robot, ideal_object_range, prop_gain, pause_time)