%% testing rangeImage
clear;clc;
load('range_image.mat')

range_obj = rangeImage();
range_im = range_vid(:, 20);

for i=1:size(range_vid, 2)
indices = 1:360;
range_im = range_vid(:, 99);

pallet_pos = range_obj.findLineCandidate(range_im, 1);
valid_r    = range_im <1 & range_im > 0.06;

range_im = range_im(valid_r);
ths = indices(valid_r);
[x_disp, y_disp, th_disp] = range_obj.irToXy(ths, range_im);
plot(x_disp, y_disp, 'rx');
hold on;
plot(pallet_pos(1), pallet_pos(2), 'go');

pause(.1)
xlim([-1 1])
ylim([-1 1])

end
%%
close all; clear; clc;

robot = raspbot();
robot.stopLaser();
pause(1);
robot.startLaser();
pause(1)

f = figure;

range_obj = rangeImage();
i = 1:360;
while true 
   r =  robot.laser.LatestMessage.Ranges;
   pallet_pos = range_obj.findLineCandidate(r, 1)
   valid_r    = r <1 & r > 0.06; %less than one meters and > 2cm 
   [x_disp, y_disp, th_disp] = irToXy(i(valid_r), r(valid_r));
   
   clf(f);
   plot(x_disp, y_disp, 'rx');
   hold on;
   plot(0,0,'ro');
   hold on;
   line([0,0], [0, 0.5])
   hold on;
   line([0,0],[0,-0.5]);
   hold on;
   line([0,0.06],[0,0])
   hold on;
   plot(pallet_pos(1), pallet_pos(2), 'go');
   pause(.2);
   
end
disp('done!!')
