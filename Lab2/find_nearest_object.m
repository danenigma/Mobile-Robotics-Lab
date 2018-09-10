function [x, y, th, distance] = find_nearest_object(r, min_dist, max_object_range, max_bearing)
% r is a lidar image


i = 1:360;
[x_img, y_img, th_img] = irToXy(i, r);
valid_bearing = (abs(th_img) < max_bearing);

valid_r = (valid_bearing & (r > min_dist)) & (r < max_object_range);

r(~valid_r) = inf;%set invalid candidates to inf

[distance, nearest_idx]  = min(r);

x = x_img(nearest_idx);
y = y_img(nearest_idx);
th = th_img(nearest_idx);


end 