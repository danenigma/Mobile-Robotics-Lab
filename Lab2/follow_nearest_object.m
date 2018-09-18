function [] = follow_nearest_object(robot, ideal_object_range, prop_gain,pause_time)
    robot.stopLaser();
    pause(1);
    robot.startLaser();
    pause(1);
    W = 0.084;
    min_dist = .06;
    max_object_range = 1.5;
    max_bearing = pi/2;
    f = figure;
    grid on;
    rotation_mat = [0, -1; 1, 0];
    while 1

       r =  robot.laser.LatestMessage.Ranges;
       [x_min, y_min, th_min, object_range] = find_nearest_object(r, min_dist, max_object_range, max_bearing);
       
       object_range
       obj_on_rotated_axis  = rotation_mat*[x_min, y_min]'
       if object_range < 1.
           plot(obj_on_rotated_axis(1), obj_on_rotated_axis(2), 'bx');
           xlabel('Y');
           ylabel('X');
           title('Smart Luggage')
           xlim([-2,2]);
           ylim([-2,2]);
           if object_range == ideal_object_range
                   robot.stop();
           else

                   V = prop_gain*(object_range-ideal_object_range);
                   %k = y_min / (object_range^2);
                   %V = 0.05;
                   k = prop_gain* atan2(y_min, x_min);
                   w = k
                   VL = V - (W/2)*w;
                   VR = V + (W/2)*w;
                   
                   robot.sendVelocity(VL, VR)
                   pause(pause_time);

           end
       
       else 
           robot.stop();
       end
    end
clf(f);


end