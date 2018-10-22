classdef rangeImage < handle

    properties(Constant)
    ANGLE_OFFSET = 5*pi/180;
    PALLET_WIDTH = 0.127;
    
    end
    
    properties(Access = public)
    
    end
    methods(Access = public)
        
        function obj = rangeImage()
        end
        function pallet_pos = findLineCandidate(obj, range_im, min_num_pts)
        pallet_pos = [0;0;0];
        pixels = length(range_im);
        goodOnes = range_im > 0.06 & range_im < 1.0;
        range_im = range_im(goodOnes);
        indices = linspace(1,pixels,pixels)';
        indices = indices(goodOnes);
        % Compute the angles of surviving points
        ths = (indices-1)*(pi/180)-rangeImage.ANGLE_OFFSET;
        
        for pixel = 1:length(range_im)
            %get the points to the left and right of the center point
            l_r_points = unique(obj.getLeftAndRightPixels(pixel, range_im, ths));
            %compute the x and y of those points
            x = range_im(l_r_points).*cos(ths(l_r_points));
            y = range_im(l_r_points).*sin(ths(l_r_points));
            %number of points 
            numPts = length(x);
            
            if numPts >= min_num_pts
               
                mu_x = mean(x);
                mu_y = mean(y);
                x = x - mu_x;
                y = y - mu_y;
                Ixx = x' * x;
                Iyy = y' * y;
                Ixy = - x' * y;
                Inertia = [Ixx Ixy;Ixy Iyy] / numPts; % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                if lambda(1) < 1.3 
                   disp('found a line!!')
                   %compute its pose 
                   th = atan2(2*Ixy, Iyy-Ixx)/2;
                   pallet_pos = [mu_x;mu_y;th];
                   break
                end
            end
            
            
        end
        
        end
        
        function [ x, y, b] = irToXy(obj, i, r )
        % irToXy finds position and bearing of a range pixel endpoint
        % Finds the position and bearing of the endpoint of a range pixel in the plane.
        % Fill in code here

        th = (-rangeImage.ANGLE_OFFSET + (i-1))*pi/180 ;
        x  = r.*cos(th');
        y  = r.*sin(th');
        b  = atan2(y,x);

        end
        function points = getLeftAndRightPixels(obj, idx, range_im, ths)
            %go right  till length/2
            points = [];
            N = length(range_im);
            r  = range_im(idx);
            th = ths(idx);
            [x, y, ~] = obj.irToXy(th, r);
            r_vec = [x; y];
            %get points to the right of the central point
            for i=1:N-1
                
                index  = mod(idx-1+i, N);
                r_i = range_im(index+1);
                th_i = ths(index+1);
                
                [x_i, y_i, ~] = obj.irToXy(th_i, r_i);
                r_i_vec = [x_i; y_i];
                distance = norm(r_i_vec-r_vec);
                
                if distance > rangeImage.PALLET_WIDTH/2
                    
                    break
                else
                    points = [points, index+1];
                end
            end
            %get points to the left of the central point
            for i=1:N-1
                
                index  = mod(idx-1-i, N);
                
                if index < 0 
                    index = index + N;
                end
                
                r_i = range_im(index+1);
                th_i = ths(index+1);
                
                [x_i, y_i, ~] = obj.irToXy(th_i, r_i);
                r_i_vec = [x_i; y_i];
                %distance from the center point.
                distance = norm(r_i_vec-r_vec);
                
                if distance > rangeImage.PALLET_WIDTH/2
                    break
                else
                    points = [points, index+1];
                end
            end
        
        end
    end

end