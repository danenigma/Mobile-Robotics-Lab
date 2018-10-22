function [ x, y, b] = irToXy( i, r )
% irToXy finds position and bearing of a range pixel endpoint
% Finds the position and bearing of the endpoint of a range pixel in the plane.
% Fill in code here

th = (-5 + (i-1))*pi/180 ;
x  = r.*cos(th');
y  = r.*sin(th');
b  = atan2(y,x);

end