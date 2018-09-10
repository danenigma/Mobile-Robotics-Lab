close all; clear; clc;

sines   = sin((1:360)*(pi/180));
cosines = cos((1:360)*(pi/180));

for i=1:100

    rand_range = rand(1,360);
    X = rand_range.*cosines;
    Y = rand_range.*sines;
    plot(X, Y)
    pause(.05)

end
