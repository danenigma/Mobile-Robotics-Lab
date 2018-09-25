close all; clear; clc;
% Setup world.
% lineObject instances represent bodies.
walls = lineObject();
% Specify points as an n x 2 array.
% Simulator length units: m.
walls.lines = [1.5 0; 3 1.5; 1.5 3; 0 1.5; 1.5 0];
obstacle = lineObject();
obstacle.lines = [1.6 1.4; 1.6 1.6; 1.4 1.6; 1.4 1.4; 1.6 1.4];

% Environments are specified as lineMap instances.
% lineMap constructor takes an array of lineObject instances as input.
map = lineMap([walls obstacle]);
% Show the map.
hf = map.plot();

% Fire simulator.
% Close map plot.
if ishandle(hf)
    close(hf);
end

% By default robot starts at pose x = 0, y = 0, theta = 0.
%rob = raspbot('sim');
% Start at desired pose.
%robot = raspbot('sim',[1; 1.5; 0]);
%robot.genMap(map.objects);
%%
robot = raspbot();

initialized = false;
firstTime = true;
goalDistance = 1.;
stopTime = 6;
errorTolerance = 0.0001;
p_gain = 3;
d_gain = 0.2;
i_gain = 0.1;
lastError = goalDistance - 0;

errorArray(1) = lastError;
distanceArray(1) = 0;
timeArray(1) = 0;
actDistArray(1) = 0;%encoder distance
figure;


distancePlot = plot(timeArray, distanceArray , 'b-', 'DisplayName', 'Integrated distance');
hold on;
actDistPlot  = plot(timeArray, actDistArray(1) , 'r-',  'DisplayName', 'Actual distance');

legend('show')
pause(0.5)
title('distance vs time')
xlabel('time in secs');
ylabel('Distance in meters');
%pause(2.);


sl_bias   = robot.encoders.LatestMessage.Vector.X;
sr_bias   = robot.encoders.LatestMessage.Vector.Y;

s = 0;
dist = 1.; 
amax = 3*0.25; 
vmax = 0.25;
sgn=1;
tf = (dist + (vmax^2)/amax)/vmax;

t_prev = 0;
t = 0;
t_delayed = 0.50;
uref_delayed = 0;
s_delayed = 0;

MODE = 1;

myClock = tic;
t0 = toc(myClock);

while t < tf + 1
    t  = toc(myClock)-t0;
    dt = t - t_prev;
    t_prev = t;

    if t > stopTime
        robot.stop();
        fprintf('stopping the robot\n');
        break
    end
    sl   = robot.encoders.LatestMessage.Vector.X - sl_bias;
    sr   = robot.encoders.LatestMessage.Vector.Y - sr_bias;    
    distance = (sl + sr)/2
    if t<t_delayed
        uref_delayed = 0;
    else
        uref_delayed = trapezoidalVelocityProfile( t-t_delayed , amax, vmax, dist, sgn);
    end
    
    s_delayed = s_delayed + uref_delayed*dt
    
    uref = trapezoidalVelocityProfile(t , amax, vmax, dist, sgn);    
    error = s_delayed - distance;
    errorDerivative = (error - lastError)/dt;
    lastError = error;
    
    if firstTime
        errorIntegral = 0;
        firstTime = false;
    end
    errorIntegral   = errorIntegral + error * dt;
    control = uref + MODE*(error * p_gain + errorDerivative * d_gain + errorIntegral * i_gain);
    
    if control > .3
        control = .3;
    end
    robot.sendVelocity(control, control);

    set(distancePlot, 'xdata', [get(distancePlot,'xdata') t],...
    'ydata', [get(distancePlot,'ydata') s_delayed]);
    set(actDistPlot, 'xdata', [get(actDistPlot,'xdata') t],...
    'ydata', [get(actDistPlot,'ydata') distance]);

    pause(0.005)
end
robot.stop()
pause(10.)
robot.shutdown();
close all; clear ; clc;
%%
close all; clear; clc;
my_clock = tic;
dist = 1.; 
amax = 3*0.25; 
vmax = 0.25;
sgn=1;
tf = (dist + (vmax^2)/amax)/vmax;
vArray(1) = 0;
tArray(1) = 0;
sArray(1) = 0;
figure;
vPlot = plot(tArray, vArray , 'b-');
xlim([0 4.5]);
ylim([0 .5]);
title('speed vs time')
xlabel('time in secs');
ylabel('speed in m/sec');

t0 = toc(my_clock);
t = 0;
s = 0;
t_prev = 0;
while t<tf
    t  = toc(my_clock)-t0;
    dt = t-t_prev;
    t_prev = t;
    uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn);
    s = s + uref*dt;

    set(vPlot, 'xdata', [get(vPlot,'xdata') t],...
    'ydata', [get(vPlot,'ydata') uref]);
    pause(0.005);
end

