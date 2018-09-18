%% sim example
close all; clear;
x = 0; y= 0; th = 0;

t = 0;
t_prev = 0;
thArray(1) = th;
xArray(1) = x;
yArray(1) = y;

tf = sqrt(32*pi);
myPlot = plot(xArray, yArray, 'b-');
xlim([0 0.4]);
ylim([0 0.4]);
v = 0.1;
tic
while( t < tf)
    
    t = toc;
    dt = t - t_prev;
    t_prev = t;
    
    w = (1/8)*t;
   
    th =  th + w*dt;
    
    x  =  x  + v*cos(th)*dt
    y  =  y  + v*sin(th)*dt
    
    set(myPlot, 'xdata', [get(myPlot,'xdata') x],...
    'ydata', [get(myPlot,'ydata') y]);
    pause(0.001)
end


%% figure 8

close all; clear;


ks = 3.;
kk = 15.1084;
sf = 1.;
v = 0.2;
tf = sf/v;
kth = (2*pi)/sf;
Tf = tf*ks;

x = 0; y= 0; th = 0;
thArray(1) = th;
xArray(1) = x;
yArray(1) = y;

t = 0;
t_prev = 0;

myPlot = plot(xArray, yArray, 'b-');
%xlim([-0.5 0.5]);
%ylim([-0.5 0.5]);

tic

while( t < Tf)
    
    t = toc/ks;
    dt = t - t_prev;
    t_prev = t;
    
    s = v*t;
    k = (kk/ks)*sin(kth*s);
    w = k*v;
    
    %VL = v + (W/2)*w
    %VR = v - (W/2)*w
    %robot.sendVelocity(VL, VR)
    
    th =  th + w*dt;
    x  =  x  + v*cos(th)*dt;
    y  =  y  + v*sin(th)*dt;
    
    set(myPlot, 'xdata', [get(myPlot,'xdata') x],...
    'ydata', [get(myPlot,'ydata') y]);
    pause(0.005)
    
end
%%
close all; clear;

robot = raspbot();
robot.encoders.NewMessageFcn=@encoderEventListener;
global encoderDataTimeStamp;

x = 0; y= 0; th = 0;
W = 0.084;
t = 0;
t_prev = 0;
thArray(1) = th;
xArray(1) = x;
yArray(1) = y;

tf = sqrt(32*pi);
myPlot = plot(xArray, yArray, 'b-');
xlim([0 0.4]);
ylim([0 0.4]);
v = 0.1;
my_time = [];
leftEncoder = [];
rightEncoder = [];
actualVL = [];
actualVR = [];
cmdVL = [];
cmdVR = [];
%tic
%t_bias   = encoderDataTimeStamp;
t_prev   = 0;
sl  = 0;
sr  = 0;

sl_bias  = robot.encoders.LatestMessage.Vector.X;
sr_bias  = robot.encoders.LatestMessage.Vector.Y;

tick_count = 1;
tic
while( t < tf)
    
    %t = encoderDataTimeStamp - t_bias
    t = toc/ks;
         
    %t = toc;
    
    dt = t - t_prev;
    t_prev  = t ;
    
    
    dsl = robot.encoders.LatestMessage.Vector.X - sl - sl_bias;
    dsr = robot.encoders.LatestMessage.Vector.Y - sr - sr_bias;
    
    sl  = sl + dsl;
    sr  = sr + dsr;
    
    
    %ds = (dsl + dsr)/2;
    
    my_time(tick_count) = t;
    leftEncoder(tick_count)  = sl;
    rightEncoder(tick_count) = sr;
    
    tick_count = tick_count + 1;
    a_vl = dsl/dt;
    a_vr = dsr/dt;
    a_v  = (a_vl + a_vr)/2;
    
    actualVL(tick_count) = a_vl;
    actualVR(tick_count) = a_vr;
    
    a_w = (a_vr-a_vl)/W;


    
    %w = (1/8)*t;
   
    th =  th + a_w*dt;
    
    x  =  x  + cos(th)*a_v*dt;
    y  =  y  + sin(th)*a_v*dt;
    
    set(myPlot, 'xdata', [get(myPlot,'xdata') x],...
    'ydata', [get(myPlot,'ydata') y]);
    VL = v - (W/16)*t;
    VR = v + (W/16)*t;
    
    cmdVL(tick_count) = VL;
    cmdVR(tick_count) = VR;
    
    robot.sendVelocity(VL, VR)
    pause(0.001)
end
robot.stop()

%% figure 8 on robot
close all; clear;clc;

robot = raspbot();

robot.encoders.NewMessageFcn=@encoderEventListener;

global encoderDataTimeStamp

x = 0; y= 0; th = 0;
W = 0.084;
t = 0;
t_prev = 0;

thArray(1) = th;
xArray(1) = x;
yArray(1) = y;

ks = 3.;
kk = 15.1084;
sf = 1.;
v  = 0.2;
tf = sf/v;
kth = (2*pi)/sf;

Tf = ks*tf;

myPlot = plot(xArray, yArray, 'b-');
xlim([-.50 .50]);
ylim([-.50 .5]);


sl_bias   = robot.encoders.LatestMessage.Vector.X;
sr_bias   = robot.encoders.LatestMessage.Vector.Y;
time_bias = encoderDataTimeStamp; 
sl = 0;
sr = 0;
t_prev = 0;
tt_prev = 0;

a = tic;

while( t < Tf)
    
    %tt = (encoderDataTimeStamp - time_bias)/ks;
    %dtt = tt - tt_prev;
    %tt_prev = tt;
    
    t = toc(a)/ks;
    dt = t - t_prev;
    t_prev = t;
    
    s = v*t;
    k = (kk/ks)*sin(kth*s);
    w = k*v;
    
    VR = v + (W/2)*w;    
    VL = v - (W/2)*w;

    robot.sendVelocity(VL, VR)
        

    dsl = robot.encoders.LatestMessage.Vector.X - sl - sl_bias;
    dsr = robot.encoders.LatestMessage.Vector.Y - sr - sr_bias;
    
    sl  = sl + dsl;
    sr  = sr + dsr;
    
    
    a_vl = dsl/dt;
    a_vr = dsr/dt;
    a_v  = (a_vl + a_vr)/2;
    a_w  = (a_vr - a_vl)/W;

    th =  th + a_w*dt;    
    x  =  x  + cos(th)*a_v*dt;
    y  =  y  + sin(th)*a_v*dt;
     


    set(myPlot, 'xdata', [get(myPlot,'xdata') x],...
    'ydata', [get(myPlot,'ydata') y]);
    pause(0.005);
        
end
robot.stop()