close all; clear; clc;

robot = raspbot();


x = 0; y= 0; th = 0;
W = 0.084;

thArray(1) = th;
xArray(1) = x;
yArray(1) = y;

ks = 3.;
kk = 15.1084;
sf = 1.;
v  = 0.2;
tf = sf/v;
kth = (2*pi)/sf;

Tf = 2*tf;

myPlot = plot(xArray, yArray, 'b-');
xlim([-.50 .50]);
ylim([-.50 .50]);


sl_bias   = robot.encoders.LatestMessage.Vector.X;
sr_bias   = robot.encoders.LatestMessage.Vector.Y;

sl = 0;
sr = 0;

t  = 0;
t_prev = 0;

a = tic;

while( t < Tf)
    
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
     

    pause(0.001);
        
    set(myPlot, 'xdata', [get(myPlot,'xdata') x],...
    'ydata', [get(myPlot,'ydata') y]);
        
end
pause(0.01)
robot.stop()