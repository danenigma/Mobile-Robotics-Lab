%% sim example
dt = 0.001;
tf = sqrt(32*pi);
t0 = 0;
t = t0:dt:tf;
W = 0.084;
vr = 0.1 + (W/16)*t;
vl = 0.1 - (W/16)*t;

[x_t, y_t, th_t] = modelDiffSteerRobot(vl, vr, t0, tf, dt)