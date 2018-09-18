function [xArray, yArray, thArray] = modelDiffSteerRobot(vl, vr, t0, tf, dt)

    t  = t0:dt:tf;
    xArray = zeros(1, length(t)+1);
    yArray = zeros(1, length(t)+1);
    thArray = zeros(1, length(t)+1);
    W = 0.084;
    
    for i = 1:length(t)
        
        V =  (vl(i) + vr(i))/2;
        w =  (vr(i)- vl(i))/W;
    
        thArray(i+1) = thArray(i) + w*dt;
        xArray(i+1)  = xArray(i)  + cos(thArray(i+1))*V*dt;
        yArray(i+1) = thArray(i)  + cos(thArray(i+1))*w*dt;
        
    end
    
end