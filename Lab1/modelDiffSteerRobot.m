function [xArray, yArray, thArray] = modelDiffSteerRobot(vl, vr, t0, tf, dt)

    t  = t0:df:tf;
    xArray = zeros(1, length(t)+1);
    yArray = zeros(1, length(t)+1);
    thArray = zeros(1, length(t)+1);
    w = 0.084;
    
    for i = 2:length(t)
        
        V =  (vl(i-1)+vr(i-1))/2;
        w =  (vr(i-1)-vl(i-1))/2;
    
        thArray(i) = thArray(i-1) + w*dt;
        xArray(i)  = xArray(i-1)  + cos(thArray(i))*V*dt;
        yArray(i) = thArray(i-1)  + cos(thArray(i))*w*dt;
        
    end
    
end