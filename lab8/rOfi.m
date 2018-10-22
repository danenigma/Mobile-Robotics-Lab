function pts = rOfi(exp_pt, detect_pts, RoI)
% detect_pts be 2*n list of x,y coordinates
% let RoI be r
% returning [] means failure

r2 = RoI^2;
pts = [];
eX= exp_pt(1);
eY= exp_pt(2);
for i = 1:length(detect_pts)
    if (detect_pts(1,i)-eX)^2 + (detect_pts(2,i)-eY)^2 < r2
        pts = [pts detect_pts(:,i)];
    end
end