function [bitmap] = createBitmap(truth,xy,xMax,xMin,yMax,yMin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global N RL

ydim = yMax-yMin;
xdim = xMax-xMin;

bitmap = zeros(ydim,xdim);

xEnds = xy(2:6,:);
for i = 1:N
    if truth(1,i,1)==1 && truth(1,i,2)==0 %%red
        bitmap(1:RL+1,xEnds(i)) = 1;
    elseif truth(1,i,1)==1 && truth(1,i,2)==0.5
        bitmap(1:RL+1,xEnds(i)) = 0.5;
    else
        bitmap(1:RL+1,xEnds(i)) = 0.25;
    end
end

end

