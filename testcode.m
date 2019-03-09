global N RL
[map,ogmap,truth] = NDVIMap(N,1);
figure
imshow(truth,'InitialMagnification','fit')
figure
bitmap = createBitmap(truth,xy,xMax,xMin,yMax,yMin);
imshow(bitmap,'InitialMagnification','fit')

%%
xdim = xMax-xMin;
ydim = yMax-yMin;
empty = zeros(ydim,xdim);

xEnds = xy(2:6,:);
% for i = 1:N
%     if truth(1,i,1)==1 && truth(1,i,2)==0 %%red
%         empty(ydim-RL+yMin:ydim+yMin,xEnds(i)-xMin) = 1;
%     elseif truth(1,i,1)==1 && truth(1,i,2)==0.5
%         empty(ydim-RL+yMin:ydim+yMin,xEnds(i)-xMin) = 0.5;
%     else
%         empty(ydim-RL+yMin:ydim+yMin,xEnds(i)-xMin) = 0.25;
%     end
% end

for i = 1:N
    if truth(1,i,1)==1 && truth(1,i,2)==0 %%red
        empty(1:RL+1,xEnds(i)) = 1;
    elseif truth(1,i,1)==1 && truth(1,i,2)==0.5
        empty(1:RL+1,xEnds(i)) = 0.5;
    else
        empty(1:RL+1,xEnds(i)) = 0.25;
    end
end

imshow(empty,'InitialMagnification','fit')