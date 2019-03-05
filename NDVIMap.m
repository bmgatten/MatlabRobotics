function [noised,ogmap,truth] = NDVIMap(numrow,numplants)
%function to make NDVI map for MAE 252
%   output meanings
%   noised = the final output with the noise added
%   ogmap = image without noise
%   truth = map with as an array, not an image
red = randi([0 1],numplants,numrow);
green = rand(numplants,numrow);
blue = randn(numplants,numrow)/5;
for g = 1:numplants
    for h = 1:numrow
        if green(g,h)<0.33 && red(g,h)~=0
            green(g,h) = 0;
        elseif green(g,h)<0.667 && green(g,h)>=0.33
            green(g,h) = 0.5;
        elseif green(g,h)<0.33 && red(g,h)==0
            green(g,h) = 1;
        else
            green(g,h) = 1;
        end
    end
end

trueblue = zeros(numplants,numrow);
truth = cat(3,red,green,trueblue);
catted = cat(3,red,green,blue);

for i = 1:numplants
    for j = 1:numrow        
        map1(1+(30*i):30+(30*i),1+(50*j):50+(50*j)) = catted(i,j,1);
        map2(1+(30*i):30+(30*i),1+(50*j):50+(50*j)) = catted(i,j,2);
        map3(1+(30*i):30+(30*i),1+(50*j):50+(50*j)) = catted(i,j,3);
    end
end
map = cat(3,map1,map2,map3);
map = map(31:end,51:end,:);
ogmap = imresize(map,[786 1024]);
h = fspecial('motion',60,randi(60));
blurred = imfilter(ogmap,h);
noised = imnoise(blurred,'speckle',0.03);
end

