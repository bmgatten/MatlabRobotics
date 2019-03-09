clear
clc
close all
plantrows = 15;
plants = 10;
[map,ogmap,truth] = NDVIMap(plantrows,plants);
maptest = map;
MDL = open('predictionmodel2.mat');
MDL = MDL.MDL;
colidx = floor(linspace(1,1024,plantrows+1));
rowidx = floor(linspace(1,786,plants+1));
imshow(map,'InitialMagnification','fit');
%%
newmap = zeros(plants,plantrows,3);

for i = 1:plantrows
    for j = 1:plants
        f = map(rowidx(j):rowidx(j+1),colidx(i):colidx(i+1),:);
        rav = mean2(f(:,:,1));
        gav = mean2(f(:,:,2));
        bav = mean2(f(:,:,3));
        meanmap(j,i,1)=rav;
        meanmap(j,i,2)=gav;
        meanmap(j,i,3)=bav;
        prediction = predict(MDL,[rav,gav,bav,rav/gav,rav/bav,gav/bav]);
        if prediction==1
            newmap(j,i,1) = 0;
            newmap(j,i,2) = 0.5;
        elseif prediction == 2
            newmap(j,i,1) = 0;
            newmap(j,i,2) = 1;
        elseif prediction == 3
            newmap(j,i,1) = 1;
            newmap(j,i,2) = 1;
        elseif prediction == 4
            newmap(j,i,1) = 1;
            newmap(j,i,2) = 0.5;
        else
            newmap(j,i,1) = 1;
            newmap(j,i,2) = 0;
        end
    end
end
imshow(newmap,'InitialMagnification','fit');figure;imshow(truth,'InitialMagnification','fit');