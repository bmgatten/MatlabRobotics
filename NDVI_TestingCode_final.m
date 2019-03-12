close all;
clear all;
clc;

plantrows = 15;
plants = 10;
[map,ogmap,truth] = NDVIMap(plantrows,plants);
maptest = map;
MDL = open('predictionmodel2.mat');
MDL = MDL.MDL;
colidx = floor(linspace(1,1024,plantrows+1));
rowidx = floor(linspace(1,786,plants+1));
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
            newmap(j,i,1) = 1;
            newmap(j,i,2) = 0.5;
        else
            newmap(j,i,1) = 1;
            newmap(j,i,2) = 0;
        end
    end
end
sum=0;

for i=1:3
    for k=1:plantrows
        for j=1:plants
            sum = sum+newmap(j,k,i);
            outmap(1,k,i) = sum/plants;
        end
        sum = 0;
    end
end

figure;
subplot(1,3,3)
imshow(outmap,'InitialMagnification','fit');
title('Filtered image');
subplot(1,3,1)
imshow(truth,'InitialMagnification','fit');
title('Ground truth image');
subplot(1,3,2)
imshow(map,'InitialMagnification','fit');
title('Noisy image');