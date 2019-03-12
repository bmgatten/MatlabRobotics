clear
clc
close all

plantrows = 12;
plants = 10;
[map,ogmap,truth] = NDVIMap(plantrows,plants);
maptest = map;
colidx = floor(linspace(1,1024,plantrows+1));
rowidx = floor(linspace(1,786,plants+1));
%%
truth = repmat(truth,10,1);
newmap = zeros(plants,plantrows,3);
for i = 1:plantrows
    for j = 1:plants
        f = map(rowidx(j):rowidx(j+1),colidx(i):colidx(i+1),:); %iterate through each section/plot of image
        rav = mean2(f(:,:,1));
        gav = mean2(f(:,:,2));
        bav = mean2(f(:,:,3));
        k(j,i,1)=rav;
        k(j,i,2)=gav;
        k(j,i,3)=bav;  
    end
end
%%
truthshape = reshape(truth,[],3); %reshape the truth to be columns
mapshape = reshape(k,[],3);

for w = 1:length(truthshape) %create table with the ground truth 
    if truthshape(w,1) == 0 && truthshape(w,2)== 0.5 %dark green
        gtruth(w) = 1;
    elseif truthshape(w,1) == 1 && truthshape(w,2) == 0.5 %orange
        gtruth(w) = 2;
    else %red
        gtruth(w) = 3;
    end
end
%%
gtruth=gtruth';
%rat1=mapshape(:,1)./mapshape(:,2);
%rat2=mapshape(:,1)./mapshape(:,3); %doesn't work with ratios because
%sometimes ratios are infinity (divide by 0 error)
%rat3=mapshape(:,2)./mapshape(:,3);
T = table(mapshape(:,1),mapshape(:,2),mapshape(:,3));
for i=1:length(mapshape) 
    %first ratio
    if mapshape(i,1)~= 0 && mapshape(i,2)== 0 
        mapshape(i,4)=1E10;
    elseif mapshape(i,1)== 0 && mapshape(i,2)== 0
        mapshape(i,4)= 0;
    else        
        mapshape(i,4)=mapshape(i,1)/mapshape(i,2); %red/green
    end
    %second ratio
    if mapshape(i,1)~= 0 && mapshape(i,3)== 0
        mapshape(i,5)=1E10;
    elseif mapshape(i,1)== 0 && mapshape(i,3)== 0
        mapshape(i,5)= 0;
    else        
        mapshape(i,5)=mapshape(i,1)/mapshape(i,3); %red/blue
    end
    %third ratio
    if mapshape(i,2)~= 0 && mapshape(i,3)== 0
        mapshape(i,6)=1E10;
    elseif mapshape(i,2)== 0 && mapshape(i,3)== 0
        mapshape(i,6)= 0;
    else        
        mapshape(i,6)=mapshape(i,2)/mapshape(i,3); %green/blue
    end
end
subplot(1,2,2);
imshow(map);
title('Nosiy map');
%hold on
subplot(1,2,1);
imshow(ogmap);
title('Ground thruth map');
MDL = fitcdiscr(mapshape,gtruth); %linear disc. to find the model to fit it
%saveCompactModel(MDL,'MAE252');