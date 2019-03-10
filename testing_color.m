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
MDL = fitcdiscr(T,gtruth); %linear disc. to find the model to fit it