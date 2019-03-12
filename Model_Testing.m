% Name: Model_Testing.m
% Author: Guilherme De Moura Araujo, Benjamin Gatten, Vivian Vuong
% Created: 03-11-2019
% Revised: xx-xx-20xx
% OBS: Final project of UC Davis MAE252 class 
% This code tests the MachineLearning_Training model for the NDVIMap function
%%

close all;
clear all;
clc;
%% Inputs
plantrows = 15; %Simulated input
plants = 10; %Simulated input
[map,ogmap,truth] = NDVIMap(plantrows,plants);
maptest = map;
MDL = open('predictionmodel2.mat'); %loadCompactModel(MAE252.m);
MDL = MDL.MDL; %Comment out
colidx = floor(linspace(1,1024,plantrows+1)); %set the output map to size of rows x plants
rowidx = floor(linspace(1,786,plants+1)); %set the output map to size of rows x plants
%% Creates a resized map of the true state of field
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
        % Uses the class probability model (MAE252.m) to predict plant health
        % class (green, orange or red)
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

%% Creates an array with the mean class of each row

sum = 0;

for i=1:3
    for k=1:plantrows
        for j=1:plants
            sum = sum+newmap(j,k,i);
            outmap(1,k,i) = sum/plants;
        end
        sum = 0;
    end
end

%% Plots of ground truth, noisy, and model prediction maps for visual aid
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