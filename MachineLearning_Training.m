% Name: MachineLearning_Training.m
% Author: Guilherme De Moura Araujo, Benjamin Gatten, Vivian Vuong
% Created: 03-11-2019
% Revised: xx-xx-20xx
% OBS: Final project of UC Davis MAE252 class 
% This code uses the NDVIMap function to generate a random map and uses the
% information to create a class probability function model
%%

close all;
clear all;
clc;
%% Inputs
plantrows = 12; %Simulated input
plants = 10; %Simulated input
[map,ogmap,truth] = NDVIMap(plantrows,plants); %outputs are true and noisy map (sensor)
maptest = map;
colidx = floor(linspace(1,1024,plantrows+1)); %set the output map to size of rows x plants
rowidx = floor(linspace(1,786,plants+1)); %set the output map to size of rows x plants
%% Creates a resized map of the true state of field
truth = repmat(truth,10,1);
newmap = zeros(plants,plantrows,3);
for i = 1:plantrows
    for j = 1:plants
        f = map(rowidx(j):rowidx(j+1),colidx(i):colidx(i+1),:); %iterate through each section/plot of image
        rav = mean2(f(:,:,1)); % 6 set of predictors: R, G, B and rates R/G, R/B, G/B
        gav = mean2(f(:,:,2));
        bav = mean2(f(:,:,3));
        k(j,i,1)=rav;
        k(j,i,2)=gav;
        k(j,i,3)=bav;  
    end
end
%% Calculate R, G, and B predictors
truthshape = reshape(truth,[],3); %reshape the truth to be columns
mapshape = reshape(k,[],3);

for w = 1:length(truthshape) %create array with the ground truth 
    if truthshape(w,1) == 0 && truthshape(w,2)== 0.5 %dark green
        gtruth(w) = 1;
    elseif truthshape(w,1) == 1 && truthshape(w,2) == 0.5 %orange
        gtruth(w) = 2;
    else %red
        gtruth(w) = 3;
    end
end
%% Calculate R/G, R/B, and G/B predictors

gtruth=gtruth';

T = table(mapshape(:,1),mapshape(:,2),mapshape(:,3)); %Tabulate info
for i=1:length(mapshape) 
    %first ratio red/green
    if mapshape(i,1)~= 0 && mapshape(i,2)== 0 %check for errors
        mapshape(i,4)=1E10;
    elseif mapshape(i,1)== 0 && mapshape(i,2)== 0 %check for errors
        mapshape(i,4)= 0;
    else        
        mapshape(i,4)=mapshape(i,1)/mapshape(i,2); 
    end
    %second ratio red/blue
    if mapshape(i,1)~= 0 && mapshape(i,3)== 0 %check for errors
        mapshape(i,5)=1E10;
    elseif mapshape(i,1)== 0 && mapshape(i,3)== 0 %check for errors
        mapshape(i,5)= 0;
    else        
        mapshape(i,5)=mapshape(i,1)/mapshape(i,3);
    end
    %third ratio green/blue
    if mapshape(i,2)~= 0 && mapshape(i,3)== 0 %check for errors
        mapshape(i,6)=1E10;
    elseif mapshape(i,2)== 0 && mapshape(i,3)== 0 %check for errors
        mapshape(i,6)= 0;
    else        
        mapshape(i,6)=mapshape(i,2)/mapshape(i,3);
    end
end

%% Plots of ground truth and noisy maps for visual aid

subplot(1,2,2);
imshow(map);
title('Nosiy map');
subplot(1,2,1);
imshow(ogmap);
title('Ground thruth map');

%% Creates class probability function model

MDL = fitcdiscr(mapshape,gtruth); %linear disc. to find the model to fit it
%mapshape are the predictors and gtruth is the true state
%saveCompactModel(MDL,'MAE252'); %saves model in file directory to post
%use in different projects