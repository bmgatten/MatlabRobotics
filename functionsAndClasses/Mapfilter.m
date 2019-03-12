function [outmap,error] = Mapfilter(noisymap,trutharray,numrow,nplants,print)

% Name: FiltNDVIMap.m
% Author: Guilherme De Moura Araujo, Benjamin Gatten, Vivian Vuong
% Created: 03-11-2019
% Revised: 03-12-2019
% OBS: Final project of UC Davis MAE252 class 
% This function uses the MachineLearning_Training model to filter a noisy map
% Output returned is an array with the mean predictions of each row for the
% filtered map

plantrows = numrow;
plants = nplants;
maptest = noisymap; %Noisy map from NDVI function
print = print;
%MDL = open('predictionmodel2.mat'); 
%MDL = MDL.MDL; %Comment out
MDL = loadCompactModel('MAE252.mat'); %Always use this piece of code instead of lines 16 and 17 -
%'predictionmodel12.mat' is the wrong model and must be used for debugging
%purposes only
colidx = floor(linspace(1,1024,plantrows+1)); %set the output map to size of rows x plants
rowidx = floor(linspace(1,786,plants+1)); %set the output map to size of rows x plants
%% Creates a resized map of the true state of field
newmap = zeros(plants,plantrows,3);

for i = 1:plantrows
    for j = 1:plants
        f = noisymap(rowidx(j):rowidx(j+1),colidx(i):colidx(i+1),:);
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
%% Calculate the error of the predictions based on the truth vector
truthshape = reshape(trutharray,[],3); %reshape the truth to be columns
predicted = reshape(outmap,[],3); %reshape the truth to be columns

for w = 1:length(truthshape) %create array with the ground truth 
    if truthshape(w,1) == 0 && truthshape(w,2)== 0.5 %dark green
        gtruth(w) = 1;
    elseif truthshape(w,1) == 1 && truthshape(w,2) == 0.5 %orange
        gtruth(w) = 2;
    else %red
        gtruth(w) = 3;
    end
end

for w = 1:length(predicted) %create array with the ground truth 
    if predicted(w,1) == 0 && predicted(w,2)== 0.5 %dark green
        pred(w) = 1;
    elseif predicted(w,1) == 1 && predicted(w,2) == 0.5 %orange
        pred(w) = 2;
    else %red
        pred(w) = 3;
    end
end
correct = 0;
sumArray = 0;
for i=1:length(pred)
    if pred(i)==gtruth(i)
        errorArray(i)=1;
    else
        errorArray(i)=0;
    end
    sumArray = sumArray+errorArray(i);
end
error = 1-sumArray/length(errorArray);
%% Plots of ground truth, noisy, and model prediction maps for visual aid
if print
    figure;
    subplot(1,3,3)
    imshow(outmap,'InitialMagnification','fit');
    title('Filtered image');
    subplot(1,3,1)
    imshow(trutharray,'InitialMagnification','fit');
    title('Ground truth image');
    subplot(1,3,2)
    imshow(noisymap,'InitialMagnification','fit');
    title('Noisy image');
end