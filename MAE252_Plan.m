clear
clc
%% Place (0,0) at the south west corner of the field
%Rectangular field
%N = 5 rows, RL = 20 m, W = 2.5 m,
%Initial and final tractor positions are (-W and RL/2) - theta unimportant
%Start and end at the same point
%Wheelbase L = 3 m
%Steering angle limit is +/- 60 degrees
%% Create two vectors x and y - storing the coordinates of all the nodes
%Distance W/2
%x(i) = x(i-1)+w -- x(N+1) = x(N)+w
global RL N Rmin numpoints W

numpoints = 50;
N = 5; %number of rows
W = 2.5; %implement width m
RL = 20; %row length m
GammaMax = deg2rad(60); %degrees
GammaMin = -GammaMax;
L = 3; %wheel base m
x = [-W, W/2:W:(N)*W, W/2:W:(N)*W, -W];
y = [RL/2, zeros(1,N), RL*ones(1,N), RL/2];
xy = [x;y].';
Rmin = L/tan(GammaMax);
%% Compute non-turning costs aij 
%Create cost table DMAT
%Moving from ANY lower headland to ANY upper headland
%Only possible if both nodes are in the same field row

huge = 1E10;
DMAT = zeros(2*N+2,2*N+2);

for i = 2:N+1
    for j = N+2:2*N+1
        if (j-i) == N
            DMAT(i,j) = 0;
            DMAT(j,i) = 0;
        else
            DMAT(i,j) = huge;
            DMAT(j,i) = huge;
        end
    end
end
%% Compute turning costs

for i = 2:N
    for j = i+1:N+1
        d = abs(i-j);
        if(Rmin<=d*W/2) %PI Turn
            DMAT(i,j) = d*W + ((pi-2)*Rmin);
        else %omega turn
            cost = ((2*Rmin+d*W)^2)/(8*Rmin^2);
            DMAT(i,j) = 3*pi*Rmin - 2*Rmin*acos(1-cost);
        end
        DMAT(j,i) = DMAT(i,j);
        DMAT(i+N,j+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
        DMAT(j+N,i+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
    end
end
%% Compute costs aij involving start/end nodes
%Approximate using the Manhattan distance
%|x1-x2|+|y1+y2|
for i = 2:2*N+1
    DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i);
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i);
end
%% Make start and end nodes big
DMAT(1,2*N+2) = huge; %going from 1 - 2*N+2 huge
DMAT(2*N+2,1) = huge; %going from 2*N+2 - 1 huge
%%
t = cputime;
resultStruct = tspof_ga('XY',xy,'DMAT',DMAT,'SHOWRESULT',false,'SHOWWAITBAR',false,'SHOWPROG',false);
E = cputime-t;
route = [1 resultStruct.optRoute 2*N+2];
resultStruct.minDist
%% Path making
path = [];
j = 0;
for i = 1:2*N+1
    
    if i<2*N+2
        d = route(i)-route(i+1);
        current = xy(route(i),:);
        next = xy(route(i+1),:);
    else
        d = 0;
        current = xy(route(i),:);
        next = 0;
    end
    
    if i == 1
        path = [current(1)*ones(1,numpoints);linspace(current(2),next(2),numpoints)];
        turnpath = PiTurn(current,next,route(i+1));
        path = [path turnpath];
    elseif i==2*N+1
        pathturn = PiTurn(current,next,route(i));
        pathstraight = [next(1)*ones(1,numpoints);linspace(current(2),next(2),numpoints)];
        pathn = [pathturn pathstraight];
        path = [path pathn];
    else
        if abs(d)==N
            pathn = [current(1)*ones(1,numpoints);linspace(current(2),next(2),numpoints)];
        else
            if abs(route(i)-route(i+1))==1
                pathn = OmegaTurn(current,next,route(i),route(i+1));
                pathn=pathn';
            else
                pathn = PiTurn(current,next,route(i+1));
            end
        end
        path = [path pathn];
    end
    
end
scatter(path(1,:),path(2,:));
 %%
    figure
    axis([-10 20 -5 30]);
    hold on
    
    for i = 1:length(path)
        scatter(path(1,i),path(2,i));
        pause(0.00001);
    end


%Define constants
    %integration constants
dT = 0.001;          %seconds
DT = 0.01;           %seconds
%start coordinates
start = [path(1,1), path(2,1)] %m

%todo: verify constants and make sure they are correct

%constants struct (needed for object)
C = struct('W', W,...     %center-to-center row distance [m]
  'swX', 20-W/2, ...      %x offset of southwest corner of grid of trees
  'swY', 20, ...          %y offset of southwest corner of grid of trees
  'L', L,...              %wheelbase [m]
  'start', start, ...     %start coordinates
  'Rw', 0.5,...           %radius [m]
  'Vmax', vd,...          %v max     [m/s]
  'Gmax', gammaMax,...        %gamma max [radians]
  'Ld',   2.2,...         %min distance to first navigatable point [meters]
  'dt',   dT,...          %seconds
  'DT',   DT,...          %seconds
  'T',    600.0,...       %total move to point time allowed
  'RL', 20, ...           %row length [m]
  'HUGE', 10^9,...        %discouraging cost
  'ptsPerMeter', 2,...    %points to plot per meter
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 20, ...   %max range of laser
  'angleSpan', deg2rad(180), ...  %angle span of laser sweep
  'angleStep', deg2rad(0.125), ... %step of laser sweep
  'occThresh', 0.5,...    %occupancy threshold
  'endI', endI,...        %number of nodes
  'K', K, ...             %tree rows
  'Rmin', L / tan(gammaMax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / vd,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );