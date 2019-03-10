clear
clc
addpath(genpath('functionsAndClasses'));
addpath(genpath('rvctools'));
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
global RL N Rmin numpoints W dT DT

numpoints = 50;
N = 10; %number of rows
W = 2.5; %m, row width
%W = 2.5; %implement width m
RL = 20; %row length m
gammaMax = deg2rad(60); %degrees
GammaMin = -gammaMax;
L = 3; %wheel base m
x = [-W, W/2:W:(N)*W, W/2:W:(N)*W, -W];
y = [RL/2, zeros(1,N), RL*ones(1,N), RL/2];
xy = [x;y].';
Rmin = L/tan(gammaMax);

%ogroute=route;
% for it = 1:N
%     if truth(1,it,1)==0 && truth(1,it,2)==0.5 %%green
%         idxgreen = find(route==it+1)
%         route(idxgreen)=[];
%         idxgreen2 = find(route==it+1+N)
%         route(idxgreen2)=[];
%     end
% end
%% Compute non-turning costs aij 
%Create cost table DMAT
%Moving from ANY lower headland to ANY upper headland
%Only possible if both nodes are in the same field row

huge = 1E10;
DMAT = zeros(2*N+2,2*N+2);

for i = 2:N+1
    for j = N+2:2*N+1
        if (j-i) == N
            DMAT(i,j) = -huge/2;
            DMAT(j,i) = -huge/2;
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
[map,ogmap,truth] = NDVIMap(N,1);
ogDMAT = DMAT;
green = [];
original = 1:2*N+2;
modxy = xy;
for it = 1:N
    if truth(1,it,1)==0 && truth(1,it,2)==0.5 %%green
        green = cat(2,green,it);
        original(original==it+1)=[];
        original(original==it+1+N)=[];
        DMAT(it+1,:)=Inf;
        DMAT(it+1+N,:)=Inf;
        DMAT(:,it+1)=Inf;
        DMAT(:,it+1+N)=Inf;
        modxy(it+1,:)=Inf;
        modxy(it+1+N,:)=Inf;
    end
end

DMAT = DMAT(~isinf(DMAT));
DMAT = reshape(DMAT,[(2*N+2)-2*length(green) (2*N+2)-2*length(green)]);
%%
modxy = modxy(~isinf(modxy));
modxy = reshape(modxy,[2 (2*N+2)-2*length(green)]);

%%
t = cputime;
resultStruct = tspof_ga('XY',modxy','DMAT',DMAT,'SHOWRESULT',false,'SHOWWAITBAR',false,'SHOWPROG',false);
E = cputime-t;
route = [1 resultStruct.optRoute 2*N+2];
resultStruct.minDist
%%
route(end)=length(route);
%[~,routesort]=sort(route); %Get the order of B
route=original(route);

%% Path making
path = [];
j = 0;
%for i = 1:2*N+1
for i = 1:length(route)-1
    if i<length(route)%i<2*N+2
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
    elseif i==length(route)-1 %i==2*N+1
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
%scatter(path(1,:),path(2,:));
 %%
%     figure
%     axis([-10 20 -5 30]);
%     hold on
%     scatter(path(1,:),path(2,:))
%     for i = 1:length(path)
%         scatter(path(1,i),path(2,i));
%          pause(0.0000000001);
%     end
%% Create a robot object to traverse the given path

start = [path(1,1), path(2,1)]; %m, starting position/coordinates of the robot
vd = 1; %m/s, velocity
%integration constants
dT = .001;%s, small integration constnat
DT = .1;%s, large integration constant
endI = 2*N+1; %? number of nodes in TSP
K = N; %Number of tree rows

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
pathPoint = 'b.'; %robot path is in blue dots
%% Instantiate a robot object. 
%starting point of the robot
x = path(1,1);
y = path(2,1);
theta = atan2(path(2,2)-y,path(1,2)-x);
vMax = vd; %m/s, max velocity of the robot
wheelRadius = .5; %m
lD = 2; %m
% lD = sqrt((path(1,1) - start(1)).^2 + (path(2,1) - start(2)).^2); %m, distance to first navigatable point
%instantiate robot object
robot = DrawableRobot(x,y,theta, wheelRadius, L, gammaMax, vMax, lD);

%more constants
skidDR = 0;
skidDF = 0;
tauG = 0.1;          %s, steering lag
tauV = 0.2;          %s, velocity lag

%maximum and minimum positions
xMax = N*W + 10; %m, maximum horizontal and vehicle position
yMax = RL + 10; %m
xMin = -10; %m
yMin = -10; %m

x_im = [xMin xMax]; y_im = [yMin yMax];
frame = [x_im, y_im]; %frame to draw the robot

robot.DrawRobot(frame)
%% Traverse the path with the robot.
robot.dim = [0.5, 1.2] ;%1/2length, 1/2width i.e. 3m long, 2.4m wide
prev = 1;    %previous point on the path to follow
planner = PathPlanner(path);
%indicies of various parameters
X = 1;
Y = 2;
THETA = 3;
GAMMA = 4;
VEL = 5;
%initialize starting pose for the robot
kPose = [start(X); start(Y); theta];
PRINT_MAP = true;
if PRINT_MAP
  lookAhead = plot(path(X, 3), path(Y, 3), 'k*');
end
%% Find covariance matrices
%State limits
Qmax = zeros(1, 5);
Qmax(X) = Inf; Qmax(Y) = Inf; Qmax(THETA) = Inf;
Qmax(GAMMA) = gammaMax; Qmax(VEL) = vd;
Qmin = -Qmax; % symmetrical negative constraints for minimum values
% Control constraints
Umax=[gammaMax vd]';
Umin= -Umax;

Ni = 10000; 
wx = zeros(1,Ni);
wy = zeros(1,Ni);
wtheta = zeros(1,Ni);
odo = zeros(Ni,2);
q = [0 0 0 0 0]; %initialize a robot w/zero for all states.
u = [0 0]; %initialize zero for steering and speed inputs.
for i=1:Ni
    [wx(i), wy(i), wtheta(i)] = GPS_CompassNoisy(q(X), q(Y), q(THETA));
    [q,odo(i,:)] = robot_odo(q, u, Umin, Umax, Qmin, Qmax, L, tauG, tauV); 
end
A = [wx; wy; wtheta]';
B = [odo(:,1) odo(:,2)];
W = cov(A); %covariance of sensor noise
V = cov(B); %covariance of odometry noise
%Initialize uncertainty matrix
P = zeros(3,3);

%initialize robot pose
robot.x = kPose(X);
robot.y = kPose(Y);
robot.theta = kPose(THETA);
%%
%-----------------------MAIN MOTION LOOP---------------------------------
figure(2)
  hold on
  scatter(path(1,:),path(2,:))
for t = 0:C.DT:(C.T - C.DT)
  redraw = mod(t, C.redrawT) == 0;  %whether to redraw this iteration or not

  %robot.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
  savePose = [robot.x, robot.y, robot.theta];
  [xsensed, ysensed, thetasensed] = GPS_CompassNoisy(savePose(X), ...
    savePose(Y), savePose(THETA));

  z = [xsensed; ysensed; thetasensed]; %measurements
  [kPose, P] = nurseryEKF(kPose, odo, z, P, V, W);
  %use estimated position for navigation controller
  robot.x = kPose(X);
  robot.y = kPose(Y);
  robot.theta = kPose(THETA);
  
  [gammaD, ~, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  if PRINT_MAP
    figure(2)
    delete(lookAhead)
    %plot lookahead point
    lookAhead = plot(path(X, prev), path(Y, prev), 'k*');
  end
  
  %use true pose for motion simulation
  q0 = [savePose(X); savePose(Y); savePose(THETA); robot.gamma; robot.v];
  u = [gammaD; vd];
  [q, odo] = robot_odo(q0, u, Umin, Umax, Qmin, Qmax, L, tauG, tauV);
  
  %set robot to new position for drawing
  robot.x = q(X); 
  robot.y = q(Y);
  robot.theta = savePose(THETA) - (q(THETA) - savePose(THETA));
  robot.gamma = q(GAMMA);
  robot.v = q(VEL);
  
  if redraw && PRINT_MAP
    figure(2)
    prevPos = [robot.x, robot.y];
    robot.RedrawRobot(frame, prevPos, pathPoint)
    pause(C.aniPause)
  end
    
  if prev == planner.nPoints && abs(errX) + abs(errY) < C.posEpsilon
    break   % stop if navigating to last path point and position close
  end
end
