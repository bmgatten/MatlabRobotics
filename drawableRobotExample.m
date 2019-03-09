%Declaring constants (a long list)
%start with an initial state/vehicle constants
x = 0; %m
y = 0; %m
start = [x,y];
theta = pi/2; %rad
wheelRadius = .5; %m, radius of the tire
L = .5; %m, wheel base
gammaMax = 55 * pi/180; %rad, max turning angle of the front tire
vMax = 4; %m/s, max vehicle speed
lD = 2.2; %m, pure pursuit look ahead distance
skidDR = 0;
skidDF = 0;
tauV = 0;
tauG = 0;
gammaD = 0;
vd = vMax; 

%integration constants
dT = 0.001;          %seconds
DT = 0.01;           %seconds

%orchard constants
endI = 100; %n, number of nodes in TSP 
K = 5; %n, number of rows of trees
W = 3;%m, row width


robot = DrawableRobot(x,y,theta, wheelRadius, L, gammaMax, vMax, lD);

xMax = 50; %m, maximum horizontal and vehicle position
yMax = xMax; %m
xMin = -xMax;
yMin = -yMax; 

x_im = [xMin xMax]; y_im = [yMin yMax];
frame = [x_im, y_im]; %frame to draw the robot

robot.DrawRobot(frame)


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


hold on;
s = 0;
pathPoint = 'b.'; %robot path is in blue dots
for i=1:2000 %move the robot 100X
    redraw = mod(i,C.redrawT); %whether or not to redraw
    prePos = [robot.x, robot.y]; 
    robot.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG)
    if(redraw)
        robot.RedrawRobot(frame, prevPos, pathPoint);
    end
    pause(.1); 
end
%Notice the vehicle has moved up!

%See DrawableRobot.m for more methods

