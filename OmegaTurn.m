function [points] = OmegaTurn(current,next,currentrow,nextrow)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global RL N Rmin W

if currentrow>=N+2 %Top of field
        xstart = current(1); %- 4.5 + W/2 + (currentrow - (N + 2))*W;
        ystart = current(2); 
else %bottom of field
        xstart = current(1);% - 4.5+ W/2 + (currentrow - 2)*W; 
        ystart = current(2); 
end 
    

gamma = acos(1 - ((2*Rmin + W)^2)/(8*Rmin^2)); %rad, bisecting angle between centers of all 3 turn segments 
alpha = (pi - gamma)/2; %rad, corner angles of base of triangle connecting centers of all 3 turn segments 

a1 = alpha/8; %rad, angular step size for the first part of the turn 
a2 = (2*pi - gamma)/42; %rad, angular step size for the second part of the turn 
a3 = a1; %rad, angular step size for the third part of the turn 

%calculate sign for path calculations, assuming CLOCKWISE turn (CCW is accounted for later)
if currentrow>N+1 %positive if at northern end of field 
    sign = 1; 
else %negative if at south of field 
    sign = -1; 
end 

%first turn, facing away from destination 
j = 1; %counter 
turn1x = zeros(1, length(0:a1:alpha));
turn1y = turn1x;
for phi = 0:a1:alpha %calculate (x,y) for first segment of pi turn 
    turn1x(j) = xstart - sign*Rmin + sign*Rmin*(cos(phi)); 
    turn1y(j) = ystart + sign*Rmin*sin(phi); 
    j = j + 1; 
end 

%second turn, looping around to face towards destination 
j = 1; %counter
turn2x = zeros(1, length((3*pi/2 - gamma/2):-a2:(-pi/2 + gamma/2)));
turn2y = turn2x;
for phi = (3*pi/2 - gamma/2):-a2:(-pi/2 + gamma/2)
    turn2x(j) = xstart + sign*W/2 + sign*Rmin*cos(phi); 
    turn2y(j) = ystart + sign*2*Rmin*sin(alpha) + sign*Rmin*sin(phi); 
    j = j + 1; 
end

%third turn, straightening out to enter desired row 
j = 1; %counter 
turn3x = zeros(1, length(alpha:-a3:0));
turn3y = turn3x;
for phi = alpha:-a3:0 
    turn3x(j) = xstart + sign*W + sign*Rmin - sign*Rmin*cos(phi); 
    turn3y(j) = ystart + sign*Rmin*sin(alpha) - sign*Rmin*sin(alpha - phi); 
    j = j + 1; 
end 

xpoints = [turn1x'; turn2x'; turn3x']; 
ypoints = [turn1y'; turn2y'; turn3y']; 

%flipping vector around if turn is counterclockwise
if (currentrow >= (N+2)) && (nextrow < currentrow) 
    for i = 1:length(xpoints)
        xpoints(i) = xpoints(1) + (xpoints(1) - xpoints(i)); 
    end 
elseif (currentrow <= (N+1)) && (nextrow > currentrow) 
    for i = 1:length(xpoints) 
        xpoints(i) = xpoints(1) + (xpoints(1) - xpoints(i)); 
    end 
end
points = [xpoints, ypoints];

end

