function [turn] = PiTurn(current,next,row)
%Pi Turn
global RL N Rmin numpoints
if row>=N+2 %top turning
    if current(1)-next(1)>0 
        % going right to left at top
        a = linspace(pi/2,0,numpoints); %first turn
        x1 = Rmin*sin(a)+current(1)-Rmin;
        y1 = Rmin*cos(a)+RL;

        c = linspace(0,-pi/2,numpoints); %second turn
        x3 = (Rmin*sin(c))+next(1)+Rmin;
        y3 = Rmin*cos(c)+RL;

        x2 = linspace(x1(end),x3(1),numpoints); %straight path
        y2 = linspace(y1(end),y1(end),numpoints);

        turn = [x1 x2 x3; y1 y2 y3];
    else
        % going left to right at top
        a = linspace(-pi/2,0,numpoints); %first turn
        x1 = (Rmin*sin(a))+current(1)+Rmin;
        y1 = Rmin*cos(a)+RL;

        c = linspace(0,pi/2,numpoints); %second turn
        x3 = (Rmin*sin(c))+next(1)-Rmin;
        y3 = Rmin*cos(c)+RL;

        x2 = linspace(x1(end),x3(1),numpoints); %straight path
        y2 = linspace(y1(end),y1(end),numpoints);

        turn = [x1 x2 x3; y1 y2 y3];
    end
else %bottom
    if current(1)-next(1)>0 %right to left
        a = linspace(pi/2,pi,numpoints); %first turn
        x1 = Rmin*sin(a)+current(1)-Rmin;
        y1 = Rmin*cos(a);

        c = linspace(pi,3*pi/2,numpoints); %second turn
        x3 = (Rmin*sin(c))+next(1)+Rmin;
        y3 = Rmin*cos(c);

        x2 = linspace(x1(end),x3(1),numpoints); %straight path
        y2 = linspace(y1(end),y1(end),numpoints);

        turn = [x1 x2 x3; y1 y2 y3];
    else %left to right
        a = linspace(3*pi/2,pi,100); %first turn
        x1 = Rmin*sin(a)+current(1)+Rmin;
        y1 = Rmin*cos(a);

        c = linspace(pi,pi/2,numpoints); %second turn
        x3 = (Rmin*sin(c))+next(1)-Rmin;
        y3 = Rmin*cos(c);

        x2 = linspace(x1(end),x3(1),numpoints); %straight path
        y2 = linspace(y1(end),y1(end),numpoints);

        turn = [x1 x2 x3; y1 y2 y3];
    end
end
%     figure
%     hold on
%     for i = 1:length(turn)
%         scatter(turn(1,i),turn(2,i));
%         pause(0.01);
%     end
end

