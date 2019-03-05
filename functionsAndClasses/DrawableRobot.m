classdef DrawableRobot < handle
  properties
    x          %x position in world frame [meters]
    y          %y position in world frame [meters]
    theta      %angle relative to world frame [radians]
    v          %current forward speed of wheels [m/s]
    gamma      %current wheel turn angle (bicycle model) [radians]
    Rw         %wheel radius [meters]
    l          %disance between wheels [meters]
    gammaMax   %max steering angle [radians]
    vMax       %max linear speed   [m/s]
    Ld         %min distance to first navigatable point [meters]
    dim        %[length, width] of robot [meters]
  end
  properties (SetAccess = private)
    plotHandle
    robotFrameX
    robotFrameY
    framepoints
    posEpsilon   %position allowed error [meters]
    velEpsilon   %velocity allowed error [m/s]
    reverse
    pi_2
  end
  methods
    function obj = DrawableRobot(varargin)
      if nargin > 7
        obj.Ld = varargin{8};
      end
      if nargin > 6
        obj.x = varargin{1};
        obj.y = varargin{2};
        obj.theta = varargin{3};
        obj.Rw = varargin{4};
        obj.l = varargin{5};
        obj.gammaMax = varargin{6};
        obj.vMax = varargin{7};
      elseif nargin > 2
        obj.x = varargin{1};
        obj.y = varargin{2};
        obj.theta = varargin{3};
      else
        obj.x = 0;
        obj.y = 0;
        obj.theta = 0;
        obj.Rw = 0.5;
        obj.l = 1;
        obj.gammaMax = pi/4;
        obj.vMax = 5;
      end
      obj.dim = [1, 0.8]; %2 meters long, 1.6 m wide
      obj.posEpsilon = 0.15;
      obj.velEpsilon = 0.05;
      obj.gamma = 0;
      obj.v = 0;
      obj.reverse = false;
      obj.pi_2 = pi/2;
      obj.framepoints = [0.12, 0.2067, 0.2933, 0.38, 0.6, 0.7, 0.8, ...
        0.9]'*2*pi - pi/2;
      %obj.framepoints = [0.12, 0.2067, 0.2933, 0.38, 0.62, 0.7067,...
      %  0.7933, 0.88]'*2*pi;
    end
    
    function DrawRobot(obj, frame, exact)
      if nargin < 2
        fprintf('DrawRobot: Must have frame to draw robot.\n')
      end
      fPoints = obj.framepoints + obj.theta;
      obj.robotFrameX = obj.dim(2) * cos(fPoints) + obj.x;
      obj.robotFrameY = obj.dim(1) * sin(fPoints) + obj.y;
      
      obj.plotHandle = fill(obj.robotFrameX, obj.robotFrameY, 'r');
      if nargin > 2 && exact
        xlim([frame(1), frame(2)])
        ylim([frame(3), frame(4)])
        axis square
      else
        frameSize = max(abs(frame(2) - frame(1)), abs(frame(4) - frame(3)));
        xlim([frame(1), frame(1) + frameSize])
        ylim([frame(3), frame(3) + frameSize])
        axis square
      end
    end
    
    function MoveRobot(obj, dx, dy, dTheta, dv, dGamma)
      obj.x = obj.x + dx;
      obj.y = obj.y + dy;
      obj.theta = obj.theta + dTheta;
      if nargin > 4
        obj.v = obj.v + dv;
      end
      if nargin > 5
        obj.gamma = obj.gamma + dGamma;
      end
    end
    
    function MoveRobotTo(obj, newX, newY, newTheta)
      obj.x = newX;
      obj.y = newY;
      obj.theta = newTheta;
    end
    
    function MoveGeometricDiff_RedrawRobot(obj, Wl, Wr, dt, frame, pathPoint)
      prevPos = [obj.x, obj.y];
      [dx, dy, dTheta] = GeometricDiffFK(Wl, Wr, obj.Rw, obj.l,...
        obj.theta, dt);
      obj.MoveRobot(dx, dy, dTheta)
      obj.RedrawRobot(frame, prevPos, pathPoint)
      
    end
    
    function MoveEulerDiff_RedrawRobot(obj, Wl, Wr, dt, Sl, Sr, skidD, frame,...
        pathPoint)
      prevPos = [obj.x, obj.y];
      [dx, dy, dTheta] = EulerDiffFK(Wl, Wr, obj.Rw, obj.l,...
        obj.theta, dt, Sl, Sr, skidD);
      obj.MoveRobot(dx, dy, dTheta)
      obj.RedrawRobot(frame, prevPos, pathPoint)
    end
    
    function MoveEulerDiff_MaybeRedraw(obj, Wl, Wr, C, Sl, Sr, skidD, frame,...
        pathPoint, prevPos, redraw)
      if nargin < 8
        prevPos = [obj.x, obj.y];
      end
      [dx, dy, dTheta] = EulerDiffFK(Wl, Wr, obj.Rw, obj.l,  obj.theta, ...
        C.DT, Sl, Sr, skidD);
      obj.MoveRobot(dx, dy, dTheta)
      if nargin < 8 || redraw
        obj.RedrawRobot(frame, prevPos, pathPoint)
        pause(C.aniPause)
      end
    end
    
    function MoveEulerAck_RedrawRobot(obj, gammaD, vd, C, s, skidDR, skidDF,...
        tauV, tauG, frame, prevPos, pathPoint, redraw)
      obj.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
      if redraw
        obj.RedrawRobot(frame, prevPos, pathPoint)
        pause(C.aniPause)
      end
    end
    
    function MoveToEulerAck_RedrawRobot(obj, xd, yd, C, s, skidDR,...
             skidDF, tauV, tauG, frame, pathPoint, redraw)
      
      for t = 0:C.DT:(C.T - C.DT)
        if redraw
          prevPos = [obj.x, obj.y];
        end
        errX = xd - obj.x;
        errY = -(yd - obj.y);
        thetaD = atan2(errY, errX);
        vd = 0;
        if abs(errX) + abs(errY) >= obj.posEpsilon
          vd = C.Kv * sqrt(errX ^ 2 + errY ^ 2);
        end
        gammaD = C.Kh * angdiff(thetaD, obj.theta);

        obj.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
        
        if redraw
          obj.reverse = true;
          obj.RedrawRobot(frame, prevPos, pathPoint)
          obj.reverse = false;
          pause(C.aniPause / 400)
        end
        
        if abs(errX) + abs(errY) < obj.posEpsilon && vd < obj.velEpsilon
          break   % stop if speed slow enough and position close enough
        end
      end
    end
    
    function Move_EulerAckFK(obj, gammaD, vd, C, s, skidDR, skidDF, tauV, tauG)
      ep = 1e-8;                          %minimum tau

      for t = 0:C.dt:(C.DT - C.dt)
        V = vd;                           %wheel forward speed
        Vl = V * (1 - s);                 %Left wheel forward ground speed
        Vy = Vl * tan(skidDR);            %perpendicular skid speed
        cosT = cos(obj.theta);
        sinT = sin(obj.theta);

        %change in position in world frame
        obj.x = obj.x + C.dt * (Vl * cosT - Vy * sinT);
        obj.y = obj.y + C.dt * (Vl * sinT + Vy * cosT);
        %change in angle to world frame
        obj.theta = obj.theta - (Vl * tan(obj.gamma + skidDF) - Vy)...
          / obj.l * C.dt;

        dv = (vd - obj.v) * C.dt;
        if tauV > ep
          dv = dv / tauV;
        end
        obj.v = max(min(obj.v + dv, obj.vMax), -obj.vMax);

        dGamma = (gammaD - obj.gamma) * C.dt;
        if tauG > ep
          dGamma = dGamma / tauG;
        end
        obj.gamma = max(min(obj.gamma + dGamma, obj.gammaMax), -obj.gammaMax);
      end

    end
        
    function MoveTo_RedrawRobot(obj, newX, newY, newTheta, frame, pathPoint)
      prevPos = [obj.x, obj.y];
      obj.MoveRobotTo(newX, newY, newTheta)
      obj.RedrawRobot(frame, prevPos, pathPoint)
    end
    
    function RedrawRobot(obj, frame, prevPos, pathPoint)
      if obj.reverse
        fPoints = obj.framepoints - obj.theta - pi;
      else
        fPoints = obj.framepoints + obj.theta;
      end
      
      obj.robotFrameX = cos(fPoints) * 0.8 + obj.x;
      obj.robotFrameY = sin(fPoints) + obj.y;
      
      delete(obj.plotHandle)
      if nargin > 2
        hold on
        plot(prevPos(1), prevPos(2), pathPoint)
      end
      obj.plotHandle = fill(obj.robotFrameX, obj.robotFrameY, 'r');
      frameSize = max(abs(frame(2) - frame(1)), abs(frame(4) - frame(3)));
      xlim([frame(1), frame(1) + frameSize])
      ylim([frame(3), frame(3) + frameSize])
      axis square
    end
    
    %------------------HELPERS----------------------------
    function result = inFront(obj, point)
      Tr = se2(obj.x, obj.y, obj.theta - obj.pi_2);
      Pr = homtrans(inv(Tr), point);
      if Pr(2) > 0
        result = true;
      else
        result = false; 
      end
    end
    
    function wPoint = WorldPoint(obj, point)
      Tr = se2(obj.x, obj.y, obj.theta - obj.pi_2);
      wPoint = homtrans(Tr, point);
    end
    
    function rPoint = RobotPoint(obj, point)
      Tr = se2(obj.x, obj.y, obj.theta - obj.pi_2);
      rPoint = homtrans(inv(Tr), point);
    end
    
  end
end