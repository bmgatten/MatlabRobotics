classdef PathPlanner < handle
  properties
    rMin       %distance between row-centers
    rowWidth   %minimum turning radius
  end
  properties (SetAccess = private)
    path_         %Defined as [x0 x1 x2 ... ]
                  %           [y0 y1 y2 ... ]
    nPoints       %number of coordinate pairs in path
    rMinMult      %multiplier to account for sterring delay
  end
  methods
    function obj = PathPlanner(varargin)
      %{1} = path
      %{2} = min turn radius
      %{3} = row-center distance
      if nargin > 2
        obj.rowWidth = varargin{3};
        obj.rMin = varargin{2};
      end
      if nargin > 0
        obj.path_ = varargin{1};
        obj.nPoints = size(obj.path_, 2);
      else
        obj.path_ = zeros(2, 3);
        obj.nPoints = 3;
      end
      obj.rMinMult = 1.2;
    end
    
    %Pure Pursuit: Find first point in front of robotf
    function [steerAngle, trackErr, first, Ex, Ey] = FirstFeasiblePoint(obj,...
        robot, previousIndex)
      trackErr = Inf;
      first = previousIndex;
      startMin = Inf;
      X = 1; Y = 2;
      stride = 12;
      endI = min(obj.nPoints, previousIndex+stride);
      startI = max(previousIndex-stride, 1);
      for i = startI:endI
        di = sqrt(abs(obj.path_(X, i) - robot.x) ^ 2 + ...
                  abs(obj.path_(Y, i) - robot.y) ^ 2 );
        if di < trackErr
          trackErr = di;
        end
        if i >= previousIndex
          pointD = abs(di - robot.Ld);
          if robot.inFront(obj.path_(:, i)) && pointD < startMin
            startMin = pointD;
            first = i;
          end
        end
      end
      
      
      Pr = robot.RobotPoint(obj.path_(:, first));
      Ey = Pr(X);
      Ex = -Pr(Y);
      steerAngle = atan(robot.l * 2 * Ey / (robot.Ld ^ 2));
      %{
      if first ~= previousIndex
        fprintf('.')
      end
      %}
    end
    
    function points = GenerateTurnPath(obj, mult, x, y, type, ...
        varargin)
      %{1} = d
      %{2} = Rmin
      %{3} = w
      [rm, w, dw] = obj.GetRWD(varargin);
      rMinNew = rm * obj.rMinMult;
      if ~(strcmp(type, 'bl') || strcmp(type, 'br') || strcmp(type, 'tl') ||...
           strcmp(type, 'tr'))
        fprintf('Invalid turn direction: %s\n', type);
        points = [0;0];
      else
        if dw > 2 * rm
          points = obj.GeneratePiTurnPath(mult, x, y, type, rMinNew, w, dw);
        else
          points = obj.GenerateOmegaTurnPath(mult, x, y, type, rMinNew, w, dw);
        end
      end
      
    end
    
    function points = GeneratePiTurnPath(obj, mult, x, y, type, rm, w, dw)
      straight = ceil((dw - 2 * rm) * 1.5 * mult);
      arc = ceil(2 * rm * mult);
      if strcmp(type, 'bl')       %bottom west (left) turn
        angles1 = linspace(0, -pi / 2, arc);
        angles2 = linspace(-pi / 2, -pi, arc);
        points = [[(rm * (cos(angles1) - 1)), linspace(-rm, -dw + rm, ...
                   straight), (rm * cos(angles2)) - dw + rm] + x;...
          [(rm * sin(angles1)), (zeros(1, straight) - rm),...
           (rm * sin(angles2))] + y];
      elseif strcmp(type, 'br')   %bottom east (right) turn
        angles1 = linspace(-pi / 2, -pi, arc);
        angles2 = linspace(0, -pi / 2, arc);
        points = [[(rm * (cos(angles1) + 1)), linspace(rm, dw - rm, ...
                   straight), (rm * cos(angles2)) + dw - rm] + x;...
          [(rm * sin(angles1)), (zeros(1, straight) - rm),...
           (rm * sin(angles2))] + y];
      elseif strcmp(type, 'tl')   %top left turn
        angles1 = linspace(0, pi / 2, arc);
        angles2 = linspace(pi / 2, pi, arc);
        points = [[(rm * (cos(angles1) - 1)), linspace(-rm, -dw + rm, ...
                   straight), (rm * cos(angles2)) - dw + rm] + x;...
          [(rm * sin(angles1)), (zeros(1, straight) + rm),...
           (rm * sin(angles2))] + y];
      else                  %top right turn
        angles1 = linspace(pi, pi / 2, arc);
        angles2 = linspace(pi / 2, 0, arc);
        points = [[(rm * (cos(angles1) + 1)), linspace(rm, dw - rm, ...
                   straight), (rm * cos(angles2)) + dw - rm] + x;...
          [(rm * sin(angles1)), (zeros(1, straight) + rm),...
           (rm * sin(angles2))] + y];
      end
    end
    
    function points = GenerateOmegaTurnPath(obj, mult, x, y, type, rm, w, dw)
      gamma = acos(1 - (2 * rm + dw) ^ 2 / (8 * rm ^ 2));
      alpha = (pi - gamma) / 2;
      alphaArc = ceil(1.1 * rm * alpha * mult);  %points on alpha arcs
      gammaArc = ceil(8 * rm / gamma * mult);
      gArcOffset = sqrt((2 * rm)^2 - (rm + dw / 2)^2);
      if strcmp(type, 'bl') || strcmp(type, 'br')
        anglesL = linspace(0, -alpha, alphaArc);
        anglesM = linspace(-pi - alpha, alpha, gammaArc);
        anglesR = linspace(pi + alpha, pi, alphaArc);
        if strcmp(type, 'bl')
          xL = x - dw;
        else
          xL = x;
        end
        points = [[(rm * (cos(anglesL) - 1)), (rm * cos(anglesM) + dw / 2), ...
                   (rm * cos(anglesR) + dw + rm)] + xL;...
          [(rm * sin(anglesL)), (rm * sin(anglesM) - gArcOffset), ...
           (rm * sin(anglesR))] + y];
        
        if strcmp(type, 'bl')       %bottom west (left) turn
          points = fliplr(points);
        end
      else
        anglesL = linspace(0, alpha, alphaArc);
        anglesM = linspace(pi + alpha, -alpha, gammaArc);
        anglesR = linspace(pi - alpha, pi, alphaArc);

        if strcmp(type, 'tl')       %top west (left) turn
          xL = x - dw;
        else                  %top east (right) turn
          xL = x;
        end
        points = [[(rm * (cos(anglesL) - 1)), (rm * cos(anglesM) + dw / 2), ...
                   (rm * cos(anglesR) + dw + rm)] + xL;...
          [(rm * sin(anglesL)), (rm * sin(anglesM) + gArcOffset), ...
           (rm * sin(anglesR))] + y];
        
        if strcmp(type, 'tl')       %bottom west (left) turn
          points = fliplr(points);
        end
      end
    end
    
    %--------------GETTERS and SETTERS-------------------
    function path = Path(obj)
      path = obj.path_;
    end
    function newPath(obj, path)
      obj.path_ = path;
      obj.nPoints = length(path, 2);
    end
    
    %---------------HELPERS------------------------------
    function [rm, w, dw] = GetRWD(obj, varargin)
      v = varargin{1};
      n = length(v);
      if n > 2
        rm = v{2};
        w = v{3};
      else
        rm = obj.rMin;
        w = obj.rowWidth;
      end
      if n > 0
        dw = v{1} * w;  %distance between d rows
      else
        dw = w;      %d = 1
      end
    end
    
  end
end