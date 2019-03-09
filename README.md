# MatlabRobotics
## UC Davis MAE 252 Class Project
### Using Protected Matlab Files 

```Matlab
function [q_true_next, odo] = robot_odo(q_true, u, umin, umax,Qmin, Qmax, L, tau_gamma, tau_v)
% model of a vehicle with closed loop steering and velocity control
% the combined effect of steering/vehicle inertia and control is
% a first order system for steering (tau_phi) and velocity (tau_v)
% state is
% q(1) -> x
% q(2) -> y
% q(3) -> theta (orientation in world frame)
% q(4) -> gamma (steering angle)
% q(5) -> linear velocity
 
% inputs are:
% u(1) -> desired steering angle
% u(2) -> desired linear velocity
%
% the model returns the next state q1 and noisy odometry, i.e.,
% distance traveled in DT in odo(1) and angle change in DT in odo(2).
% note:robot_odo relies on global integration constants dT and DT. 



function [ x_n, y_n, theta_n ] = GPS_CompassNoisy( x, y, theta )
% function accepts as arguments the true pose of the robot (from q_true) and returns a noisy measurement of the pose. Angle is in radians.
```
