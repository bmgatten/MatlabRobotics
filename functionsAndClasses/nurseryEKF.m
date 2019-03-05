function [kPose, P] = nurseryEKF(oldkPose, odo, z, oldP, V, W)
%Extended Kalman Filter takes in dead reckoning states, covariance
%matrices, and noisy gps reading and returns new, filtered states and
%updated covariance matrices

%kPose -> kalman filter pose
%odo -> [distance change, angle change]' column vector from model
%z -> [x, y, theta]' column vector from gps/compass noisy
%P -> Estimate of state uncertainty
%V -> estimate of process noise covariance
%W -> estimate of sensor noise covariance

x = oldkPose(1);
y = oldkPose(2);
theta = oldkPose(3);
dDist = odo(1);
dTheta = odo(2);

%Estimate the new state w/ nonlinear dead reckoning
kPose = [x + dDist*cos(theta); y + dDist*cos(theta); theta + dTheta];
%Calculate Jacobians
Fx = [1 0 -dDist*sin(theta+dTheta); 0 1 dDist*cos(theta+dTheta); 0 0 1];
Fv = [cos(theta+dTheta)  -dDist*sin(theta+dTheta); sin(theta+dTheta) dDist*cos(theta+dTheta); 0 1];
%Update the uncertainty matrix
P = Fx*oldP*transpose(Fx) + Fv*V*transpose(Fv);
%Innovation covariance
S = P + W;
%Kalman gain matrix
K = P/S;
%Compute innovation/residuals
v = z - kPose; 
%Update step
kPose = kPose + K*v;
P = P - K*P;
end