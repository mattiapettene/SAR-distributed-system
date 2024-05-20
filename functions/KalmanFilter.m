function [pose_gps_hist, pose_est_hist, P] = KalmanFilter(nd, time, u, pose_new, pose_actual, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyMap)

% nd = drone number to save data [1/2/3]
% u = true control vector
% pose_new = true next position vector
% pose_actual = true actual position
% Q = covariance matrix input
% R = covariance matrix GPS
% mu_u = actual measured input noise mean
% mu_gps = actual measured GPS noise mean
% pose_gps_hist = vector collecting all GPS positions
% pose_est_hist = vector collecting all estimated positions
% occupancyMap = tree positions to reduce GPS probability

u_bar = u + randn(1,4)*Q + mu_u;
pose_gps = pose_new + randn(1,4)*R + mu_gps;
pose_gps_hist(time+1,:, nd) = pose_gps;

% GPS probability
pose_gps(pose_gps(1:2) < 1) = 1;
if(interp2(occupancyMap, pose_gps(2), pose_gps(1)))
    GPSProb = 0.4;
else
    GPSProb = 0.9;
end

% ------------------------- KALMAN FILTER --------------------------------

% Prediction step
poseEstPred = A*pose_actual' + B*u_bar';
Ppred = A*P*A' + B*Q^2*B';

% Update step
if(rand(1) <= GPSProb)
    H = eye(4);
    InnCov = H*Ppred*H' + R^2;
    W = Ppred*H'*inv(InnCov);
    poseEst = (poseEstPred + W*(pose_gps'-H*poseEstPred))';
    P = (eye(4) - W*H)*Ppred;
else
    poseEst = poseEstPred;
    P = Ppred;
end

pose_est_hist(time+1,:, nd) = poseEst;

end