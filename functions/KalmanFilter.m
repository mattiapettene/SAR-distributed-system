function [pose_gps_hist, pose_est_hist, P] = KalmanFilter(nd, u, pose_new, pose_actual, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist, occupancyMap)

u_bar = u + randn(1,4)*Q + mu_u;
pose_gps = pose_new + randn(1,4)*R + mu_gps;
pose_gps_hist(end+1,:, nd) = pose_gps;

% GPS probability
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
if(rand(1) <= 0.9)
    H = eye(4);
    InnCov = H*Ppred*H' + R^2;
    W = Ppred*H'*inv(InnCov);
    poseEst = (poseEstPred + W*(pose_gps'-H*poseEstPred))';
    P = (eye(4) - W*H)*Ppred;
else
    poseEst = poseEstPred;
    P = Ppred;
end

pose_est_hist(end+1,:, nd) = poseEst;

end