function [pose_gps_hist, pose_est_hist, P] = KalmanFilter(u, pose_new, pose_actual, Q, R, mu_gps, mu_u, A, B, P, pose_gps_hist, pose_est_hist)

u_bar = u + randn(1,4)*Q + mu_u;
pose_gps = pose_new + randn(1,4)*R + mu_gps;
pose_gps_hist = [pose_gps_hist; pose_gps];

% ------------------- KALMAN FILTER ----------------------------------

% Prediction step
poseEstPred = A*pose_actual' + B*u_bar';
Ppred = A*P*A' + B*Q^2*B';

% Update step
H = eye(4);
InnCov = H*Ppred*H' + R^2;
W = Ppred*H'*inv(InnCov);
poseEst = (poseEstPred + W*(pose_gps'-H*poseEstPred))';
P = (eye(4) - W*H)*Ppred;
pose_est_hist = [pose_est_hist; poseEst];

end