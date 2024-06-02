function H_detected = WLS_distributed(cam, n, sigma_cam, m, plt)

% cam = logic matrix with true obstacles detected
% n = number of sensors (cameras)
% sigma_cam = camera standard deviation
% m = number of messages exchanged
% plt = plot yes/no

[x,y] = find(cam);
points = [x,y]; % Real points
pEst_MD = [];
pEst_MH = [];
sStore = cell(size(points,1),1);

% Topology matrix (all connected)
A = ones(n);

for r=1:size(points,1)
    
    % Simulate camera
    s = zeros(2,n);
    for i=1:n
        s(:,i) = points(r,:)' + randn(2,1)*sigma_cam;
    end

    sStore{r} = s;
    
    % Initialization
    F_MD = cell(n,1);
    a_MD = cell(n,1);
    F_MH = cell(n,1);
    a_MH = cell(n,1);
    for i=1:n
        Hi = eye(2);
        Ri = sigma_cam^2*eye(2);
        zi = s(:,i);
        F_MD{i} = Hi'*inv(Ri)*Hi;
        a_MD{i} = Hi'*inv(Ri)*zi;
        F_MH{i} = Hi'*inv(Ri)*Hi;
        a_MH{i} = Hi'*inv(Ri)*zi;
    end
    
    % Degree vector - indicates number of connections for each node
    D = A*ones(n,1);
        
    for k=1:m-1
        FStore_MD = F_MD;
        aStore_MD = a_MD;
        FStore_MH = F_MH;
        aStore_MH = a_MH;
        for i=1:n
            for j=1:n
                if A(i,j) == 1

                    % Maximum Degree Waighting
                    F_MD{i} = F_MD{i} + 1/(1+max(D))*(FStore_MD{j} - FStore_MD{i});
                    a_MD{i} = a_MD{i} + 1/(1+max(D))*(aStore_MD{j} - aStore_MD{i});

                    % Metropolis Hastings
                    F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(FStore_MH{j} - FStore_MH{i});
                    a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(aStore_MH{j} - aStore_MH{i});

                end
            end
        end
    end
    
    % Estimation (after msgs exchange)
    p_hat_MD = zeros(2,n);
    p_hat_MH = zeros(2,n);
    for i=1:n
        p_hat_MD(:,i) = inv(F_MD{i})*a_MD{i};
        p_hat_MH(:,i) = inv(F_MH{i})*a_MH{i};
    end

    % Estimations mean
    p_hat_MD = [mean(p_hat_MD(1,:)); mean(p_hat_MD(2,:))];
    p_hat_MH = [mean(p_hat_MH(1,:)); mean(p_hat_MH(2,:))];

    % Fully connected node: MD == MH!
    pEst_MD = [pEst_MD, p_hat_MD];
    pEst_MH = [pEst_MH, p_hat_MH];
end

if plt == 1
    figure('Name','Obstacle pose estimation');
    hold on;
    for i=1:size(points,1)
        s = sStore{i};
        plot(points(i,1), points(i,2), 'xr', 'LineWidth', 1);
        plot(s(1,:), s(2,:), 'ob', 'LineWidth', 0.1, 'MarkerSize', 2)
        plot(pEst_MD(1,i), pEst_MD(2,i), 'gs', 'LineWidth', 1);
    end
    legend('Real', 'Measured', 'Estimated')
end

% Fill obstacle matrix with obstacle detected
H_detected = zeros(1000, 1000);
if(size(pEst_MD,2)>=1)
    for i=1:size(pEst_MD,2)
        x = round(pEst_MD(1,i));
        y = round(pEst_MD(2,i));
        H_detected(x,y) = 1;
    end
end


end