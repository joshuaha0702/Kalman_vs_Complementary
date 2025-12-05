function [xhat, P, K] = kalman_filter(z, A, H, Q, R, x0, P0)

    % 초기값 차원 설정
    n = size(x0, 1); % 상태 벡터 차원
    m = size(R, 1);  % 측정 벡터 차원
    T = size(z, 2);  % 총 시간 단계
    
    % 결과 행렬 초기화 (3차원 배열 사용)
    xhat = zeros(n, T);  % n x T (각 시점의 상태 벡터를 열 벡터로 저장)
    K_arr = zeros(n, m, T); % n x m x T (칼만 이득 행렬)
    P_arr = zeros(n, n, T); % n x n x T (오차 공분산 행렬)
    
    % 현재 상태 및 공분산 초기화
    x_curr = x0; 
    P_curr = P0;
    
    I = eye(n); % n x n 항등 행렬

    for i = 1:T
        % 1. Predict (예측)
        xhat_pred = A * x_curr;      % n x 1
        P_pred = A * P_curr * A' + Q; % n x n

        % 2. Kalman Gain (칼만 이득)
        % 잔차 공분산 S = H*P_pred*H' + R (m x m)
        S = H * P_pred * H' + R;
        % 칼만 이득 K = P_pred*H' * inv(S) (n x m)
        K_current = P_pred * H' / S; 
        % 3. Update (업데이트)
        % 잔차 (z - H*xhat_pred) : m x 1
        residual = z(:, i) - H * xhat_pred;
        
        % 상태 벡터 업데이트
        x_curr = xhat_pred + K_current * residual; % n x 1
        
        % 오차 공분산 업데이트 (I - K*H)*P_pred
        P_curr = (I - K_current * H) * P_pred; % n x n

        % 결과 저장
        xhat(:, i) = x_curr;
        K_arr(:, :, i) = K_current;
        P_arr(:, :, i) = P_curr;
    end
    
    % 최종 출력 변수
    K = K_arr;
    P = P_arr;
end