function [xhat, P, K] = kalman_filter_input(z, u, A, B, H, Q, R, x0, P0)
% 입력 변수에 u(제어입력)와 B(입력행렬)가 추가되었습니다.

    % 시간 길이 계산 (데이터 개수 기준)
    T = size(z, 2); 
    n = size(x0, 1); % 상태 차원
    m = size(R, 1);  % 측정 벡터 차원    

    % 결과 저장용 배열
    xhat = zeros(n, T);
    K_arr = zeros(n, m, T); % n x m x T (칼만 이득 행렬)
    P_arr = zeros(n, n, T);
    
    x_curr = x0;
    P_curr = P0;
    I = eye(n);

    for i = 1:T
        % --- 1. Predict (예측 단계 수정됨) ---
        % 기존: A * x_curr
        % 수정: A * x_curr + B * u(:, i) (각속도 입력을 더함)
        xhat_pred = A * x_curr + B * u(:, i);
        
        P_pred = A * P_curr * A' + Q;

        % --- 2. Kalman Gain ---
        S = H * P_pred * H' + R;
        K = P_pred * H' / S;

        % --- 3. Update (보정 단계) ---
        residual = z(:, i) - H * xhat_pred;
        x_curr = xhat_pred + K * residual;
        P_curr = (I - K * H) * P_pred;

        % 저장
        xhat(:, i) = x_curr;
        K_arr(:, :, i) = K;
        P_arr(:, :, i) = P_curr;
    end
    
    K = K_arr;
    P = P_arr;
end