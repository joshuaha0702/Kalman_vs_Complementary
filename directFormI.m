function [y] = directFormI(ctf, x, dt, y_init)
    % 1. 이산 시스템 변환 및 차수 확인
    tfFd = c2d(ctf, dt, 'tustin'); 
    [num, den] = tfdata(tfFd, 'v');
    nb = length(num);
    na = length(den);
    
    % 2. 출력 초기값 설정 (y_init)
    if nargin < 4
        y_init = 0; 
    end
    y_buf = zeros(na-1, 1); 
    y_buf(:) = y_init; % 과거 출력은 지정한 값으로 초기화
    
    % 3. [핵심] 입력 초기값 설정 (x_buf)
    % "과거에도 지금 들어온 첫 값(x(1))과 같았을 것이다"라고 가정
    x_buf = zeros(nb, 1);     
    x_buf(:) = x(1);   % <--- 여기가 0이 아니라 x(1)로 채워짐!
    
    % 4. 메인 루프 (이전과 동일)
    y = zeros(size(x));
    for n = 1:length(x)
        % (1) 입력 버퍼 업데이트
        x_buf = [x(n); x_buf(1:end-1)];
        
        % (2) 차분 방정식 계산
        feed_forward = num * x_buf;          
        feed_back    = den(2:end) * y_buf;   
        
        y(n) = (feed_forward - feed_back) / den(1);
        
        % (3) 출력 버퍼 업데이트
        y_buf = [y(n); y_buf(1:end-1)];
    end
end