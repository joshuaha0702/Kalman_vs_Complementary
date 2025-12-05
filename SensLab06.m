clear;
complementary; % call complementary.m to get ac_theta, y, dt, N
load('SensData.mat');		% t, ax, ay, az, gx, gy, gz

%% this is same as complementary.m
% Acceleration Sensor Sensitivity: max 2G
% Gyro Sensor Sensitivity: max 1000 degree per sec

% N = size(t, 1);
% dt = t(2) - t(1);

% figure(1);
% subplot(311);
% plot(t, ax);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('AX');
% title('Acceleration');

% subplot(312);
% plot(t, ay);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('AY');

% subplot(313);
% plot(t, az);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('AZ');
% xlabel('t(sec)');

% figure(2);
% subplot(311);
% plot(t, gx);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('GX');
% title('Gyro');

% subplot(312);
% plot(t, gy);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('GY');

% subplot(313);
% plot(t, gz);
% grid on;
% axis([0 60 -40000 40000]);
% ylabel('GZ');
% xlabel('t(sec)');

% 1. Scaling considering sensor sensitivity and word length(16 bits signed integer)

accel = 0.00059814453125;  % 2G / 2^15 
gyro = 0.030517578125;    % 1000 / 2^15 deg/s 기준

ax_s = zeros(N, 1);
ay_s = zeros(N, 1);
az_s = zeros(N, 1);
gx_s = zeros(N, 1);
gy_s = zeros(N, 1);
gz_s = zeros(N, 1);

for i = 1:N % loop for scaling
    ax_s(i) = ax(i) * accel;
    ay_s(i) = ay(i) * accel;
    az_s(i) = az(i) * accel;
    
    gx_s(i) = gx(i) * gyro;
    gy_s(i) = gy(i) * gyro;
    gz_s(i) = gz(i) * gyro;
end

figure(3);
subplot(311);
plot(t, ax_s);
grid on;
axis([0 60 -20 20]);
ylabel('m/s^2');
title('Acceleration');

subplot(312);
plot(t, ay_s);
grid on;
axis([0 60 -20 20]);
ylabel('m/s^2');

subplot(313);
plot(t, az_s);
grid on;
axis([0 60 -20 20]);
ylabel('m/s^2');
xlabel('t(sec)');

figure(4);
subplot(311);
plot(t, gx_s);
grid on;
axis([0 60 -1000 1000]);
ylabel('deg/s');
title('Gyro');

subplot(312);
plot(t, gy_s);
grid on;
axis([0 60 -1000 1000]);
ylabel('deg/s');

subplot(313);
plot(t, gz_s);
grid on;
axis([0 60 -1000 1000]);
ylabel('deg/s');
xlabel('t(sec)');

%% 2. Get angles from Acceleration Sensor and Gyro Sensor, respectively.

z = [ac_theta'; gy_s'];
Q = 0.001*eye(2);
R = [1 0; 0 0.001];
x0 = [0; 0];          % 초기 상태 [각도; 각속도]
P0 = eye(2) * 1;      % 초기 오차 공분산

A = [1 dt; 0 1]; % 상태 전이 행렬
H = [1 0; 0 1]; % 관측 행렬
[xhat] = kalman_filter(z, A, H, Q, R, x0, P0);

figure(5);
plot(t, xhat(1, :), 'r');
hold on;
plot(t, y, 'b');
hold off;
grid on;
xlabel('time(s)');
ylabel('angle(deg)');
title('Angle from Acceleration Sensor vs Kalman Filtered Angle');
legend('Kalman Filter', 'Angle from Accel Sensor');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Sensor Fusion ( Complementary filter )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B = [dt; 0];
H = [1 0];
Q = [0.001, 0; 
     0, 0.001]; 
R = 1;
[xhat_cf] = kalman_filter_input(ac_theta', gy_s', A, B, H, Q, R, x0, P0);
figure(6);
plot(t, xhat_cf(1, :), 'r');
hold on;
plot(t, y, 'b');
hold off;
grid on;
xlabel('time(s)');
ylabel('angle(deg)');
title('Complementary vs Kalman Filtered');
legend('Kalman Filter with Input', 'Angle from Accel Sensor');



