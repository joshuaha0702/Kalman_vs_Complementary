clear;

load('SensData.mat');		% t, ax, ay, az, gx, gy, gz
% Acceleration Sensor Sensitivity: max 2G
% Gyro Sensor Sensitivity: max 1000 degree per sec


N = size(t, 1);
dt = t(2) - t(1);


figure(1);
subplot(311);
plot(t, ax);
grid on;
axis([0 60 -40000 40000]);
ylabel('AX');
title('Acceleration');

subplot(312);
plot(t, ay);
grid on;
axis([0 60 -40000 40000]);
ylabel('AY');

subplot(313);
plot(t, az);
grid on;
axis([0 60 -40000 40000]);
ylabel('AZ');
xlabel('t(sec)');


figure(2);
subplot(311);
plot(t, gx);
grid on;
axis([0 60 -40000 40000]);
ylabel('GX');
title('Gyro');

subplot(312);
plot(t, gy);
grid on;
axis([0 60 -40000 40000]);
ylabel('GY');

subplot(313);
plot(t, gz);
grid on;
axis([0 60 -40000 40000]);
ylabel('GZ');
xlabel('t(sec)');

%% 1 Scaling considering sensor sensitivity and word length(16 bits signed integer)


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

gy_theta = zeros(N, 1); % angle from Gyro y
gy_tmp = 0;
ac_theta = atan2(-ax, az) * (180/pi);
for i = 1:N
    gy_theta(i) = gy_tmp + gy_s(i) * dt; % deg
    gy_tmp = gy_theta(i);
end

% figure(5);
% subplot(211);
% plot(t, ac_theta);
% grid on;
% axis([0 60 -180 180]);
% ylabel('deg');
% title('Angle from Accel y');

% subplot(212);
% plot(t, gy_theta);
% grid on;
% axis([0 60 -180 180]);
% ylabel('deg');
% xlabel('t(sec)');
% title('Angle from gyro y');

%% 3 Sensor Fusion ( Complementary filter )

wc = 2 * 0.3 * pi;
hpf = tf([1 0], [1 wc]);
lpf = tf([0 wc], [1 wc]);
dhpf = c2d(hpf, dt, 'tustin');
dlpf = c2d(lpf, dt, 'tustin');

gyro_y = directFormI(hpf, gy_theta, dt, -50);
ac_y = directFormI(lpf, ac_theta, dt);

y = gyro_y + ac_y;

% figure(8);
% plot(t, y);
% grid on;
% axis([0 60 -180 180]);
% ylabel('deg');
% xlabel('t(sec)');
% title('Complementary Filter Angle');

ts_gyro = timeseries(gy_s, t);
ts_accel = timeseries(ac_theta, t); % for simulink


%% fft
% figure(9);
% Y = fft(ac_theta);
% T = dt;
% Fs = 1/T;
% L = N;
% P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/2))/L;
% semilogx(f, P1);
% xlabel('Frequency (Hz)');
% ylabel('|P1(f)|');
% title('Single-Sided Amplitude Spectrum of ac\_theta(t)');
% grid on;






