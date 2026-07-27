clear; clc; close all;

data = readtable("LOG.CSV");
t = data.time_ms / 1000;

ax = data.accel_x / 4096;
ay = data.accel_y / 4096;
az = data.accel_z / 4096;

gx = data.gyro_x / 131;
gy = data.gyro_y / 131;
gz = data.gyro_z / 131;

figure;
plot(t, ax);
hold on;
plot(t, ay);
plot(t, az);
xlabel("Time (s)");
ylabel("Acceleration (g)");
legend("X", "Y", "Z");
title("Accelerometer Data Converted to g");
grid on;

figure;
plot(t, gx);
hold on;
plot(t, gy);
plot(t, gz);
xlabel("Time (s)");
ylabel("Angular velocity (deg/s)");
legend("X", "Y", "Z");
title("Gyroscope Data Converted to deg/s");
grid on;