% ==============================================================================================
%                          VALIDASI DATA YAW
% * Validasi dilakukan dengan menggunakan kecepatan dan heading dari data eksperimen kapal.
% * Kecepatan dan heading kapal digunakan untuk memprediksi posisi kapal (kinematika kapal).
% * Proses validasi dilakukan dengan membandingkan posisi antara prediksi dengan data eksperimen.
% ===============================================================================================
clear; clc; close all;

%% READ & PREPROCESS DATA EKSPERIMEN CSV
csvPath = 'gerak_random.csv';
if ~exist(csvPath, 'file')
    error('File %s tidak ditemukan! Pastikan script berada di folder yang sama dengan file CSV', csvPath);
end

fprintf('Membaca data eksperimen dari: %s\n', csvPath);
data = readtable(csvPath);
t = data.timestamp - data.timestamp(1);
yaw_deg = data.yaw;
speed = data.speedMps;
x_gps = data.x_enu_flat;
y_gps = data.y_enu_flat;
N = height(data);

%% Pengecekan Yaw (Speed + Heading)
x_cek = zeros(N,1); 
y_cek = zeros(N,1);

x_cek(1) = x_gps(1); 
y_cek(1) = y_gps(1);

for k = 1:N-1
    dt = t(k+1) - t(k);
    if dt <= 0, dt = 0.1; end
    psi_enu_rad = deg2rad(yaw_deg(k)+180);
    x_cek(k+1) = x_cek(k) + speed(k) * cos(psi_enu_rad) * dt;
    y_cek(k+1) = y_cek(k) + speed(k) * sin(psi_enu_rad) * dt;
end

%% Plot Visualisasi Validasi
figure('Name', 'Validasi Data Yaw');
plot(x_gps, y_gps, 'b-', 'LineWidth', 2); hold on;
plot(x_cek, y_cek, 'r--', 'LineWidth', 2);
grid on; axis equal;
xlabel('East X [m]'); ylabel('North Y [m]');
title('Validasi Yaw: Data Eksperimen vs Integrasi Speed + Yaw');
legend('Data Eksperimen', 'Integrasi Speed + Yaw');
