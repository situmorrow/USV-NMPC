% ==============================================================================================
%                          VALIDASI DATA YAW & KINEMATIKA KAPAL
% * Validasi dilakukan dengan menggunakan kecepatan (speedMps) dan heading (yaw) dari data eksperimen.
% * Kecepatan dan heading kapal digunakan untuk mengintegrasikan posisi kinematika kapal.
% * Proses validasi dilakukan dengan membandingkan lintasan prediksi vs data posisi GPS (ENU).
% ===============================================================================================
clear; clc; close all;

%% 1. READ & PREPROCESS DATA EKSPERIMEN
dataFile = 'DATA_FIKS.xlsx';
if ~exist(dataFile, 'file')
    if exist('DATA_FIKS.csv', 'file')
        dataFile = 'DATA_FIKS.csv';
    else
        error('File %s tidak ditemukan! Pastikan file berada di direktori yang sama.', dataFile);
    end
end

fprintf('Membaca data eksperimen dari: %s\n', dataFile);
data = readtable(dataFile);
N = height(data);

% Helper function konversi aman ke double array (menghindari error jika kolom dibaca sebagai text/cell)
to_double = @(col) to_num_array(col);

% Ekstraksi data
t_raw   = to_double(data.timestamp);
yaw_deg = to_double(data.yaw);
speed   = to_double(data.speedMps);

% Ekstraksi posisi ENU (prioritas x_enu_ecef / x_enu_flat)
if ismember('x_enu_ecef', data.Properties.VariableNames) && ismember('y_enu_ecef', data.Properties.VariableNames)
    x_gps = to_double(data.x_enu_ecef);
    y_gps = to_double(data.y_enu_ecef);
elseif ismember('x_enu_flat', data.Properties.VariableNames) && ismember('y_enu_flat', data.Properties.VariableNames)
    x_gps = to_double(data.x_enu_flat);
    y_gps = to_double(data.y_enu_flat);
else
    error('Kolom x_enu_ecef atau x_enu_flat tidak ditemukan pada file data!');
end

t = t_raw - t_raw(1);

fprintf('Total sampel data : %d\n', N);
fprintf('Durasi data       : %.2f detik\n', t(end));
fprintf('Kecepatan rata-rata: %.4f m/s\n\n', mean(speed(speed > 0 & ~isnan(speed))));

%% 2. Integrasi Kinematika Posisi (Speed + Yaw Heading)
x_cek = zeros(N, 1); 
y_cek = zeros(N, 1);

x_cek(1) = x_gps(1); 
y_cek(1) = y_gps(1);

for k = 1:N-1
    dt = t(k+1) - t(k);
    if dt <= 0 || isnan(dt)
        dt = 0.1; 
    end
    
    % Sudut heading ENU (dengan offset kalibrasi sensor IMU/kompas 180 derajat)
    psi_enu_rad = deg2rad(yaw_deg(k) + 180);
    
    x_cek(k+1) = x_cek(k) + speed(k) * cos(psi_enu_rad) * dt;
    y_cek(k+1) = y_cek(k) + speed(k) * sin(psi_enu_rad) * dt;
end

%% 3. Evaluasi & Perhitungan Galat (RMSE)
err_x = x_gps - x_cek;
err_y = y_gps - y_cek;
err_2d = sqrt(err_x.^2 + err_y.^2);

rmse_x  = sqrt(mean(err_x.^2));
rmse_y  = sqrt(mean(err_y.^2));
rmse_2d = sqrt(mean(err_2d.^2));

fprintf('=======================================================\n');
fprintf('        HASIL VALIDASI DATA YAW & KINEMATIKA           \n');
fprintf('=======================================================\n');
fprintf('RMSE Posisi X (East) : %.4f meter\n', rmse_x);
fprintf('RMSE Posisi Y (North): %.4f meter\n', rmse_y);
fprintf('RMSE Posisi 2D       : %.4f meter\n', rmse_2d);
fprintf('=======================================================\n\n');

%% 4. Plot Visualisasi Validasi
figure('Name', 'Validasi Data Yaw', 'NumberTitle', 'off', 'Color', 'w');
plot(x_gps, y_gps, 'b-', 'LineWidth', 2.0); hold on;
plot(x_cek, y_cek, 'r--', 'LineWidth', 2.0);
plot(x_gps(1), y_gps(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2); % Start
plot(x_gps(end), y_gps(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2); % End

grid on; axis equal;
xlabel('East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Yaw: Data Eksperimen vs Integrasi (Speed + Yaw) [RMSE 2D = %.3f m]', rmse_2d), ...
      'FontSize', 12, 'FontWeight', 'bold');
legend('Data Eksperimen (GPS ENU)', 'Integrasi Kinematika (Speed + Yaw)', 'Titik Awal (Start)', 'Titik Akhir', 'Location', 'best');

%% Helper Function
function out = to_num_array(in)
    if isnumeric(in)
        out = double(in);
    elseif iscell(in) || isstring(in) || ischar(in)
        out = str2double(in);
    else
        out = double(in);
    end
end
