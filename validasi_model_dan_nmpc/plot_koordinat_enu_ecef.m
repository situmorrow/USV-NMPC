%% =========================================================================
% SCRIPT 2: PLOTTING HASIL KONVERSI KOORDINAT ENU (FLAT), ENU (ECEF), DAN ECEF
% Percobaan Kapal Autonomous - USV NMPC
% Folder: d:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\validasi_model_dan_nmpc
% =========================================================================

clear; clc; close all;

% 1. Load Data CSV yang Sudah Diperbarui oleh Script 1
csvPath = 'gerak_random.csv';
if ~exist(csvPath, 'file')
    error('File %s tidak ditemukan! Jalankan script 01_konversi_latlon_ecef_enu.m terlebih dahulu.', csvPath);
end

data = readtable(csvPath);

% Pastikan kolom konversi sudah ada di CSV
if ~ismember('x_enu_flat', data.Properties.VariableNames)
    error('Kolom konversi belum ada di CSV. Jalankan 01_konversi_latlon_ecef_enu.m terlebih dahulu!');
end

% Extract Data
x_flat = data.x_enu_flat;
y_flat = data.y_enu_flat;

x_ecef_enu = data.x_enu_ecef;
y_ecef_enu = data.y_enu_ecef;

X_ecef = data.X_ecef;
Y_ecef = data.Y_ecef;
Z_ecef = data.Z_ecef;

timestamp = data.timestamp;

%% =========================================================================
% PLOT 1: PLOT ENU HASIL DARI LATLON -> ENU (FLAT-EARTH)
% =========================================================================
figure('Name', '1. Plot ENU (Direct LatLon -> ENU)', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
plot(x_flat, y_flat, 'b-', 'LineWidth', 1.8); hold on;
plot(x_flat(1), y_flat(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot(x_flat(end), y_flat(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End
grid on; axis equal;
xlabel('East (x) [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('North (y) [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title('Lintasan Kapal ENU (Metode Direct LatLon \rightarrow ENU Flat-Earth)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Lintasan Kapal', 'Start Point (Home)', 'End Point', 'Location', 'best');

%% =========================================================================
% PLOT 2: PLOT ENU HASIL DARI LATLON -> ECEF -> ENU (RIGOROUS WGS84)
% =========================================================================
figure('Name', '2. Plot ENU (LatLon -> ECEF -> ENU)', 'NumberTitle', 'off', 'Position', [150, 150, 800, 600]);
plot(x_ecef_enu, y_ecef_enu, 'r-', 'LineWidth', 1.8); hold on;
plot(x_ecef_enu(1), y_ecef_enu(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot(x_ecef_enu(end), y_ecef_enu(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End
grid on; axis equal;
xlabel('East (x) [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('North (y) [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title('Lintasan Kapal ENU (Metode LatLon \rightarrow ECEF \rightarrow ENU WGS84)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Lintasan Kapal', 'Start Point (Home)', 'End Point', 'Location', 'best');

%% =========================================================================
% PLOT 3: PLOT KOORDINAT 3D ECEF (X, Y, Z)
% =========================================================================
figure('Name', '3. Plot 3D ECEF', 'NumberTitle', 'off', 'Position', [200, 200, 850, 650]);
plot3(X_ecef, Y_ecef, Z_ecef, 'm-', 'LineWidth', 1.8); hold on;
plot3(X_ecef(1), Y_ecef(1), Z_ecef(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(X_ecef(end), Y_ecef(end), Z_ecef(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('ECEF X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('ECEF Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('ECEF Z [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title('Lintasan Kapal dalam Koordinat 3D ECEF (Earth-Centered Earth-Fixed)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Lintasan ECEF', 'Start Point', 'End Point', 'Location', 'best');
view(45, 30);

%% =========================================================================
% PLOT 4 (BONUS): COMPARISON OVERLAY & ERROR DIFFERENCE (MM)
% =========================================================================
figure('Name', '4. Perbandingan ENU Flat vs ECEF->ENU', 'NumberTitle', 'off', 'Position', [250, 250, 950, 700]);

% Subplot A: Overlay Lintasan
subplot(2, 1, 1);
plot(x_flat, y_flat, 'b-', 'LineWidth', 2.5); hold on;
plot(x_ecef_enu, y_ecef_enu, 'r--', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('East (x) [m]'); ylabel('North (y) [m]');
title('Overlay Lintasan ENU: Direct Flat-Earth (Biru) vs ECEF\rightarrowENU (Merah Putus)');
legend('Direct ENU (Flat)', 'LatLon \rightarrow ECEF \rightarrow ENU');

% Subplot B: Selisih Eror Jarak (dalam milimeter)
subplot(2, 1, 2);
err_dist_mm = sqrt((x_flat - x_ecef_enu).^2 + (y_flat - y_ecef_enu).^2) * 1000;
plot(timestamp - timestamp(1), err_dist_mm, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Waktu [detik]'); ylabel('Selisih Eror Jarak [mm]');
title('Selisih Perbedaan Hasil ENU Antara Kedua Metode (Skala Milimeter)');
ylim([0, max(err_dist_mm)*1.2 + 0.1]);

fprintf('Finished plotting successfully!\n');
