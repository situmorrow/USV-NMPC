%% =========================================================================
%  PREPROCESSING & KONVERSI KOORDINAT DATA KAPAL (WYNDA MODEL PREPARATION)
%  GPS (LLA) -> ECEF -> ENU -> Body-Fixed States (v, r, x_ENU, y_ENU, psi, delta, rpm)
% =========================================================================
%  Deskripsi:
%  Script ini mengonversi data eksperimen maritim mentah (format CSV) menjadi
%  state kapal dalam koordinat ENU dan frame body-fixed, kemudian menyimpannya
%  ke format Excel (.xlsx) dengan struktur sheet 'Lembar2' untuk model WyNDA.
%
%  Input:
%  - DATA_FIKS.csv (Data log eksperimen GPS, IMU, Servo, dan Propeller)
%
%  Output:
%  - DATA_UNTUK_WYNDA.xlsx (Sheet 'Lembar2' siap pakai untuk turning_otomatis_kode_ENU.m)
%    Kolom: [v, r, x_ENU, y_ENU, psi, delta, rpm]
%
%  Definisi Variabel Output:
%  - v     : Sway velocity kapal pada body-fixed frame [m/s]
%  - r     : Yaw rate (kecepatan sudut putar) [rad/s]
%  - x_ENU : Posisi East relatif terhadap titik awal / Home [m]
%  - y_ENU : Posisi North relatif terhadap titik awal / Home [m]
%  - psi   : Sudut heading/yaw kapal [rad] (unwrapped)
%  - delta : Sudut rudder rata-rata [rad]
%  - rpm   : Putaran propeller rata-rata [rpm]
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('   PREPROCESSING & KONVERSI KOORDINAT DATA KAPAL (WYNDA FORMAT)          \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%  1. KONFIGURASI FILE INPUT & OUTPUT
% =========================================================================

input_csv  = 'DATA_FIKS.csv';
output_xlsx = 'DATA_UNTUK_WYNDA.xlsx';
target_sheet = 'Lembar2';

% Cek keberadaan file input CSV
if ~exist(input_csv, 'file')
    error('File input "%s" tidak ditemukan! Pastikan file berada di direktori yang sama.', input_csv);
end

fprintf('1. Membaca data eksperimen dari CSV: %s\n', input_csv);
T_raw = readtable(input_csv);
N_total = height(T_raw);
fprintf('   Total data terbaca: %d baris\n\n', N_total);

%% =========================================================================
%  2. SPESIFIKASI KAPAL
% =========================================================================

L = 1.0107;          % Panjang kapal referensi [meter]
fprintf('2. Parameter Kapal:\n');
fprintf('   Panjang Kapal L = %.2f m\n\n', L);

%% =========================================================================
%  3. EKSTRAKSI VARIABEL DARI TABEL DATA CSV
% =========================================================================

% Ekstraksi Timestamp & Time Step
if ismember('timestamp', T_raw.Properties.VariableNames)
    t_raw = T_raw.timestamp;
else
    t_raw = (0:N_total-1)' * 0.1;
end

% Ekstraksi Koordinat Geodetik (Latitude & Longitude)
if ismember('latitude', T_raw.Properties.VariableNames)
    lat_deg = T_raw.latitude;
elseif ismember('lat', T_raw.Properties.VariableNames)
    lat_deg = T_raw.lat;
else
    error('Kolom latitude tidak ditemukan pada data CSV!');
end

if ismember('longitude', T_raw.Properties.VariableNames)
    lon_deg = T_raw.longitude;
elseif ismember('lon', T_raw.Properties.VariableNames)
    lon_deg = T_raw.lon;
else
    error('Kolom longitude tidak ditemukan pada data CSV!');
end

% Ekstraksi Heading / Yaw
if ismember('yaw', T_raw.Properties.VariableNames)
    yaw_deg = T_raw.yaw;
else
    error('Kolom yaw tidak ditemukan pada data CSV!');
end

% Ekstraksi Rudder (Servo 1 & Servo 2)
if ismember('Calc_deg_servo_1', T_raw.Properties.VariableNames) && ismember('Calc_deg_servo_2', T_raw.Properties.VariableNames)
    delta1_deg = T_raw.Calc_deg_servo_1;
    delta2_deg = T_raw.Calc_deg_servo_2;
elseif ismember('Calc_deg_servo_1', T_raw.Properties.VariableNames)
    delta1_deg = T_raw.Calc_deg_servo_1;
    delta2_deg = delta1_deg;
else
    delta1_deg = zeros(N_total, 1);
    delta2_deg = zeros(N_total, 1);
end

% Ekstraksi Propeller RPM
if ismember('rpm_prop_1', T_raw.Properties.VariableNames) && ismember('rpm_prop_2', T_raw.Properties.VariableNames)
    rpm_prop_1 = T_raw.rpm_prop_1;
    rpm_prop_2 = T_raw.rpm_prop_2;
elseif ismember('rpm_prop_1', T_raw.Properties.VariableNames)
    rpm_prop_1 = T_raw.rpm_prop_1;
    rpm_prop_2 = rpm_prop_1;
else
    rpm_prop_1 = zeros(N_total, 1);
    rpm_prop_2 = zeros(N_total, 1);
end

%% =========================================================================
%  4. PENGOLAHAN TIME STEP (dt), HEADING (psi), RUDDER (delta), & RPM
% =========================================================================

% Time step aktual
dt = diff(t_raw);
mean_dt = mean(dt(dt > 0 & ~isnan(dt)));
if isempty(mean_dt) || isnan(mean_dt) || mean_dt <= 0
    mean_dt = 0.1;
end
dt(dt <= 0 | isnan(dt)) = mean_dt;

% Heading (yaw) dalam radian dan unwrapped
psi = deg2rad(yaw_deg);
psi = unwrap(psi);

% Sudut Rudder Rata-rata (delta) dalam radian
delta_deg = (delta1_deg + delta2_deg) / 2;
delta = deg2rad(delta_deg);

% Propeller RPM (rata-rata dan RPS per sisi)
rpm_mean = (rpm_prop_1 + rpm_prop_2) / 2;
n_P = rpm_prop_1 / 60;   % rps port
n_S = rpm_prop_2 / 60;   % rps starboard

%% =========================================================================
%  5. TRANSFORMASI KOORDINAT GEODETIK (LLA) -> ECEF (WGS84)
% =========================================================================

% Parameter Ellipsoid WGS84
a  = 6378137.0;                 % Semi-major axis [m]
f  = 1 / 298.257223563;         % Flattening
e2 = f * (2 - f);               % First eccentricity squared

lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

% Jari-jari kurvatur bidang vertikal utama
Ngeo = a ./ sqrt(1 - e2 * (sin(lat).^2));

% Koordinat ECEF (X, Y, Z)
X_ecef = Ngeo .* cos(lat) .* cos(lon);
Y_ecef = Ngeo .* cos(lat) .* sin(lon);
Z_ecef = (Ngeo * (1 - e2)) .* sin(lat);

%% =========================================================================
%  6. TRANSFORMASI KOORDINAT ECEF -> ENU (LOCAL TANGENT PLANE)
% =========================================================================

% Titik Referensi Origin (Titik Awal Pengukuran)
lat0 = lat(1);
lon0 = lon(1);

X0 = X_ecef(1);
Y0 = Y_ecef(1);
Z0 = Z_ecef(1);

% Selisih relatif terhadap Origin
dX = X_ecef - X0;
dY = Y_ecef - Y0;
dZ = Z_ecef - Z0;

% Matriks Rotasi ECEF -> ENU pada (lat0, lon0)
R = [ -sin(lon0)            cos(lon0)           0
      -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0) cos(lat0)
       cos(lat0)*cos(lon0)  cos(lat0)*sin(lon0) sin(lat0)];

enu = R * [dX'; dY'; dZ'];

E = enu(1,:)';   % East (sumbu X ENU) [m]
N = enu(2,:)';   % North (sumbu Y ENU) [m]
U = enu(3,:)';   % Up (sumbu Z ENU) [m]

%% =========================================================================
%  7. SMOOTHING POSISI & ESTIMASI KECEPATAN (BODY-FIXED & EARTH-FIXED)
% =========================================================================

% Smoothing lintasan posisi menggunakan moving average (window = 5)
E_smooth = movmean(E, 5);
N_smooth = movmean(N, 5);

% Kecepatan Earth-Fixed (Edot, Ndot) [m/s]
Edot = [0; diff(E_smooth) ./ dt];
Ndot = [0; diff(N_smooth) ./ dt];

% Transformasi Kinematika ke Body-Fixed Frame:
% u = Surge velocity (kecepatan maju kapal) [m/s]
% v = Sway velocity (kecepatan samping kapal) [m/s]
u =  cos(psi) .* Edot + sin(psi) .* Ndot;
v = -sin(psi) .* Edot + cos(psi) .* Ndot;

% Yaw Rate (r) [rad/s]
r = [0; diff(psi) ./ dt];

%% =========================================================================
%  8. PEMBENTUKAN TABEL DATA WYNDA (LEMBAR2) & FULL STATE
% =========================================================================

% Format Kolom Lembar2 WyNDA:
% 1 = v      : sway velocity [m/s]
% 2 = r      : yaw rate [rad/s]
% 3 = x_ENU  : posisi East [m]
% 4 = y_ENU  : posisi North [m]
% 5 = psi    : heading [rad]
% 6 = delta  : rudder angle [rad]
% 7 = rpm    : propeller speed [rpm]

x_ENU = E;
y_ENU = N;

T_wynda = table(v, r, x_ENU, y_ENU, psi, delta, rpm_mean, ...
    'VariableNames', {'v', 'r', 'x_ENU', 'y_ENU', 'psi', 'delta', 'rpm'});

% Full state table untuk kebutuhan inspeksi lengkap
T_full = table(u, v, r, x_ENU, y_ENU, psi, delta, n_P, n_S, rpm_mean, ...
    'VariableNames', {'u', 'v', 'r', 'x_ENU', 'y_ENU', 'psi', 'delta', 'n_P', 'n_S', 'rpm'});

%% =========================================================================
%  9. MENYIMPAN HASIL KE FILE EXCEL (DATA_UNTUK_WYNDA.xlsx)
% =========================================================================

fprintf('3. Menyimpan hasil konversi ke file Excel:\n');
fprintf('   Target File : %s\n', output_xlsx);
fprintf('   Target Sheet: %s\n', target_sheet);

try
    if exist(output_xlsx, 'file')
        writetable(T_wynda, output_xlsx, 'Sheet', target_sheet, 'WriteMode', 'overwritesheet');
    else
        writetable(T_wynda, output_xlsx, 'Sheet', target_sheet);
    end
    fprintf('   ✔ Sukses menulis sheet "%s" ke %s (Total: %d baris data)\n\n', target_sheet, output_xlsx, height(T_wynda));
catch ME
    warning('Gagal menulis dengan format standar: %s. Mencoba fallback...', ME.message);
    writetable(T_wynda, output_xlsx);
    fprintf('   ✔ Sukses menulis tabel ke %s\n\n', output_xlsx);
end

% Menyimpan salinan CSV processed untuk opsi komputasi langsung
writetable(T_wynda, 'data_wynda_processed_ENU.csv');
writetable(T_full, 'data_turning_fullstate_ENU.csv');
fprintf('   ✔ Menyimpan backup CSV: data_wynda_processed_ENU.csv\n\n');

%% =========================================================================
%  10. INFORMASI STATISTIK HASIL KONVERSI
% =========================================================================

fprintf('=========================================================================\n');
fprintf('                         STATISTIK DATA & STATE                          \n');
fprintf('=========================================================================\n');
fprintf('Origin Geodetik (Titik Awal) :\n');
fprintf('  Lat0 = %.8f deg | Lon0 = %.8f deg\n', rad2deg(lat0), rad2deg(lon0));
fprintf('  X0   = %.3f m   | Y0   = %.3f m   | Z0   = %.3f m\n\n', X0, Y0, Z0);

fprintf('Rentang Posisi ENU :\n');
fprintf('  East  (x_ENU) : min = %8.3f m | max = %8.3f m\n', min(x_ENU), max(x_ENU));
fprintf('  North (y_ENU) : min = %8.3f m | max = %8.3f m\n\n', min(y_ENU), max(y_ENU));

fprintf('Statistik Kecepatan & Yaw Rate :\n');
fprintf('  Surge (u)     : min = %8.4f m/s   | max = %8.4f m/s   | mean = %8.4f m/s\n', min(u), max(u), mean(u));
fprintf('  Sway  (v)     : min = %8.4f m/s   | max = %8.4f m/s   | mean = %8.4f m/s\n', min(v), max(v), mean(v));
fprintf('  Yaw Rate (r)  : min = %8.6f rad/s | max = %8.6f rad/s | mean = %8.6f rad/s\n\n', min(r), max(r), mean(r));

fprintf('Statistik Input Kemudi & Propeller :\n');
fprintf('  Rudder (delta): min = %8.2f deg   | max = %8.2f deg\n', rad2deg(min(delta)), rad2deg(max(delta)));
fprintf('  Propeller RPM : min = %8.1f rpm   | max = %8.1f rpm\n', min(rpm_mean), max(rpm_mean));
fprintf('  Mean dt       : %8.4f detik\n', mean_dt);
fprintf('=========================================================================\n\n');

%% =========================================================================
%  11. VISUALISASI GRAFIK
% =========================================================================

% Figure 1: Lintasan Kapal 2D ENU (East vs North)
figure('Name', '1. Lintasan Turning Test ENU (meter)', 'NumberTitle', 'off', 'Color', 'w');
plot(x_ENU, y_ENU, 'b-', 'LineWidth', 2);
hold on;
plot(x_ENU(1), y_ENU(1), 'go', 'MarkerSize', 10, 'LineWidth', 2.5, 'DisplayName', 'Titik Awal (Start)');
plot(x_ENU(end), y_ENU(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2.5, 'DisplayName', 'Titik Akhir (End)');
grid on; axis equal;
xlabel('East / x_{ENU} [meter]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('North / y_{ENU} [meter]', 'FontSize', 12, 'FontWeight', 'bold');
title('Lintasan Hasil Konversi Koordinat ENU', 'FontSize', 13, 'FontWeight', 'bold');
legend('Lintasan Kapal', 'Start', 'End', 'Location', 'best');

% Figure 2: Profil State Kapal (v, r, psi, delta, rpm)
time_vec = t_raw - t_raw(1);

figure('Name', '2. Profil State Kapal untuk WyNDA', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,2,1);
plot(time_vec, u, 'b-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('u [m/s]');
title('Surge Velocity (u)');

subplot(3,2,2);
plot(time_vec, v, 'r-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('v [m/s]');
title('Sway Velocity (v)');

subplot(3,2,3);
plot(time_vec, r, 'm-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('r [rad/s]');
title('Yaw Rate (r)');

subplot(3,2,4);
plot(time_vec, rad2deg(psi), 'k-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('\psi [deg]');
title('Heading / Yaw (\psi)');

subplot(3,2,5);
plot(time_vec, rad2deg(delta), 'g-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('\delta [deg]');
title('Rudder Angle (\delta)');

subplot(3,2,6);
plot(time_vec, rpm_mean, 'c-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [s]'); ylabel('RPM');
title('Propeller Speed (RPM)');

fprintf('✔ PREPROCESSING & KONVERSI SELESAI!\n');
