% ==============================================================================================
%                          KONVERSI KOORDINAT LAT/LON -> ENU
% * Proses ini mengubah data Latitude dan Longitude menjadi koordinat ENU (East, North, Up)
%   menggunakan dua metode berbeda untuk perbandingan, yaitu:
%   (1) LAT/LON -> ENU (Flat-Earth)
%   (2) LAT/LON -> ECEF -> ENU (WGS84 Rigorous Transformation)
% * Origin/Reference Point (Titik Awal) ditetapkan berdasarkan titik pertama dari data CSV.
% * Hasil konversi dari kedua metode kemudian ditambahkan sebagai kolom baru pada file CSV
%   yang sama untuk kemudahan analisis lebih lanjut.
% ===============================================================================================

clear; clc; close all;

%% READ & PREPROCESS DATA EKSPERIMEN CSV
csvPath = 'ZIGZAG.csv';
if ~exist(csvPath, 'file')
    error('File %s tidak ditemukan! Pastikan script berada di folder yang sama dengan file CSV', csvPath);
end

fprintf('Membaca data eksperimen dari: %s\n', csvPath);
data = readtable(csvPath);

% Deteksi Nama Kolom Latitude & Longitude
varNames = data.Properties.VariableNames;
latIdx = find(strcmpi(varNames, 'latitude') | strcmpi(varNames, 'lat'), 1);
lonIdx = find(strcmpi(varNames, 'longitude') | strcmpi(varNames, 'lon') | strcmpi(varNames, 'long'), 1);

if isempty(latIdx) || isempty(lonIdx)
    error('Kolom latitude / longitude tidak ditemukan di CSV! Kolom yang tersedia: %s', strjoin(varNames, ', '));
end

lat = data{:, latIdx};
lon = data{:, lonIdx};
N_data = height(data);

fprintf('Total data: %d baris\n', N_data);

% Tentukan Origin / Home Point (Diambil dari Baris Pertama Data)
lat0 = lat(1);
lon0 = lon(1);
h0 = 0.0; % Altitude diasumsikan 0 m (permukaan air)

fprintf('Home Point (Origin Locked) pada Lat: %.6f, Lon: %.6f\n', lat0, lon0);

%% METODE 1: Direct Lat/Lon -> ENU (Flat-Earth / Local Equirectangular)
R_earth = 6371000.0; % Jari-jari bumi rata-rata (meter)

lat0_rad = lat0 * (pi / 180.0);
lon0_rad = lon0 * (pi / 180.0);

dlat_rad = (lat - lat0) * (pi / 180.0);
dlon_rad = (lon - lon0) * (pi / 180.0);

x_enu_flat = R_earth * cos(lat0_rad) .* dlon_rad; % East (meter)
y_enu_flat = R_earth .* dlat_rad;                 % North (meter)
z_enu_flat = zeros(N_data, 1);                    % Up (meter)

%% METODE 2: Lat/Lon -> ECEF -> ENU (WGS84 Rigorous Transformation)

% Parameter Ellipsoid WGS84
a = 6378137.0;               % Semi-major axis (meter)
f = 1 / 298.257223563;       % Flattening
e2 = 2*f - f^2;              % Eksentrisitas pertama kuadrat

lat_rad = lat * (pi / 180.0);
lon_rad = lon * (pi / 180.0);
h = zeros(N_data, 1);        % Altitude diasumsikan 0 m

% Konversi Lat/Lon/Alt -> ECEF (X, Y, Z)
N_phi = a ./ sqrt(1.0 - e2 * (sin(lat_rad)).^2);
X_ecef = (N_phi + h) .* cos(lat_rad) .* cos(lon_rad);
Y_ecef = (N_phi + h) .* cos(lat_rad) .* sin(lon_rad);
Z_ecef = (N_phi * (1.0 - e2) + h) .* sin(lat_rad);

% ECEF Origin (Home Point)
N_phi0 = a / sqrt(1.0 - e2 * (sin(lat0_rad))^2);
X0 = (N_phi0 + h0) * cos(lat0_rad) * cos(lon0_rad);
Y0 = (N_phi0 + h0) * cos(lat0_rad) * sin(lon0_rad);
Z0 = (N_phi0 * (1.0 - e2) + h0) * sin(lat0_rad);

% Selisih ECEF (Delta ECEF)
dX = X_ecef - X0;
dY = Y_ecef - Y0;
dZ = Z_ecef - Z0;

% Matriks Rotasi ECEF -> ENU pada (lat0, lon0)
slat0 = sin(lat0_rad); clat0 = cos(lat0_rad);
slon0 = sin(lon0_rad); clon0 = cos(lon0_rad);

x_enu_ecef = -slon0 * dX + clon0 * dY;
y_enu_ecef = -slat0 * clon0 * dX - slat0 * slon0 * dY + clat0 * dZ;
z_enu_ecef =  clat0 * clon0 * dX + clat0 * slon0 * dY + slat0 * dZ;

%% MEMASUKKAN KEMBALI HASIL KONVERSI KE SHEET / TABEL CSV
data.x_enu_flat = x_enu_flat;
data.y_enu_flat = y_enu_flat;
data.z_enu_flat = z_enu_flat;

data.X_ecef = X_ecef;
data.Y_ecef = Y_ecef;
data.Z_ecef = Z_ecef;

data.x_enu_ecef = x_enu_ecef;
data.y_enu_ecef = y_enu_ecef;
data.z_enu_ecef = z_enu_ecef;

% Simpan kembali ke file CSV
writetable(data, csvPath);
fprintf('SUCCESS! Data hasil konversi berhasil dimasukkan dan disimpan ke %s\n', csvPath);

% Evaluasi Selisih Antara Kedua Metode seperti pad
diff_x = abs(x_enu_flat - x_enu_ecef);
diff_y = abs(y_enu_flat - y_enu_ecef);
max_diff_m = max(sqrt(diff_x.^2 + diff_y.^2));

fprintf('\n=== EVALUASI PERBEDAAAN METODE ===\n');
fprintf('Max Selisih Position ENU (Flat vs ECEF->ENU): %.6f meter (%.2f mm)\n', max_diff_m, max_diff_m * 1000);
