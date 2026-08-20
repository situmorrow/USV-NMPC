% ==============================================================================================
%                          NMPC WAYPOINT TRACKING KAPAL AUTONOMOUS
%               MENGGUNAKAN MODEL MATEMATIKA WYNDA (11 PARAMETER BASIS)
%                       SPESIFIKASI SKALA FROUDE (1:100) — ENU
% ===============================================================================================
clc; clear; close all;

%% 1. Parameter Model Kapal WyNDA (11 Parameter Basis)
L = 1.0107;         % Panjang kapal model [meter]
u_0 = 0.6114;       % Kecepatan surge nominal [m/s]

% Vektor Parameter Theta Hasil Identifikasi WyNDA (11 Basis):
% v_dot   = theta_1*v + theta_2*r + theta_3*delta
% r_dot   = theta_4*v + theta_5*r + theta_6*delta
% x_dot   = theta_7*u0*cos(psi) - theta_8*v*sin(psi)
% y_dot   = theta_9*u0*sin(psi) + theta_10*v*cos(psi)
% psi_dot = theta_11*r
theta = [
   -9.2816e-01;  % theta_1  : v pada v_dot
   -2.6644e-01;  % theta_2  : r pada v_dot
    1.2074e-01;  % theta_3  : delta pada v_dot
    2.6348e-03;  % theta_4  : v pada r_dot
   -1.0577e-02;  % theta_5  : r pada r_dot
   -1.3502e-02;  % theta_6  : delta pada r_dot
    5.8118e-02;  % theta_7  : u0*cos(psi)
    1.4903e-03;  % theta_8  : v*sin(psi)
    4.7426e-02;  % theta_9  : u0*sin(psi)
   -4.6814e-03;  % theta_10 : v*cos(psi)
    4.5806e-02   % theta_11 : r pada psi_dot
];

%% 2. Inisialisasi Posisi Awal & Waypoints (Dalam Lat/Lon)
% Opsi 1: Koordinat Kolam Uji Coba USV (Skala Model)
Posisi_Awal_Lat = -7.28715; 
Posisi_Awal_Lon = 112.79600;
Heading_Awal_deg = 90.0;    % Heading awal kapal (derajat)

waypoints_geo = [
    -7.28675, 112.79600;   % WP 1: Titik Utara 1 (Y ~ 44.5 m)
    -7.28630, 112.79600;   % WP 2: Titik Utara 2 (Y ~ 94.5 m)
    -7.28675, 112.79610;   % WP 3: Sisi Timur (X ~ 11.0 m, Y ~ 44.5 m)
    -7.28715, 112.79600    % WP 4: Kembali ke titik awal
];

% Opsi 2: Koordinat Skala Luas (Dapat diaktifkan jika diperlukan)
% Posisi_Awal_Lat = -34.603722; 
% Posisi_Awal_Lon = -58.381592;
% Heading_Awal_deg = -320; 
% waypoints_geo = [
%     -34.605521, -58.388148;   % Waypoint 1
%     -34.596527, -58.390333;   % Waypoint 2
%     -34.600125, -58.375036;   % Waypoint 3
%     -34.603722, -58.381592    % Waypoint 4
% ];

num_waypoints = size(waypoints_geo, 1);

%% 3. Konversi Waypoints Lat/Lon -> ENU Meter
R_earth = 6371000; % Jari-jari bumi (meter)
lat_0 = Posisi_Awal_Lat;
lon_0 = Posisi_Awal_Lon;

waypoints_enu = zeros(num_waypoints, 2);
for i = 1:num_waypoints
    dLat = deg2rad(waypoints_geo(i, 1) - lat_0);
    dLon = deg2rad(waypoints_geo(i, 2) - lon_0);
    
    y_north = R_earth * dLat;
    x_east  = R_earth * cosd(lat_0) * dLon;
    
    waypoints_enu(i, :) = [x_east, y_north];
end

% Seluruh Rute Waypoint ENU (termasuk titik awal [0,0])
route_enu = [0, 0; waypoints_enu];

fprintf('=========================================================================\n');
fprintf('  SIMULASI NMPC WAYPOINT TRACKING USV (MODEL WYNDA 11 PARAMETER)         \n');
fprintf('=========================================================================\n');
fprintf('[INIT] Home Point Locked at Lat: %.6f, Lon: %.6f\n', lat_0, lon_0);
fprintf('[INIT] %d Waypoints dikonversi ke ENU Meter:\n', num_waypoints);
for i = 1:num_waypoints
    fprintf('  -> WP %d: East (X) = %7.2f m, North (Y) = %7.2f m\n', ...
        i, waypoints_enu(i,1), waypoints_enu(i,2));
end

%% 4. Parameter Simulasi & Horizon NMPC
N = 20;             % Horizon prediksi (steps)
T_sim = 0.10;       % Waktu sampling per iterasi (detik, 10 Hz)
T_final = 400;      % Durasi simulasi total (detik)
N_steps = round(T_final / T_sim);

r_tran = 3.0;       % Switching Radius r_tran (meter, skala model)

%% 5. Setup State Awal & Penampung Data Simulasi
Heading_Awal_rad = deg2rad(Heading_Awal_deg); % Heading awal (radian)
current_state_nd = [0; 0; 0; 0; Heading_Awal_rad]; % [v; r; x; y; psi] non-dimensional
u_prev = 0;         % Sudut kemudi awal (radian)

active_wp_idx = 1;  % Waypoint target aktif saat ini (1 s/d num_waypoints)

history_input      = zeros(1, N_steps);
hist_state_nd      = zeros(5, N_steps);
hist_active_wp     = zeros(1, N_steps);
hist_theta_target  = zeros(1, N_steps);
hist_lat_ship      = zeros(1, N_steps);
hist_lon_ship      = zeros(1, N_steps);

%% 6. Loop Utama Simulasi (NMPC Waypoint Tracking)
fprintf('\n[RUN] Memulai loop simulasi kendali NMPC...\n');
timer_val = tic;

for k = 1:N_steps
    % Posisi Kapal Real-Time di ENU Meter
    x_ship = current_state_nd(3) * L;
    y_ship = current_state_nd(4) * L;
    psi_ship = current_state_nd(5);

    % Konversi Posisi Kapal ke Lat/Lon
    hist_lat_ship(k) = lat_0 + (y_ship / R_earth) * (180/pi);
    hist_lon_ship(k) = lon_0 + (x_ship / (R_earth * cosd(lat_0))) * (180/pi);

    % Target Waypoint Aktif saat ini
    wp_target = waypoints_enu(active_wp_idx, :);
    dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);

    % Logika Switching Radius r_tran
    if dist_to_wp <= r_tran && active_wp_idx < num_waypoints
        fprintf('[STEP %4d | t = %5.1f s] Masuk Radius r_tran (%.1f m) dari WP %d -> Ganti Target ke WP %d\n', ...
                k, k*T_sim, dist_to_wp, active_wp_idx, active_wp_idx + 1);
        active_wp_idx = active_wp_idx + 1;
        wp_target = waypoints_enu(active_wp_idx, :);
        dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);
    end

    % Hitung Target Heading Angle (theta_Target)
    theta_target = atan2(wp_target(2) - y_ship, wp_target(1) - x_ship);

    % Bangun Lintasan Referensi Horizon N-Langkah
    x_ref_seq   = zeros(N, 1);
    y_ref_seq   = zeros(N, 1);
    psi_ref_seq = zeros(N, 1);

    for h = 1:N
        dist_step = h * u_0 * T_sim;
        x_ref_dim_h = x_ship + dist_step * cos(theta_target);
        y_ref_dim_h = y_ship + dist_step * sin(theta_target);
        
        x_ref_seq(h)   = x_ref_dim_h / L;
        y_ref_seq(h)   = y_ref_dim_h / L;
        psi_ref_seq(h) = theta_target;
    end

    % Pemanggilan Solver NMPC Waypoint Tracking
    [u_opt, exitflag] = nmpc_kapal_waypoint(current_state_nd, u_prev, u_0, x_ref_seq, y_ref_seq, psi_ref_seq);

    % Simpan Riwayat Simulasi
    hist_state_nd(:, k) = current_state_nd;
    history_input(k)    = u_opt;
    hist_active_wp(k)   = active_wp_idx;
    hist_theta_target(k)= theta_target;

    % Update State Dinamika Plant Kapal Menggunakan Model WyNDA (Euler Step)
    current_state_nd = euler_step_wynda(current_state_nd, u_opt, theta);
    u_prev = u_opt;

    % Cek Terminasi: Jika sudah mencapai Waypoint Terakhir dalam radius r_tran
    if active_wp_idx == num_waypoints && dist_to_wp <= r_tran
        fprintf('\n[GOAL] Kapal telah mencapai Waypoint Terakhir (WP %d, Jarak: %.2f m)!\n', ...
                num_waypoints, dist_to_wp);
        fprintf('[GOAL] Simulasi selesai pada Step %d (t = %.1f s).\n', k, k * T_sim);
        
        % Truncate riwayat data agar sesuai dengan jumlah step aktual
        history_input      = history_input(1:k);
        hist_state_nd      = hist_state_nd(:, 1:k);
        hist_active_wp     = hist_active_wp(1:k);
        hist_theta_target  = hist_theta_target(1:k);
        hist_lat_ship      = hist_lat_ship(1:k);
        hist_lon_ship      = hist_lon_ship(1:k);
        N_steps            = k;
        break;
    end
end

waktu = toc(timer_val);
fprintf('\n=========================================================================\n');
fprintf('SIMULASI NMPC WAYPOINT TRACKING SELESAI\n');
fprintf('Total Waktu Komputasi : %.4f detik (Rata-rata: %.2f ms/step)\n', ...
    waktu, (waktu / N_steps) * 1000);
fprintf('=========================================================================\n\n');

%% 7. Dimensionalisasi Data Output
hist_dim = [
    hist_state_nd(3, :) * L;          % X (m)
    hist_state_nd(4, :) * L;          % Y (m)
    hist_state_nd(5, :);              % Psi (rad)
    hist_state_nd(1, :) * u_0;        % v (m/s)
    hist_state_nd(2, :) * (u_0 / L)   % r (rad/s)
];

time_vector = (0:N_steps-1) * T_sim;

%% 8. Visualisasi Hasil Simulasi (Docked Tab Interface)
set(0, 'DefaultFigureWindowStyle', 'docked');

% Figure 1: Trajektori Kapal & Waypoint di Koordinat ENU Meter
figure('Name', '1. Trajektori Waypoint Tracking (ENU Meter)', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(route_enu(:,1), route_enu(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Sekuens Waypoint'); hold on;
plot(hist_dim(1,:), hist_dim(2,:), 'b-', 'LineWidth', 2.0, 'DisplayName', 'Lintasan NMPC Kapal');
plot(hist_dim(1,1), hist_dim(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

% Gambar Lingkaran Switching Radius r_tran di setiap Waypoint
th_circle = linspace(0, 2*pi, 50);
for i = 1:num_waypoints
    xc = waypoints_enu(i,1) + r_tran * cos(th_circle);
    yc = waypoints_enu(i,2) + r_tran * sin(th_circle);
    plot(xc, yc, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(waypoints_enu(i,1)+1, waypoints_enu(i,2)+1, sprintf('WP %d', i), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
end

grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontWeight', 'bold');
title('Simulasi NMPC Waypoint Tracking (Koordinat ENU Meter)', 'FontWeight', 'bold');
legend('Location', 'best');

% Figure 2: Trajektori Kapal di Koordinat Geografis (Lat/Lon)
figure('Name', '2. Trajektori Geografis (Lat/Lon)', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(waypoints_geo(:,2), waypoints_geo(:,1), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Waypoints (Lat/Lon)'); hold on;
plot(Posisi_Awal_Lon, Posisi_Awal_Lat, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
     'DisplayName', 'Start Point');
plot(hist_lon_ship, hist_lat_ship, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Jejak Kapal (Lat/Lon)');
grid on;
xlabel('Longitude [\circ]', 'FontWeight', 'bold');
ylabel('Latitude [\circ]', 'FontWeight', 'bold');
title('Trajektori Kapal dalam Koordinat Geografis (Lat/Lon)', 'FontWeight', 'bold');
legend('Location', 'best');

% Figure 3: Dynamic States (v, r, Heading vs Target Heading)
figure('Name', '3. Dynamic States Kapal', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(3,1,1);
plot(time_vector, hist_dim(4,:), 'b', 'LineWidth', 1.5); grid on;
ylabel('v [m/s]', 'FontWeight', 'bold'); title('Kecepatan Sway (v)');

subplot(3,1,2);
plot(time_vector, rad2deg(hist_dim(5,:)), 'b', 'LineWidth', 1.5); grid on;
ylabel('r [^\circ/s]', 'FontWeight', 'bold'); title('Kecepatan Yaw Rate (r)');

subplot(3,1,3);
plot(time_vector, rad2deg(hist_dim(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, rad2deg(hist_theta_target), 'r--', 'LineWidth', 1.5); grid on;
ylabel('\psi [^\circ]', 'FontWeight', 'bold'); xlabel('Waktu [detik]', 'FontWeight', 'bold');
title('Sudut Heading (\psi) vs Target Heading (\theta_{Target})');
legend('Heading Kapal (\psi)', 'Target Heading (\theta_{Target})', 'Location', 'best');

% Figure 4: Sinyal Kontrol Rudder
figure('Name', '4. Sinyal Kontrol Kemudi Rudder', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time_vector, rad2deg(history_input), 'r-', 'LineWidth', 1.5); hold on;
yline(45, 'k--', '+45^\circ (Upper Limit)');
yline(-45, 'k--', '-45^\circ (Lower Limit)');
grid on;
xlabel('Waktu [detik]', 'FontWeight', 'bold');
ylabel('Sudut Rudder \delta [^\circ]', 'FontWeight', 'bold');
title('Sinyal Kontrol Kemudi Rudder NMPC', 'FontWeight', 'bold');
legend('Input Kemudi (\delta)', 'Batas Kemudi (\pm 45^\circ)', 'Location', 'best');

%% =========================================================================
% FUNGSI-FUNGSI HELPER MODEL WYNDA UNTUK SIMULASI PLANT
% =========================================================================
function s_next = euler_step_wynda(s, u, theta)
    Phi = compute_basis_phi_11(s, u);
    s_next = s + Phi * theta;
end

function Phi = compute_basis_phi_11(s, delta)
    v   = s(1);
    r   = s(2);
    psi = s(5);
    
    Phi = [
        v,  r,  delta, zeros(1,8);
        zeros(1,3), v,  r,  delta, zeros(1,5);
        zeros(1,6), cos(psi), -v*sin(psi), zeros(1,3);
        zeros(1,8), sin(psi),  v*cos(psi), zeros(1,1);
        zeros(1,10), r
    ];
end