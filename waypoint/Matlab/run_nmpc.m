clc; clear; close all;
% =========================================================================
% SIMULASI NMPC WAYPOINT TRACKING KAPAL AUTONOMOUS (PAPER POTOČNIK 2025 ADAPTATION)
% =========================================================================
% File: run_nmpc.m
% Adaptasi: d:\2026\Percobaan_Kapal_Autonomous\diskusi\khusus\09_adaptasi_metode_waypoint_dan_mpc_paper_potocnik.md
% Arsitektur: Konversi Lat/Lon -> ENU Meter, Dynamic Horizon Generator, Switching Radius r_tran

fprintf('Memulai Simulasi NMPC Waypoint Tracking...\n');
timer_val = tic;

%% 1. Inisialisasi Posisi Awal & Waypoints (Geografis Lat/Lon)
Posisi_Awal_Lat = -34.603722; 
Posisi_Awal_Lon = -58.381592;
Heading_Awal_deg = -140; % Heading awal (derajat), mengarah ke arah Waypoint #1

% Daftar Waypoint Misi (Latitude, Longitude)
waypoints_geo = [
    -34.605347, -58.384339;   % Waypoint 1
    -34.608054, -58.389990;   % Waypoint 2
    -34.610763, -58.395625;   % Waypoint 3
    -34.613476, -58.401211    % Waypoint 4 (Goal)
];

num_waypoints = size(waypoints_geo, 1);

%% 2. Konversi Waypoints Lat/Lon -> ENU Meter (Origin Locking 1x)
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

fprintf('[INIT] Home Point Locked at Lat: %.6f, Lon: %.6f\n', lat_0, lon_0);
fprintf('[INIT] %d Waypoints dikonversi ke ENU Meter.\n', num_waypoints);
for i = 1:num_waypoints
    fprintf('  -> WP %d: X = %.2f m, Y = %.2f m\n', i, waypoints_enu(i,1), waypoints_enu(i,2));
end

%% 3. Parameter Simulasi & Kapal
N = 30;             % Horizon prediksi (sama dengan Tp di nmpc_kapal_waypoint.m)
T_sim = 1.0;        % Waktu sampling (detik)
T_final = 200;      % Durasi simulasi total (detik)
N_steps = round(T_final / T_sim);

L = 101.07;         % Panjang kapal (meter)
u_0 = 15.4;         % Kecepatan nominal (m/s)
u_0_nd = 1;         % Kecepatan non-dimensional
dt_nd = T_sim * u_0 / L;

r_tran = 30.0;      % Switching Radius r_tran (meter) ala Paper Potočnik

%% 4. Setup State Awal & Penampung Data Simulasi
% Posisi awal di ENU adalah (0,0)
Heading_Awal_rad = deg2rad(Heading_Awal_deg);
current_state_nd = [0; 0; 0; 0; Heading_Awal_rad]; % [v; r; x; y; psi] non-dimensional
u_prev = 0;

active_wp_idx = 1;  % Waypoint target aktif saat ini (1 s/d num_waypoints)

history_input      = zeros(1, N_steps);
hist_state_nd      = zeros(5, N_steps);
hist_active_wp     = zeros(1, N_steps);
hist_theta_target  = zeros(1, N_steps);
hist_lat_ship      = zeros(1, N_steps);
hist_lon_ship      = zeros(1, N_steps);

%% 5. Matriks Sistem Dinamika Plant Kapal
B_ship = 14; T_ship = 3.7; m = 2423*1e3; C_B = 0.65; x_G = 5.25; rho = 1024;
Y_v_dot = -(1 + 0.16*C_B*B_ship/T_ship - 5.1*(B_ship/L)^2)*pi*(T_ship/L)^2;
Y_r_dot = -(0.67*(B_ship/L) - 0.0033*(B_ship/T_ship)^2)*pi*(T_ship/L)^2;
N_v_dot = -(1.1*B_ship/L - 0.041*B_ship/T_ship)*pi*(T_ship/L)^2;
N_r_dot = -((1/12) + 0.017*C_B*B_ship/T_ship - 0.33*B_ship/L)*pi*(T_ship/L)^2;
Y_v = -(1 + 0.4*C_B*B_ship/T_ship)*pi*(T_ship/L)^2;
Y_r = -(-0.5 + 2.2*B_ship/L - 0.08*B_ship/T_ship)*pi*(T_ship/L)^2;
N_v = -(0.5 + 2.4*T_ship/L)*pi*(T_ship/L)^2;
N_r = -(0.25 + 0.039*B_ship/T_ship - 0.56*B_ship/L)*pi*(T_ship/L)^2;

m_nd = 2*m/(rho*L^3); x_G_nd = x_G/L; I_z_nd = 1.2392*10^(-4);

M_mat = [m_nd - Y_v_dot,         m_nd*x_G_nd - Y_r_dot;
         m_nd*x_G_nd - N_v_dot,  I_z_nd - N_r_dot];

a11 = ((I_z_nd - N_r_dot)*Y_v - (m_nd*x_G_nd - Y_r_dot)*N_v) / det(M_mat);
a12 = ((I_z_nd - N_r_dot)*(Y_r - m_nd*u_0_nd) - (m_nd*x_G_nd - Y_r_dot)*(N_r - m_nd*x_G_nd*u_0_nd)) / det(M_mat);
a21 = ((m_nd - Y_v_dot)*N_v - (m_nd*x_G_nd - N_v_dot)*Y_v) / det(M_mat);
a22 = ((m_nd - Y_v_dot)*(N_r - m_nd*x_G_nd*u_0_nd) - (m_nd*x_G_nd - N_v_dot)*(Y_r - m_nd*u_0_nd)) / det(M_mat);

A_sys = [a11, a12; a21, a22];
B_sys = [0.01; 1];

%% 6. Loop Utama Simulasi (NMPC Waypoint Tracking)
for k = 1:N_steps
    % 6.1 Posisi Kapal Real-Time di ENU Meter
    x_ship = current_state_nd(3) * L;
    y_ship = current_state_nd(4) * L;
    psi_ship = current_state_nd(5);

    % Konversi Posisi Kapal ke Lat/Lon untuk Logging
    hist_lat_ship(k) = lat_0 + (y_ship / R_earth) * (180/pi);
    hist_lon_ship(k) = lon_0 + (x_ship / (R_earth * cosd(lat_0))) * (180/pi);

    % 6.2 Target Waypoint Aktif saat ini
    wp_target = waypoints_enu(active_wp_idx, :);
    dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);

    % 6.3 Logika Switching Radius r_tran (Paper Potočnik 2025)
    if dist_to_wp <= r_tran && active_wp_idx < num_waypoints
        fprintf('[STEP %d] Masuk Zona r_tran (%.1f m) dari WP %d. Ganti Target -> WP %d\n', ...
                k, dist_to_wp, active_wp_idx, active_wp_idx + 1);
        active_wp_idx = active_wp_idx + 1;
        wp_target = waypoints_enu(active_wp_idx, :);
        dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);
    end

    % 6.4 Hitung Target Heading Angle (theta_Target)
    theta_target = atan2(wp_target(2) - y_ship, wp_target(1) - x_ship);

    % 6.5 Dynamic Reference Horizon Generator (N-Step)
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

    % 6.6 Pemanggilan Entry Point Function nmpc_kapal_waypoint
    [u_opt, exitflag] = nmpc_kapal_waypoint(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq);

    % Simpan Riwayat Simulasi
    hist_state_nd(:, k) = current_state_nd;
    history_input(k)    = u_opt;
    hist_active_wp(k)   = active_wp_idx;
    hist_theta_target(k)= theta_target;

    % 6.7 Update State Dinamika Plant Kapal
    current_state_nd = euler_step(@(s, u) ship_dynamics(s, u, A_sys, B_sys, u_0_nd), ...
                                  current_state_nd, u_opt, dt_nd);
    u_prev = u_opt;
end

waktu = toc(timer_val);
fprintf('\n=========================================\n');
fprintf('SIMULASI NMPC WAYPOINT TRACKING SELESAI\n');
fprintf('Total Waktu Komputasi: %.4f detik\n', waktu);
fprintf('=========================================\n\n');

%% 7. Dimensionalisasi Data Output
hist_dim = [
    hist_state_nd(3, :) * L;          % X (m)
    hist_state_nd(4, :) * L;          % Y (m)
    hist_state_nd(5, :);              % Psi (rad)
    hist_state_nd(1, :) * u_0;        % v (m/s)
    hist_state_nd(2, :) * (u_0 / L)   % r (rad/s)
];

time_vector = (0:N_steps-1) * T_sim;

%% 8. Visualisasi Hasil Simulasi
% Figure 1: Trajektori Kapal & Waypoint di Koordinat ENU Meter
figure('Name', 'NMPC Waypoint Tracking (ENU Meter)', 'Color', 'w');
plot(route_enu(:,1), route_enu(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Sekuens Waypoint'); hold on;
plot(hist_dim(1,:), hist_dim(2,:), 'b-', 'LineWidth', 2.0, 'DisplayName', 'Lintasan NMPC Kapal');

% Gambar Lingkaran Switching Radius r_tran di setiap Waypoint
th_circle = linspace(0, 2*pi, 50);
for i = 1:num_waypoints
    xc = waypoints_enu(i,1) + r_tran * cos(th_circle);
    yc = waypoints_enu(i,2) + r_tran * sin(th_circle);
    plot(xc, yc, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(waypoints_enu(i,1)+10, waypoints_enu(i,2)+10, sprintf('WP %d', i), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
end

grid on; axis equal;
xlabel('Posisi East X (meter)'); ylabel('Posisi North Y (meter)');
title('Simulasi NMPC Waypoint Tracking (Koordinat ENU Meter)');
legend('Location', 'best');

% Figure 2: Trajektori Kapal di Koordinat Geografis (Lat/Lon)
figure('Name', 'Trajektori Geografis (Lat/Lon)', 'Color', 'w');
plot(waypoints_geo(:,2), waypoints_geo(:,1), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Waypoints (Lat/Lon)'); hold on;
plot(Posisi_Awal_Lon, Posisi_Awal_Lat, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
     'DisplayName', 'Start Point');
plot(hist_lon_ship, hist_lat_ship, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Jejak Kapal (Lat/Lon)');
grid on;
xlabel('Longitude (\circ)'); ylabel('Latitude (\circ)');
title('Trajektori Kapal dalam Koordinat Geografis (Lat/Lon)');
legend('Location', 'best');

% Figure 3: Dynamic States (v, r, Heading vs Target Heading)
figure('Name', 'Dynamic States', 'Color', 'w');
subplot(3,1,1);
plot(time_vector, hist_dim(4,:), 'b', 'LineWidth', 1.5); grid on;
ylabel('v (m/s)'); title('Kecepatan Sway (v)');

subplot(3,1,2);
plot(time_vector, rad2deg(hist_dim(5,:)), 'b', 'LineWidth', 1.5); grid on;
ylabel('r (deg/s)'); title('Kecepatan Yaw Rate (r)');

subplot(3,1,3);
plot(time_vector, rad2deg(hist_dim(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, rad2deg(hist_theta_target), 'r--', 'LineWidth', 1.5); grid on;
ylabel('\psi (deg)'); xlabel('Waktu (detik)'); title('Sudut Heading (\psi) vs Target Heading (\theta_{Target})');
legend('Heading Kapal (\psi)', 'Target Heading (\theta_{Target})', 'Location', 'best');

% Figure 4: Sinyal Kontrol Rudder
figure('Name', 'Sinyal Kontrol Rudder', 'Color', 'w');
plot(time_vector, rad2deg(history_input), 'k-', 'LineWidth', 1.5); grid on;
xlabel('Waktu (detik)'); ylabel('Sudut Kemudi \delta (derajat)');
title('Sinyal Kontrol Kemudi Rudder NMPC');

%% --- Local Functions untuk Simulasi Plant ---
function x_next = euler_step(f, x, u, dt)
    x_next = x + dt * f(x, u);
end

function s_dot = ship_dynamics(s, u, A_sys, B_sys, u0_nd)
    v = s(1); r = s(2); psi = s(5);
    v_r_dot = A_sys * [v; r] + B_sys * u;
    x_dot = u0_nd*cos(psi) - v*sin(psi);
    y_dot = u0_nd*sin(psi) + v*cos(psi);
    s_dot = [v_r_dot(1); v_r_dot(2); x_dot; y_dot; r];
end