% ==============================================================================================
%               NMPC WAYPOINT TRACKING USV AUTONOMOUS - DANAU 8 ITS
% * Lokasi Simulasi  : Danau 8 ITS Surabaya (Latitude -7.28715, Longitude 112.79600)
% * Spesifikasi Kapal: USV RC (L = 1.2 m, B = 0.35 m, T = 0.10 m, m = 8.5 kg, u_0 = 1.08 m/s)
% * Output           : Video MP4 Simulasi lengkap Peta Danau 8 ITS & Grafik Respon Sinyal
% ===============================================================================================
clc; clear; close all;

fprintf('=======================================================================\n');
fprintf('   SIMULASI NMPC WAYPOINT TRACKING USV AUTONOMOUS (DANAU 8 ITS)\n');
fprintf('=======================================================================\n\n');

%% 1. PARAMETER KAPAL & DINAMIKA PLANT (USV RC FISIK)
L_ship   = 1.2;            % Panjang kapal USV (m)
B_ship   = 0.35;           % Lebar kapal USV (m)
T_ship   = 0.10;           % Draft kapal USV (m)
m_ship   = 8.5;            % Massa kapal USV (kg)
u_0_ship = 1.08;           % Kecepatan surge rata-rata USV (m/s)
C_B      = 0.65;           % Koefisien Blok
x_G_ship = 0.05;           % Pusat massa sumbu-x (m)
rho      = 1000;           % Massa jenis air tawar danau (kg/m^3)

% Koefisien Hidrodinamika Nondimensional USV RC
Y_v_dot = -(1 + 0.16*C_B*B_ship/T_ship - 5.1*(B_ship/L_ship)^2)*pi*(T_ship/L_ship)^2;
Y_r_dot = -(0.67*(B_ship/L_ship) - 0.0033*(B_ship/T_ship)^2)*pi*(T_ship/L_ship)^2;
N_v_dot = -(1.1*B_ship/L_ship - 0.041*B_ship/T_ship)*pi*(T_ship/L_ship)^2;
N_r_dot = -((1/12) + 0.017*C_B*B_ship/T_ship - 0.33*B_ship/L_ship)*pi*(T_ship/L_ship)^2;
Y_v     = -(1 + 0.4*C_B*B_ship/T_ship)*pi*(T_ship/L_ship)^2;
Y_r     = -(-0.5 + 2.2*B_ship/L_ship - 0.08*B_ship/T_ship)*pi*(T_ship/L_ship)^2;
N_v     = -(0.5 + 2.4*T_ship/L_ship)*pi*(T_ship/L_ship)^2;
N_r     = -(0.25 + 0.039*B_ship/T_ship - 0.56*B_ship/L_ship)*pi*(T_ship/L_ship)^2;

m_nd_ship   = 2*m_ship/(rho*L_ship^3);
x_G_nd_ship = x_G_ship/L_ship;
I_z_nd_ship = 1.2392*10^(-4);

M_mat_ship = [m_nd_ship - Y_v_dot , m_nd_ship*x_G_nd_ship - Y_r_dot ;
              m_nd_ship*x_G_nd_ship - N_v_dot , I_z_nd_ship - N_r_dot ];

a11_s = ((I_z_nd_ship - N_r_dot)*Y_v - (m_nd_ship*x_G_nd_ship - Y_r_dot)*N_v)/det(M_mat_ship);
a12_s = ((I_z_nd_ship - N_r_dot)*(Y_r - m_nd_ship) - (m_nd_ship*x_G_nd_ship - Y_r_dot)*(N_r - m_nd_ship*x_G_nd_ship))/det(M_mat_ship);
a21_s = ((m_nd_ship - Y_v_dot)*N_v - (m_nd_ship*x_G_nd_ship - N_v_dot)*Y_v)/det(M_mat_ship);
a22_s = ((m_nd_ship - Y_v_dot)*(N_r - m_nd_ship*x_G_nd_ship) - (m_nd_ship*x_G_nd_ship - N_v_dot)*(Y_r - m_nd_ship))/det(M_mat_ship);

A_sys_ship = [a11_s , a12_s ; a21_s , a22_s ];
B_sys_ship = [0.01 ; 1];

%% 2. KOORDINAT GEOGRAFIS DANAU 8 ITS & WAYPOINTS
Posisi_Awal_Lat  = -7.28715;
Posisi_Awal_Lon  = 112.79600;
Heading_Awal_deg = 90; % Heading awal menghadap ke Utara (ENU 90 deg)

% Batas Polygon Perairan Danau 8 ITS (21 Titik Koordinat Presisi)
lake_lat = [ -7.2864264, -7.2865324, -7.2866007, -7.2866566, -7.2868322, -7.2870184, -7.2871248, -7.2872259, -7.2873058, -7.2872738, -7.2872738, -7.2871568, -7.2869377, -7.2868893, -7.2867524, -7.2865661, -7.2863214, -7.2861830, -7.2861724, -7.2862096, -7.2863054, -7.2864264 ];
lake_lon = [ 112.7957422, 112.7958015, 112.7959085, 112.7959545, 112.7959648, 112.7959648, 112.7958736, 112.7959112, 112.7960077, 112.7961472, 112.7962652, 112.7963403, 112.7962630, 112.7961728, 112.7961258, 112.7961633, 112.7962545, 112.7961901, 112.7960131, 112.7958522, 112.7957556, 112.7957422 ];

% Sekuens Waypoints Presisi Dalam Air Danau 8 ITS
waypoints_geo = [
    -7.28675, 112.79600;   % WP 1: Penyempitan Tengah
    -7.28630, 112.79600;   % WP 2: Kolam Utara
    -7.28675, 112.79610;   % WP 3: Sisi Timur Tengah
    -7.28715, 112.79600    % WP 4: Kolam Selatan (Kembali ke titik awal)
];

num_waypoints = size(waypoints_geo, 1);

%% 3. KONVERSI LAT/LON KE KOORDINAT LOKAL ENU (METER)
R_earth = 6371000;
lat_0   = Posisi_Awal_Lat;
lon_0   = Posisi_Awal_Lon;

waypoints_enu = zeros(num_waypoints, 2);
for i = 1:num_waypoints
    dLat = deg2rad(waypoints_geo(i, 1) - lat_0);
    dLon = deg2rad(waypoints_geo(i, 2) - lon_0);
    
    y_north = R_earth * dLat;
    x_east  = R_earth * cosd(lat_0) * dLon;
    
    waypoints_enu(i, :) = [x_east, y_north];
end

lake_enu_x = R_earth * cosd(lat_0) * deg2rad(lake_lon - lon_0);
lake_enu_y = R_earth * deg2rad(lake_lat - lat_0);

route_enu = [0, 0; waypoints_enu];

fprintf('[INIT] Home Point Danau 8 ITS: Lat %.6f, Lon %.6f\n', lat_0, lon_0);
fprintf('[INIT] %d Waypoints terkonversi ke ENU Meter:\n', num_waypoints);
for i = 1:num_waypoints
    fprintf('  -> WP %d: Lat = %.6f, Lon = %.6f | X = %.2f m, Y = %.2f m\n', ...
            i, waypoints_geo(i,1), waypoints_geo(i,2), waypoints_enu(i,1), waypoints_enu(i,2));
end

%% 4. PARAMETER SIMULASI & SWITCHING RADIUS
N       = 30;             % Horizon prediksi
T_sim   = 0.1;            % Waktu sampling 10 Hz (0.1 detik)
T_final = 220;            % Durasi simulasi maksimum (detik)
N_steps = round(T_final / T_sim);

r_tran  = 5;            % Switching Radius r_tran USV (3.5 meter)

%% 5. SETUP STATE AWAL
Heading_Awal_rad = deg2rad(Heading_Awal_deg);
current_state_dim = [0; 0; 0; 0; Heading_Awal_rad]; % [v; r; x; y; psi]
u_prev = 0;

active_wp_idx = 1;

history_input      = zeros(1, N_steps);
hist_state_dim     = zeros(5, N_steps);
hist_active_wp     = zeros(1, N_steps);
hist_theta_target  = zeros(1, N_steps);
hist_lat_ship      = zeros(1, N_steps);
hist_lon_ship      = zeros(1, N_steps);

%% 6. LOOP UTAMA SIMULASI NMPC (CEPAT)
fprintf('\n[SIM] Memulai simulasi NMPC Loop...\n');
timer_val = tic;
actual_steps = N_steps;

for k = 1:N_steps
    x_ship   = current_state_dim(3);
    y_ship   = current_state_dim(4);
    psi_ship = current_state_dim(5);

    hist_lat_ship(k) = lat_0 + (y_ship / R_earth) * (180/pi);
    hist_lon_ship(k) = lon_0 + (x_ship / (R_earth * cosd(lat_0))) * (180/pi);

    wp_target = waypoints_enu(active_wp_idx, :);
    dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);

    if dist_to_wp <= r_tran && active_wp_idx < num_waypoints
        fprintf('[STEP %d | t=%.1fs] Masuk Radius r_tran (%.2fm) WP %d -> Target WP %d\n', ...
                k, k*T_sim, dist_to_wp, active_wp_idx, active_wp_idx + 1);
        active_wp_idx = active_wp_idx + 1;
        wp_target = waypoints_enu(active_wp_idx, :);
        dist_to_wp = hypot(wp_target(1) - x_ship, wp_target(2) - y_ship);
    end

    theta_target = atan2(wp_target(2) - y_ship, wp_target(1) - x_ship);

    x_ref_seq   = zeros(N, 1);
    y_ref_seq   = zeros(N, 1);
    psi_ref_seq = zeros(N, 1);

    for h = 1:N
        dist_step   = h * u_0_ship * T_sim;
        x_ref_dim_h = x_ship + dist_step * cos(theta_target);
        y_ref_dim_h = y_ship + dist_step * sin(theta_target);
        
        x_ref_seq(h)   = x_ref_dim_h / L_ship;
        y_ref_seq(h)   = y_ref_dim_h / L_ship;
        psi_ref_seq(h) = theta_target;
    end

    current_state_nd = [
        current_state_dim(1) / u_0_ship;
        current_state_dim(2) * (L_ship / u_0_ship);
        current_state_dim(3) / L_ship;
        current_state_dim(4) / L_ship;
        current_state_dim(5)
    ];

    % Pemanggilan NMPC Solver khusus USV Danau 8 ITS (Tanpa scaling 101.07 m)
    [u_opt, exitflag] = nmpc_kapal_waypoint_danau8its(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq);

    if exitflag <= 0 || isnan(u_opt) || isinf(u_opt)
        u_opt = u_prev;
    end

    hist_state_dim(:, k) = current_state_dim;
    history_input(k)     = u_opt;
    hist_active_wp(k)    = active_wp_idx;
    hist_theta_target(k) = theta_target;

    % Update State Dinamika Plant USV RC menggunakan RK4 Integration
    n_sub = 4;
    dt_sub = T_sim / n_sub;
    s_sub = current_state_dim;
    for sub = 1:n_sub
        s_sub = rk4_step(@(s, u) ship_dynamics_dim(s, u, A_sys_ship, B_sys_ship, u_0_ship, L_ship), ...
                         s_sub, u_opt, dt_sub);
    end
    current_state_dim = s_sub;
    u_prev = u_opt;

    if active_wp_idx == num_waypoints && dist_to_wp <= r_tran
        fprintf('\n[GOAL] USV telah berhasil mencapai Waypoint Terakhir (WP %d, Jarak: %.2f m)!\n', ...
                num_waypoints, dist_to_wp);
        fprintf('[GOAL] Simulasi dihentikan pada Step %d (t = %.1f s).\n', k, k * T_sim);
        actual_steps = k;
        break;
    end
end

waktu_komputasi = toc(timer_val);

% Truncate data
history_input     = history_input(1:actual_steps);
hist_state_dim    = hist_state_dim(:, 1:actual_steps);
hist_active_wp    = hist_active_wp(1:actual_steps);
hist_theta_target = hist_theta_target(1:actual_steps);
hist_lat_ship     = hist_lat_ship(1:actual_steps);
hist_lon_ship     = hist_lon_ship(1:actual_steps);
time_vector       = (0:actual_steps-1) * T_sim;

fprintf('\n=======================================================\n');
fprintf('  SIMULASI NMPC DANAU 8 ITS SELESAI\n');
fprintf('  Total Waktu Simulasi : %.1f detik (%d steps)\n', actual_steps * T_sim, actual_steps);
fprintf('  Waktu Komputasi      : %.4f detik\n', waktu_komputasi);
fprintf('=======================================================\n\n');

%% 7. GENERASI VIDEO ANIMASI SIMULASI MP4
video_filename = 'simulasi_nmpc_danau8its_video.mp4';
fprintf('[VIDEO] Membuat dan mengekspor video animasi simulasi MP4: %s...\n', video_filename);

fig = figure('Name', 'Simulasi NMPC Waypoint Tracking - Danau 8 ITS', ...
             'Color', 'w', 'Position', [100, 100, 1000, 600], 'Visible', 'off');

ax_map = subplot(4, 4, [1 2 3 5 6 7 9 10 11 13 14 15]);
hold(ax_map, 'on'); grid(ax_map, 'on'); axis(ax_map, 'equal');
xlabel(ax_map, 'Posisi East X (meter)', 'FontWeight', 'bold');
ylabel(ax_map, 'Posisi North Y (meter)', 'FontWeight', 'bold');
title(ax_map, 'Peta Danau 8 ITS & Real-Time Trajektori USV NMPC', 'FontSize', 11, 'FontWeight', 'bold');

fill(ax_map, lake_enu_x, lake_enu_y, [0.85 0.93 1.0], 'EdgeColor', [0.2 0.5 0.9], 'LineWidth', 2.0, 'DisplayName', 'Perairan Danau 8 ITS');
plot(ax_map, route_enu(:,1), route_enu(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 7, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Rute Waypoint');

th_circle = linspace(0, 2*pi, 40);
for i = 1:num_waypoints
    xc = waypoints_enu(i,1) + r_tran * cos(th_circle);
    yc = waypoints_enu(i,2) + r_tran * sin(th_circle);
    plot(ax_map, xc, yc, 'm:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(ax_map, waypoints_enu(i,1)+1.2, waypoints_enu(i,2)+1.2, sprintf('WP %d', i), ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.7 0 0]);
end

h_traj = plot(ax_map, 0, 0, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Lintasan Kapal USV');
h_ship = plot(ax_map, 0, 0, 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g', 'DisplayName', 'Posisi Kapal');
legend(ax_map, 'Location', 'northeast');

ax_v   = subplot(4, 4, 4);
ax_r   = subplot(4, 4, 8);
ax_psi = subplot(4, 4, 12);
ax_u   = subplot(4, 4, 16);

try
    vObj = VideoWriter(video_filename, 'MPEG-4');
    vObj.FrameRate = 15;
    vObj.Quality   = 95;
    open(vObj);

    step_stride = 10; % 1 fps video per second simulation
    for k = 1:step_stride:actual_steps
        set(h_traj, 'XData', hist_state_dim(3, 1:k), 'YData', hist_state_dim(4, 1:k));
        set(h_ship, 'XData', hist_state_dim(3, k), 'YData', hist_state_dim(4, k));

        t_v = time_vector(1:k);
        
        % Subplot 1: Sway velocity
        plot(ax_v, t_v, hist_state_dim(1, 1:k), 'b', 'LineWidth', 1.2); grid(ax_v, 'on');
        ylabel(ax_v, 'v (m/s)'); title(ax_v, 'Sway (v)');
        
        % Subplot 2: Yaw Rate vs Batasan r_limit (+-5.34 deg/s)
        plot(ax_r, t_v, rad2deg(hist_state_dim(2, 1:k)), 'b', 'LineWidth', 1.2); hold(ax_r, 'on');
        yline(ax_r, 5.34, 'r--', 'LineWidth', 1.0);
        yline(ax_r, -5.34, 'r--', 'LineWidth', 1.0); hold(ax_r, 'off'); grid(ax_r, 'on');
        ylabel(ax_r, 'r (deg/s)'); title(ax_r, 'Yaw Rate (r) vs Limit (\pm5.34^\circ/s)');
        
        % Subplot 3: Heading vs Target Heading
        plot(ax_psi, t_v, rad2deg(hist_state_dim(5, 1:k)), 'b-', 'LineWidth', 1.2); hold(ax_psi, 'on');
        plot(ax_psi, t_v, rad2deg(hist_theta_target(1:k)), 'r--', 'LineWidth', 1.2); grid(ax_psi, 'on'); hold(ax_psi, 'off');
        ylabel(ax_psi, '\psi (deg)'); title(ax_psi, 'Heading (\psi)');
        
        % Subplot 4: Rudder Angle vs Batasan u_limit (+-35 deg)
        plot(ax_u, t_v, rad2deg(history_input(1:k)), 'k-', 'LineWidth', 1.2); hold(ax_u, 'on');
        yline(ax_u, 35, 'r--', 'LineWidth', 1.0);
        yline(ax_u, -35, 'r--', 'LineWidth', 1.0); hold(ax_u, 'off'); grid(ax_u, 'on');
        ylabel(ax_u, '\delta (deg)'); xlabel(ax_u, 'Waktu (s)'); title(ax_u, 'Rudder (\delta) vs Limit (\pm35^\circ)');
        
        % Rendering offscreen frame presisi tinggi
        frame_rgb = print(fig, '-RGBImage', '-r80');
        writeVideo(vObj, frame_rgb);
    end
    close(vObj);
    fprintf('[VIDEO] Video MP4 berhasil disimpan secara presisi: %s\n', video_filename);
catch ME
    warning('[VIDEO] Ekspor video gagal (%s). Menyiapkan plot statis akhir.', ME.message);
end

%% 8. VISUALISASI GRAFIK HASIL AKHIR & BATASAN KENDALI
saveas(fig, 'hasil_simulasi_danau8its.png');
fprintf('[PLOT] Gambar grafik hasil simulasi disimpan: hasil_simulasi_danau8its.png\n');

% Figure 3: Dedicated Plot Evaluasi Batasan Kendali NMPC (Yaw Rate, Rudder Angle, Rudder Rate)
fig3 = figure('Name', 'Evaluasi Batasan Kendali NMPC USV', 'Color', 'w', 'Visible', 'off', 'Position', [100, 100, 900, 680]);

% Perhitungan Rate Perubahan Sudut Rudder (deg/s)
rudder_deg = rad2deg(history_input);
rudder_rate_deg_s = [0, diff(rudder_deg) / T_sim];

% Subplot 1: Yaw Rate (r) vs Limit
subplot(3, 1, 1);
plot(time_vector, rad2deg(hist_state_dim(2, :)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Yaw Rate Response'); hold on;
yline(5.34, 'r--', 'Limit (+5.34^\circ/s)', 'LineWidth', 1.2, 'DisplayName', 'Batas Max (+5.34^\circ/s)');
yline(-5.34, 'r--', 'Limit (-5.34^\circ/s)', 'LineWidth', 1.2, 'DisplayName', 'Batas Min (-5.34^\circ/s)');
grid on; ylabel('r (deg/s)', 'FontWeight', 'bold');
title('1. Evaluasi Batasan Kecepatan Sudut Putar Kapal (Yaw Rate r \le 5.34^\circ/s)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast');

% Subplot 2: Sudut Rudder (delta) vs Limit
subplot(3, 1, 2);
plot(time_vector, rudder_deg, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Perintah Rudder NMPC'); hold on;
yline(35, 'r--', 'Limit (+35^\circ)', 'LineWidth', 1.2, 'DisplayName', 'Batas Max (+35^\circ)');
yline(-35, 'r--', 'Limit (-35^\circ)', 'LineWidth', 1.2, 'DisplayName', 'Batas Min (-35^\circ)');
grid on; ylabel('\delta (deg)', 'FontWeight', 'bold');
title('2. Evaluasi Batasan Sudut Kemudi Rudder (\delta \le 35^\circ)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast');

% Subplot 3: Perubahan Sudut Rudder per Step (Rudder Delta Rate) vs Limit (5 deg/step)
subplot(3, 1, 3);
rudder_diff_step = [0, abs(diff(rudder_deg))];
plot(time_vector, rudder_diff_step, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Perubahan Sudut Rudder (\Delta\delta)'); hold on;
yline(5, 'r--', 'Limit (+5^\circ/step)', 'LineWidth', 1.2, 'DisplayName', 'Batas Max (+5^\circ/step)');
grid on; xlabel('Waktu (detik)', 'FontWeight', 'bold'); ylabel('\Delta\delta (deg)', 'FontWeight', 'bold');
title('3. Evaluasi Batasan Perubahan Sudut Rudder (\Delta\delta \le 5^\circ per step)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast');

saveas(fig3, 'grafik_batasan_kendali.png');
fprintf('[PLOT] Gambar evaluasi batasan kendali disimpan: grafik_batasan_kendali.png\n');

try
    fig2 = figure('Name', 'Trajektori Geografis Danau 8 ITS (Lat/Lon)', 'Color', 'w', 'Visible', 'off');
    geoplot(lake_lat, lake_lon, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Batas Perairan Danau 8 ITS'); hold on;
    geoplot(waypoints_geo(:,1), waypoints_geo(:,2), 'r--o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
            'MarkerFaceColor', 'r', 'DisplayName', 'Sekuens Waypoints');
    geoplot(Posisi_Awal_Lat, Posisi_Awal_Lon, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');
    geoplot(hist_lat_ship, hist_lon_ship, 'm-', 'LineWidth', 2.2, 'DisplayName', 'Trajektori USV NMPC');
    geobasemap streets-light;
    title('Trajektori NMPC Waypoint USV di Koordinat Geografis Danau 8 ITS');
    legend('Location', 'northeast');
    saveas(fig2, 'trajektori_geografis_danau8its.png');
catch ME
    warning('Geoplot tidak tersedia atau luring: %s', ME.message);
end

close all;
fprintf('=== SEMUA PROSES SIMULASI & GENERASI HASIL SELESAI SANTAI ===\n');

%% LOCAL FUNCTIONS
function x_next = rk4_step(f, x, u, dt)
    k1 = f(x, u);
    k2 = f(x + 0.5 * dt * k1, u);
    k3 = f(x + 0.5 * dt * k2, u);
    k4 = f(x + dt * k3, u);
    x_next = x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
end

function s_dot = ship_dynamics_dim(s, u, A_sys, B_sys, u0, L_ship)
    v = s(1); r = s(2); psi = s(5);
    v_r_dot_nd = A_sys * [v/u0; r*L_ship/u0] + B_sys * u;
    
    v_dot = v_r_dot_nd(1) * (u0^2 / L_ship);
    r_dot = v_r_dot_nd(2) * (u0^2 / (L_ship^2));
    
    x_dot = u0 * cos(psi) - v * sin(psi);
    y_dot = u0 * sin(psi) + v * cos(psi);
    psi_dot = r;
    
    s_dot = [v_dot; r_dot; x_dot; y_dot; psi_dot];
end
