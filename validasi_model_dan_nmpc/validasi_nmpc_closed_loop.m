%% =========================================================================
% TAHAP 2: VALIDASI KENDALI NMPC (CLOSED-LOOP TRAJECTORY TRACKING / REPLAY)
% Berdasarkan Audit Data Riil gerak_random.csv:
% 1. Waktu Sampling: dt = 0.10 detik (10 Hz)
% 2. Maksimal Sudut Rudder: +/- 45.0 derajat
% 3. Maksimal Perubahan Rudder: 150 - 300 deg/s (15 - 30 deg/step)
% 4. Polaritas Kemudi Servo: Inverted (u_model = -delta_servo)
% Folder: d:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\validasi_model_dan_nmpc\
% =========================================================================

clear; clc; close all;

fprintf('=======================================================\n');
fprintf('  TAHAP 2: VALIDASI KENDALI NMPC (CLOSED-LOOP SIMULATION)\n');
fprintf('  Sesuai Audit Parameter Sensor Riil gerak_random.csv\n');
fprintf('=======================================================\n\n');

%% 1. READ & PREPROCESS DATA REFERENSI (EXPERIMENTAL CSV)
csvPath = 'turning_otomatis.csv';
if ~exist(csvPath, 'file')
    error('File %s tidak ditemukan! Pastikan script berada di folder validasi_model_dan_nmpc.', csvPath);
end

fprintf('Membaca data referensi dari: %s\n', csvPath);
data = readtable(csvPath);

if ~ismember('x_enu_flat', data.Properties.VariableNames)
    error('Kolom ENU belum ada. Jalankan script konversi_latlon_ecef_enu.m terlebih dahulu!');
end

timestamp = data.timestamp;
time = timestamp - timestamp(1); % Waktu relatif (s)
N_total = height(data);

% Waktu Sampling Presisi 0.1 Detik (10 Hz)
dt_csv = mean(diff(time));
if isnan(dt_csv) || dt_csv <= 0, dt_csv = 0.10; end
T_sim = 0.10; % Waktu per iterasi = 0.1 detik

% Posisi acuan/target dari data eksperimen CSV (Reference Trajectory)
x_ref_full = data.x_enu_flat;
y_ref_full = data.y_enu_flat;

% Input kemudi manual RC asli untuk perbandingan
delta_deg_rc = data.Calc_deg_servo_1;

% Heading acuan dari data eksperimen CSV
yaw_compass_deg = data.yaw;
psi_enu_deg = 90.0 - yaw_compass_deg;
psi_ref_full_rad = unwrap(deg2rad(psi_enu_deg));

% Kecepatan surge eksperimen
speed_exp = data.speedMps;
u_0_exp = mean(speed_exp);
if u_0_exp < 0.05, u_0_exp = 1.08; end

fprintf('Data referensi berhasil dimuat (%d sampel, durasi %.2f s).\n', N_total, time(end));
fprintf('-> Waktu per iterasi (dt) : %.2f detik (10 Hz)\n', T_sim);
fprintf('-> Kecepatan rata-rata (u_0): %.2f m/s\n\n', u_0_exp);

%% 2. PARAMETER AUDIT SERVO & NMPC
% Maksimal Sudut Rudder yang Diperbolehkan (Hasil Audit CSV: -42.56 deg s/d +46.92 deg)
u_limit_deg = 45.0;            
u_limit = deg2rad(u_limit_deg); % +/- 45 derajat

% Perubahan Sudut Rudder yang Diperbolehkan per Detik (Rudder Rate Limit)
% Hasil Audit CSV: Servo RC bergerak hingga 360 deg/s. Diset 150 deg/s (15 deg per 0.1s step)
u_rate_limit_deg = 150.0;       
u_rate_limit = deg2rad(u_rate_limit_deg); 
du_max = u_rate_limit * T_sim; % 15 derajat per step 0.1s

% Setup Horizon Prediksi
Tp = 2.0;                      % Horizon prediksi 2.0 detik
N = round(Tp / T_sim);         % N = 20 steps prediksi

% Bobot Matriks Biaya (Cost Function)
Q = diag([100, 100, 100]);      % Bobot tracking [x_err, y_err, psi_err]
R = 0.005;                     % Bobot kemudi (agresif & responsif)

% Batasan Yaw Rate USV RC (45 deg/s)
r_limit_deg = 45.0;
r_limit = deg2rad(r_limit_deg);

fprintf('=== BATASAN KONTROLER NMPC USV ===\n');
fprintf('1. Maksimal Sudut Rudder (u_limit)      : +/- %.1f derajat\n', u_limit_deg);
fprintf('2. Perubahan Sudut Rudder (du_max)       : %.1f deg/step (%.1f deg/s)\n', ...
    rad2deg(du_max), u_rate_limit_deg);
fprintf('3. Sampling Time per Iterasi (T_sim)    : %.2f detik\n', T_sim);
fprintf('4. Predict Horizon (N steps)            : %d steps (%.1f detik)\n\n', N, Tp);

%% 3. PARAMETER KAPAL & STRUKTUR SISTEM (USV RC FISIK)
L = 1.2;            % Panjang USV (m)
B = 0.35;           % Lebar USV (m)
T = 0.10;           % Draft USV (m)
m = 8.5;            % Massa USV (kg)
u_0 = u_0_exp;      % Kecepatan rata-rata (~1.08 m/s)
C_B = 0.65;         
x_G = 0.05;         
rho = 1000;         

% Koefisien Hidrodinamika Nondimensional
Y_v_dot = -(1 + 0.16*C_B*B/T - 5.1*(B/L)^2)*pi*(T/L)^2;
Y_r_dot = -(0.67*(B/L) - 0.0033*(B/T)^2)*pi*(T/L)^2;
N_v_dot = -(1.1*B/L - 0.041*B/T)*pi*(T/L)^2;
N_r_dot = -((1/12) + 0.017*C_B*B/T - 0.33*B/L)*pi*(T/L)^2;
Y_v = -(1 + 0.4*C_B*B/T)*pi*(T/L)^2;
Y_r = -(-0.5 + 2.2*B/L - 0.08*B/T)*pi*(T/L)^2;
N_v = -(0.5 + 2.4*T/L)*pi*(T/L)^2;
N_r = -(0.25 + 0.039*B/T - 0.56*B/L)*pi*(T/L)^2;

m_nd = 2*m/(rho*L^3);
x_G_nd = x_G/L;
I_z_nd = 1.2392*10^(-4);
u_0_nd = 1;

M_mat = [m_nd - Y_v_dot , m_nd*x_G_nd - Y_r_dot ;
         m_nd*x_G_nd - N_v_dot , I_z_nd - N_r_dot ];

a11 = ((I_z_nd - N_r_dot)*Y_v - (m_nd*x_G_nd - Y_r_dot)*N_v) / det(M_mat);
a12 = ((I_z_nd - N_r_dot)*(Y_r - m_nd*u_0_nd) - (m_nd*x_G_nd - Y_r_dot)*(N_r - m_nd*x_G_nd*u_0_nd)) / det(M_mat);
a21 = ((m_nd - Y_v_dot)*N_v - (m_nd*x_G_nd - N_v_dot)*Y_v) / det(M_mat);
a22 = ((m_nd - Y_v_dot)*(N_r - m_nd*x_G_nd*u_0_nd) - (m_nd*x_G_nd - N_v_dot)*(Y_r - m_nd*u_0_nd)) / det(M_mat);

A_sys = [a11 , a12 ;
         a21 , a22 ];

% KOREKSI POLARITAS KEMUDI: Pada USV riil, servo positif memutar yaw rate negatif (-1.0)
B_sys = [-0.01 ;
         -1.0];

r_limit_nd = r_limit * (L / u_0);

lb = -u_limit * ones(N,1);
ub =  u_limit * ones(N,1);

options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', ...
    'MaxIterations', 100, 'OptimalityTolerance', 1e-4);

%% 4. SIMULASI KENDALI NMPC CLOSED-LOOP
history_state_nmpc_nd = zeros(5, N_total);
history_u_nmpc = zeros(N_total, 1);

% Inisialisasi kondisi awal closed-loop sesuai titik awal eksperimen
s0_dim = [0; 0; x_ref_full(1); y_ref_full(1); psi_ref_full_rad(1)];
s0_nd = dimensional_to_nondimensional(s0_dim, L, u_0);
history_state_nmpc_nd(:, 1) = s0_nd;

s_nd = s0_nd;
u_prev = 0;

fprintf('=======================================================\n');
fprintf('  MEMULAI SIMULASI KENDALI CLOSED-LOOP NMPC TRACKING   \n');
fprintf('=======================================================\n');

timer_nmpc = tic;

for k = 1:N_total-1
    dt_k = time(k+1) - time(k);
    if dt_k <= 0, dt_k = T_sim; end
    dt_nd = dt_k * u_0 / L;

    % 1. Ekstrak Horizon Referensi N-Step ke depan dari Lintasan CSV
    x_ref_seq_nd = zeros(N, 1);
    y_ref_seq_nd = zeros(N, 1);
    psi_ref_seq  = zeros(N, 1);
    
    for i = 1:N
        idx_look = min(k + i, N_total);
        x_ref_seq_nd(i) = x_ref_full(idx_look) / L;
        y_ref_seq_nd(i) = y_ref_full(idx_look) / L;
        psi_ref_seq(i)  = psi_ref_full_rad(idx_look);
    end

    % 2. Batasan Perubahan Input (du)
    [A_du, b_du] = du_constraints(N, u_prev, du_max);

    % 3. Fungsi Biaya NMPC
    cost_fun = @(U) mpc_cost(U, s_nd, u_prev, x_ref_seq_nd, y_ref_seq_nd, psi_ref_seq, ...
                             T_sim, L, u_0, A_sys, B_sys, u_0_nd, Q, R);

    % 4. Fungsi Kendala Nonlinear (Yaw Rate Limit)
    nonlcon = @(U) state_constraints(U, s_nd, T_sim, L, u_0, A_sys, B_sys, u_0_nd, r_limit_nd);

    % 5. Tebakan Awal (Warm Start)
    U0 = u_prev * ones(N,1);

    % 6. Optimasi fmincon (SQP)
    [U_opt, ~, exitflag] = fmincon(cost_fun, U0, A_du, b_du, [], [], lb, ub, nonlcon, options);

    if exitflag <= 0
        u_applied = u_prev;
    else
        u_applied = U_opt(1);
    end

    history_u_nmpc(k) = u_applied;

    % 7. Update Variabel State dengan Integrator Step
    s_nd = euler_step(@(s,u) ship_dynamics(s, u, A_sys, B_sys, u_0_nd), s_nd, u_applied, dt_nd);
    history_state_nmpc_nd(:, k+1) = s_nd;

    u_prev = u_applied;

    if mod(k, 400) == 0 || k == N_total-1
        fprintf('Progres: Step %d / %d (t = %.1f s) | Rudder Opt: %.2f deg\n', ...
            k, N_total-1, time(k), rad2deg(u_applied));
    end
end

history_u_nmpc(end) = history_u_nmpc(end-1);
waktu_komputasi_total = toc(timer_nmpc);

fprintf('SIMULASI CLOSED-LOOP NMPC SELESAI!\n');
fprintf('Total Waktu Komputasi: %.2f detik\n', waktu_komputasi_total);

%% 5. KONVERSI HASIL SIMULASI KE DIMENSIONAL & EVALUASI GALAT
history_state_nmpc_dim = zeros(5, N_total);
for i = 1:N_total
    history_state_nmpc_dim(:, i) = nondimensional_to_dimensional(history_state_nmpc_nd(:, i), L, u_0);
end

v_nmpc = history_state_nmpc_dim(1, :)';
r_nmpc = history_state_nmpc_dim(2, :)';
x_nmpc = history_state_nmpc_dim(3, :)';
y_nmpc = history_state_nmpc_dim(4, :)';
psi_nmpc_rad = history_state_nmpc_dim(5, :)';

% Perhitungan Galat Tracking NMPC vs Target CSV
err_x = x_ref_full - x_nmpc;
err_y = y_ref_full - y_nmpc;
err_2d = sqrt(err_x.^2 + err_y.^2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
rmse_2d = sqrt(mean(err_2d.^2));
mae_2d = mean(err_2d);

err_psi_rad = psi_ref_full_rad - psi_nmpc_rad;
rmse_psi_deg = rad2deg(sqrt(mean(err_psi_rad.^2)));

% Evaluasi Kehalusan Sinyal Kemudi (Total Control Variation)
delta_u_nmpc_deg = abs(diff(rad2deg(history_u_nmpc)));
delta_u_rc_deg   = abs(diff(delta_deg_rc));

tot_variation_nmpc = sum(delta_u_nmpc_deg);
tot_variation_rc   = sum(delta_u_rc_deg);

fprintf('\n=======================================================\n');
fprintf('       HASIL EVALUASI KENDALI NMPC TRAJECTORY TRACKING \n');
fprintf('=======================================================\n');
fprintf('RMSE Posisi X       : %.4f meter\n', rmse_x);
fprintf('RMSE Posisi Y       : %.4f meter\n', rmse_y);
fprintf('RMSE Posisi 2D      : %.4f meter\n', rmse_2d);
fprintf('MAE Posisi 2D       : %.4f meter\n', mae_2d);
fprintf('RMSE Yaw Heading    : %.2f derajat\n', rmse_psi_deg);
fprintf('-------------------------------------------------------\n');
fprintf('Variasi Perubahan Kemudi NMPC (Smoothness) : %.2f deg\n', tot_variation_nmpc);
fprintf('Variasi Perubahan Kemudi Manual RC (Human) : %.2f deg\n', tot_variation_rc);
if tot_variation_rc > 0
    fprintf('Tingkat Kehalusan Kemudi NMPC              : %.2f%% lebih mulus!\n', ...
        (1 - tot_variation_nmpc / tot_variation_rc) * 100);
end
fprintf('=======================================================\n\n');

%% 6. VISUALISASI GRAFIK HASIL VALIDASI NMPC
% FIGURE 1: Trajektori Tracking NMPC (Target CSV vs NMPC Closed-Loop)
figure('Name', '1. Trajectory Tracking NMPC Closed-Loop', 'NumberTitle', 'off', 'Position', [100, 100, 850, 650]);
plot(x_ref_full, y_ref_full, 'b-', 'LineWidth', 2.0); hold on;
plot(x_nmpc, y_nmpc, 'r--', 'LineWidth', 2.0);
plot(x_ref_full(1), y_ref_full(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x_ref_full(end), y_ref_full(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title('Validasi Closed-Loop: NMPC Trajectory Tracking vs Target Experimental CSV', 'FontSize', 12, 'FontWeight', 'bold');
legend('Target Reference (CSV)', 'NMPC Closed-Loop Response', 'Start Point', 'End Point', 'Location', 'best');

% FIGURE 2: Perbandingan Sinyal Kontrol Kemudi (NMPC Optimal vs Manual RC)
figure('Name', '2. Sinyal Kontrol Kemudi (NMPC vs RC)', 'NumberTitle', 'off', 'Position', [150, 150, 900, 500]);
plot(time, rad2deg(history_u_nmpc), 'r-', 'LineWidth', 1.8); hold on;
plot(time, delta_deg_rc, 'b:', 'LineWidth', 1.2);
yline(u_limit_deg, 'k--', 'Upper Limit (+45 deg)');
yline(-u_limit_deg, 'k--', 'Lower Limit (-45 deg)');
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Sudut Kemudi \\delta [derajat]', 'FontWeight', 'bold');
title('Perbandingan Perintah Kemudi: NMPC Optimal Controller vs Kemudi Manual RC (Human)', 'FontSize', 12, 'FontWeight', 'bold');
legend('NMPC Optimal Command (u_{opt})', 'Manual RC Input (Human)', 'Actuator Limits', 'Location', 'best');

% FIGURE 3: Perbandingan Posisi X(t) dan Y(t)
figure('Name', '3. Tracking Posisi X dan Y', 'NumberTitle', 'off', 'Position', [200, 200, 900, 600]);
subplot(2,1,1);
plot(time, x_ref_full, 'b-', 'LineWidth', 1.8); hold on;
plot(time, x_nmpc, 'r--', 'LineWidth', 1.8);
grid on; ylabel('Posisi X (East) [m]', 'FontWeight', 'bold');
title(sprintf('Tracking Posisi X (RMSE = %.3f m)', rmse_x));
legend('Target Reference', 'NMPC Response');

subplot(2,1,2);
plot(time, y_ref_full, 'b-', 'LineWidth', 1.8); hold on;
plot(time, y_nmpc, 'r--', 'LineWidth', 1.8);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Posisi Y (North) [m]', 'FontWeight', 'bold');
title(sprintf('Tracking Posisi Y (RMSE = %.3f m)', rmse_y));
legend('Target Reference', 'NMPC Response');

% FIGURE 4: Respon Sudut Yaw Heading & Yaw Rate
figure('Name', '4. Respon Yaw Heading & Yaw Rate NMPC', 'NumberTitle', 'off', 'Position', [250, 250, 900, 600]);
subplot(2,1,1);
plot(time, rad2deg(psi_ref_full_rad), 'b-', 'LineWidth', 1.8); hold on;
plot(time, rad2deg(psi_nmpc_rad), 'r--', 'LineWidth', 1.8);
grid on; ylabel('Yaw Heading \\psi [derajat]', 'FontWeight', 'bold');
title(sprintf('Tracking Sudut Yaw Heading (RMSE = %.2f deg)', rmse_psi_deg));
legend('Target Reference', 'NMPC Response');

subplot(2,1,2);
plot(time, rad2deg(r_nmpc), 'm-', 'LineWidth', 1.5); hold on;
yline(r_limit_deg, 'r--', sprintf('Limit (+%.1f deg/s)', r_limit_deg));
yline(-r_limit_deg, 'r--', sprintf('Limit (-%.1f deg/s)', r_limit_deg));
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Yaw Rate r [deg/s]', 'FontWeight', 'bold');
title('Kecepatan Sudut Putar Kapal (Yaw Rate r) Dalam Kendali NMPC');

% FIGURE 5: Grafik Eror Jarak Tracking 2D (Euclidean Distance Error)
figure('Name', '5. Eror Jarak Tracking 2D NMPC', 'NumberTitle', 'off', 'Position', [300, 300, 850, 450]);
plot(time, err_2d, 'r-', 'LineWidth', 1.5); hold on;
yline(rmse_2d, 'b--', sprintf('RMSE 2D = %.3f m', rmse_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(mae_2d, 'g--', sprintf('MAE 2D = %.3f m', mae_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Galat Jarak Tracking [meter]', 'FontWeight', 'bold');
title('Galat Jarak Tracking Posisi 2D (Euclidean Distance Error) NMPC');
legend('Eror Jarak Tracking', 'Nilai RMSE 2D', 'Nilai MAE 2D');

fprintf('\nValidasi Closed-Loop NMPC Selesai! Semua grafik hasil simulasi berhasil ditampilkan.\n');

%% =========================================================================
% FUNGSI-FUNGSI HELPER NMPC & INTEGRATOR
% =========================================================================

function x_next = euler_step(f, x, u, dt)
    x_next = x + dt * f(x, u);
end

function s_dot = ship_dynamics(s, u, A_sys, B_sys, u0_nd)
    v = s(1); 
    r = s(2); 
    psi = s(5);
    
    v_r_dot = A_sys * [v; r] + B_sys * u;
    x_dot = u0_nd*cos(psi) - v*sin(psi);
    y_dot = u0_nd*sin(psi) + v*cos(psi);
    
    s_dot = [v_r_dot(1); v_r_dot(2); x_dot; y_dot; r];
end

function J = mpc_cost(U, s0, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq, ...
                      T_sim, L, u0, A_sys, B_sys, u0_nd, Q, R)
    N = length(U);
    s = s0;
    dt_nd = T_sim * u0 / L;
    J = 0;
    
    for i = 1:N
        u = U(i);
        s = euler_step(@(s,u) ship_dynamics(s, u, A_sys, B_sys, u0_nd), s, u, dt_nd);
        
        err = [s(3) - x_ref_seq(i);
               s(4) - y_ref_seq(i);
               s(5) - psi_ref_seq(i)];

        J = J + err' * Q * err + R * u^2;
    end
end

function [c, ceq] = state_constraints(U, s0, T_sim, L, u0, A_sys, B_sys, u0_nd, r_limit_nd)
    N = length(U);
    s = s0;
    dt_nd = T_sim * u0 / L;
    c = [];
    
    for i = 1:N
        s = euler_step(@(s,u) ship_dynamics(s, u, A_sys, B_sys, u0_nd), s, U(i), dt_nd);
        r = s(2);
        c = [c; r - r_limit_nd; -r_limit_nd - r];
    end
    ceq = [];
end

function [A, b] = du_constraints(N, u_prev, du_max)
    A = zeros(2*N, N);
    b = zeros(2*N, 1);
    
    A(1,1) = 1;   b(1) = u_prev + du_max;
    for i = 2:N
        A(i, i-1) = -1;   A(i, i) = 1;
        b(i) = du_max;
    end
    
    A(N+1,1) = -1;   b(N+1) = du_max - u_prev;
    for i = 2:N
        A(N+i, i-1) = 1;   A(N+i, i) = -1;
        b(N+i) = du_max;
    end
end

function x_nd = dimensional_to_nondimensional(x, L, u0)
    x_nd = [x(1)/u0; x(2)*L/u0; x(3)/L; x(4)/L; x(5)];
end

function x_dim = nondimensional_to_dimensional(x, L, u0)
    x_dim = [x(1)*u0; x(2)*u0/L; x(3)*L; x(4)/L; x(5)];
end
