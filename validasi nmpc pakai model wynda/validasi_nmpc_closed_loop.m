%% =========================================================================
% TAHAP 2: VALIDASI KENDALI NMPC (CLOSED-LOOP TRAJECTORY TRACKING / REPLAY)
% DENGAN MODEL MATEMATIKA WYNDA (11 PARAMETER BASIS)
% SPESIFIKASI SKALA FROUDE (1:100) — KOORDINAT LOCAL ENU
% =========================================================================
% Spesifikasi Kendali & Model:
% 1. Batasan Sudut Rudder           : +/- 45.0 derajat
% 2. Batasan Perubahan Sudut Rudder : +/- 30.0 derajat per step
% 3. Model Matematika & Parameter   : WyNDA 11 Parameter (Skala Froude L=1.0107 m)
% 4. Metode Integrasi               : Metode Euler (euler_step)
% 5. Lintasan Referensi             : hasil_preprocessing_ENU(data_FIKS).xlsx (x, y, psi)
% 6. Kalibrasi Heading (psi)        : Unwrapped & Shortest-Angular Distance Handling
% 7. Validasi Kemudi                : Perbandingan Sudut Rudder NMPC vs Eksperimen
% =========================================================================

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('=========================================================================\n');
fprintf('  VALIDASI KENDALI CLOSED-LOOP NMPC USV DENGAN MODEL WYNDA (11 PARAMETER) \n');
fprintf('  Lintasan Referensi: hasil_preprocessing_ENU(data_FIKS).xlsx             \n');
fprintf('=========================================================================\n\n');

%% 1. READ & PREPROCESS DATA REFERENSI (EXCEL PREPROCESSED ENU)
dataFile = 'hasil_preprocessing_ENU(data_FIKS).xlsx';
if ~exist(dataFile, 'file')
    error('File %s tidak ditemukan! Pastikan file Excel berada di folder kerja.', dataFile);
end

fprintf('1. Membaca data referensi dari: %s\n', dataFile);
data = readtable(dataFile);
N_total = height(data);

% Ekstraksi Koordinat Posisi ENU Referensi
x_ref_full = data.x_ENU;
y_ref_full = data.y_ENU;

% Ekstraksi & Kalibrasi Heading psi (unwrap untuk mencegah diskontinuitas)
psi_ref_full = unwrap(data.psi);

% Ekstraksi Sudut Rudder Eksperimen (Acuan Validasi Kemudi)
delta_exp_rad = data.delta;
delta_exp_deg = rad2deg(delta_exp_rad);

% Ekstraksi Kecepatan Surge Nominal u_0
if ismember('u_exp', data.Properties.VariableNames)
    u_0_exp = mean(data.u_exp);
    if isnan(u_0_exp) || u_0_exp <= 0, u_0_exp = 0.6114; end
else
    u_0_exp = 0.6114; % Nominal default [m/s]
end

% Ekstraksi State Sway (v) dan Yaw Rate (r) dari Data Eksperimen
if ismember('v', data.Properties.VariableNames), v_exp = data.v; else, v_exp = zeros(N_total, 1); end
if ismember('r', data.Properties.VariableNames), r_exp = data.r; else, r_exp = zeros(N_total, 1); end

% Waktu Sampling Presisi 0.1 Detik (10 Hz)
T_sim = 0.10;
time = (0:N_total-1)' * T_sim;

fprintf('   Total sampel data      : %d titik\n', N_total);
fprintf('   Durasi eksperimen      : %.2f detik\n', time(end));
fprintf('   Sampling time per step : %.2f detik (10 Hz)\n', T_sim);
fprintf('   Kecepatan surge (u_0)  : %.4f m/s\n\n', u_0_exp);

%% 2. PARAMETER KAPAL & MODEL MATEMATIKA WYNDA (11 PARAMETER)
% Spesifikasi Dimensi Skala Froude (1:100)
L = 1.0107;          % Panjang kapal model [meter]
u_0 = u_0_exp;       % Kecepatan surge referensi [m/s]
u_0_nd = 1.0;        % Kecepatan surge nondimensional

% Vektor Parameter Theta Hasil Identifikasi WyNDA 11 Basis Fungsi:
% Eq (4.27) -> v_dot   = theta_1*v + theta_2*r + theta_3*delta
% Eq (4.28) -> r_dot   = theta_4*v + theta_5*r + theta_6*delta
% Eq (4.29) -> x_dot   = theta_7*u0*cos(psi) - theta_8*v*sin(psi)
% Eq (4.30) -> y_dot   = theta_9*u0*sin(psi) + theta_10*v*cos(psi)
% Eq (4.31) -> psi_dot = theta_11*r

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

fprintf('2. Parameter Model WyNDA 11 Basis Terdefinisi:\n');
fprintf('   Panjang Kapal L = %.4f m | Kecepatan u_0 = %.4f m/s\n\n', L, u_0);

%% 3. SETUP KONTROLER NMPC & BATASAN AKTISUASI (KENDALA)
% 1. Batas Maksimal & Minimal Sudut Rudder: +/- 45 derajat
u_limit_deg = 45.0;
u_limit = deg2rad(u_limit_deg);

% 2. Batas Maksimal & Minimal Perubahan Sudut Rudder: +/- 30 derajat per step
du_max_deg = 30.0;
du_max = deg2rad(du_max_deg);

% Horizon Prediksi NMPC
Tp = 3;                      % Horizon waktu prediksi (detik)
N = round(Tp / T_sim);         % N = 15 steps

% Matriks Bobot Biaya NMPC (Cost Function)
Q = diag([120, 120, 60]);      % Bobot tracking [x_err, y_err, psi_err]
R = 0.008;                     % Bobot penalti besaran sudut rudder

% Batasan Yaw Rate Kapal
r_limit_deg = 45.0;
r_limit = deg2rad(r_limit_deg);
r_limit_nd = r_limit * (L / u_0);

% Batas Nilai Variabel Optimasi (Lower Bound & Upper Bound)
lb = -u_limit * ones(N, 1);
ub =  u_limit * ones(N, 1);

% Opsi Optimasi fmincon (Algoritma SQP Cepat)
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', ...
    'MaxIterations', 60, 'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-4);

fprintf('3. Konfigurasi Kontroler NMPC:\n');
fprintf('   - Batas Sudut Rudder (u_limit)    : +/- %.1f derajat\n', u_limit_deg);
fprintf('   - Batas Perubahan Rudder (du_max) : +/- %.1f deg/step (%.1f deg/s)\n', ...
    du_max_deg, du_max_deg / T_sim);
fprintf('   - Horizon Prediksi (N steps)      : %d steps (%.1f detik)\n\n', N, Tp);

%% 4. SIMULASI KENDALI CLOSED-LOOP NMPC TRACKING
history_state_nmpc_nd = zeros(5, N_total);
history_u_nmpc = zeros(N_total, 1);

% Inisialisasi Kondisi Awal Kapal (State t=0)
s0_dim = [v_exp(1); r_exp(1); x_ref_full(1); y_ref_full(1); psi_ref_full(1)];
s0_nd = dimensional_to_nondimensional(s0_dim, L, u_0);
history_state_nmpc_nd(:, 1) = s0_nd;

s_nd = s0_nd;
u_prev = delta_exp_rad(1);

fprintf('=========================================================================\n');
fprintf('  MEMULAI SIMULASI KENDALI CLOSED-LOOP NMPC TRACKING (METODE EULER)      \n');
fprintf('=========================================================================\n');

timer_nmpc = tic;

for k = 1:N_total-1
    % 1. Ekstrak Horizon Referensi N-Step ke Depan dari Lintasan Excel
    x_ref_seq_nd = zeros(N, 1);
    y_ref_seq_nd = zeros(N, 1);
    psi_ref_seq  = zeros(N, 1);
    
    for i = 1:N
        idx_look = min(k + i, N_total);
        x_ref_seq_nd(i) = x_ref_full(idx_look) / L;
        y_ref_seq_nd(i) = y_ref_full(idx_look) / L;
        psi_ref_seq(i)  = psi_ref_full(idx_look);
    end

    % 2. Matriks Kendala Laju Perubahan Kemudi: |u(i) - u(i-1)| <= du_max
    [A_du, b_du] = du_constraints(N, u_prev, du_max);

    % 3. Fungsi Biaya (Cost Function) NMPC: Tracking Error + Penalti Sudut Rudder
    cost_fun = @(U) mpc_cost(U, s_nd, x_ref_seq_nd, y_ref_seq_nd, psi_ref_seq, ...
                             theta, Q, R);

    % 4. Fungsi Kendala Nonlinear State (Yaw Rate Limit)
    nonlcon = @(U) state_constraints(U, s_nd, theta, r_limit_nd);

    % 5. Inisialisasi Tebakan Awal (Warm Start)
    U0 = u_prev * ones(N, 1);

    % 6. Eksekusi Optimasi fmincon (SQP)
    [U_opt, ~, exitflag] = fmincon(cost_fun, U0, A_du, b_du, [], [], lb, ub, nonlcon, options);

    if exitflag <= 0
        u_applied = u_prev; % Fallback ke input sebelumnya jika optimasi gagal konvergen
    else
        u_applied = U_opt(1);
    end

    history_u_nmpc(k) = u_applied;

    % 7. Integrasi State dengan Metode Euler Menggunakan Model WyNDA
    s_nd = euler_step(s_nd, u_applied, theta);
    history_state_nmpc_nd(:, k+1) = s_nd;

    u_prev = u_applied;

    if mod(k, 80) == 0 || k == N_total-1
        fprintf('Progres: Step %3d / %3d (t = %5.1f s) | Rudder NMPC: %6.2f deg | Rudder Exp: %6.2f deg\n', ...
            k, N_total-1, time(k), rad2deg(u_applied), delta_exp_deg(k));
    end
end

history_u_nmpc(end) = history_u_nmpc(end-1);
waktu_komputasi_total = toc(timer_nmpc);

fprintf('SIMULASI CLOSED-LOOP NMPC SELESAI!\n');
fprintf('Total Waktu Komputasi: %.2f detik (Rata-rata: %.2f ms/step)\n\n', ...
    waktu_komputasi_total, (waktu_komputasi_total / (N_total-1)) * 1000);

%% 5. KONVERSI STATE KE DIMENSIONAL & EVALUASI VALIDASI
history_state_nmpc_dim = zeros(5, N_total);
for i = 1:N_total
    history_state_nmpc_dim(:, i) = nondimensional_to_dimensional(history_state_nmpc_nd(:, i), L, u_0);
end

v_nmpc       = history_state_nmpc_dim(1, :)';
r_nmpc       = history_state_nmpc_dim(2, :)';
x_nmpc       = history_state_nmpc_dim(3, :)';
y_nmpc       = history_state_nmpc_dim(4, :)';
psi_nmpc_rad = history_state_nmpc_dim(5, :)';

% Konversi Rudder NMPC ke Derajat
delta_nmpc_deg = rad2deg(history_u_nmpc);

% Perhitungan Galat Trajektori (Posisi & Heading)
err_x = x_ref_full - x_nmpc;
err_y = y_ref_full - y_nmpc;
err_2d = sqrt(err_x.^2 + err_y.^2);

rmse_x  = sqrt(mean(err_x.^2));
rmse_y  = sqrt(mean(err_y.^2));
rmse_2d = sqrt(mean(err_2d.^2));
mae_2d  = mean(err_2d);
max_2d  = max(err_2d);

err_psi_rad = atan2(sin(psi_ref_full - psi_nmpc_rad), cos(psi_ref_full - psi_nmpc_rad));
rmse_psi_deg = rad2deg(sqrt(mean(err_psi_rad.^2)));

% PERHITUNGAN METRIK VALIDASI SUDUT RUDDER (NMPC VS EKSPERIMEN)
err_rudder_deg = delta_exp_deg - delta_nmpc_deg;
rmse_rudder_deg = sqrt(mean(err_rudder_deg.^2));
mae_rudder_deg  = mean(abs(err_rudder_deg));
max_rudder_deg  = max(abs(err_rudder_deg));

% Korelasi Linear antara Kemudi NMPC dan Kemudi Eksperimen
corr_mat = corrcoef(delta_exp_deg, delta_nmpc_deg);
corr_rudder = corr_mat(1, 2);

% Evaluasi Kehalusan Perubahan Sudut Kemudi
du_nmpc_step_deg = [0; diff(delta_nmpc_deg)];
du_exp_step_deg  = [0; diff(delta_exp_deg)];

tot_variation_nmpc = sum(abs(du_nmpc_step_deg));
tot_variation_exp  = sum(abs(du_exp_step_deg));

fprintf('=========================================================================\n');
fprintf('             HASIL EVALUASI & VALIDASI KENDALI NMPC USV                 \n');
fprintf('=========================================================================\n');
fprintf('--- 1. EVALUASI TRACKING LINTASAN 2D ---\n');
fprintf('RMSE Posisi X (East)       : %.4f meter\n', rmse_x);
fprintf('RMSE Posisi Y (North)      : %.4f meter\n', rmse_y);
fprintf('RMSE Posisi 2D (Euclidean) : %.4f meter\n', rmse_2d);
fprintf('MAE Posisi 2D              : %.4f meter\n', mae_2d);
fprintf('Max Error Posisi 2D        : %.4f meter\n', max_2d);
fprintf('RMSE Yaw Heading           : %.2f derajat\n', rmse_psi_deg);
fprintf('-------------------------------------------------------------------------\n');
fprintf('--- 2. HASIL VALIDASI SUDUT RUDDER (KEMUDI) ---\n');
fprintf('Rentang Rudder NMPC        : [%.2f deg,  %.2f deg]\n', min(delta_nmpc_deg), max(delta_nmpc_deg));
fprintf('Rentang Rudder Eksperimen  : [%.2f deg,  %.2f deg]\n', min(delta_exp_deg), max(delta_exp_deg));
fprintf('RMSE Sudut Kemudi          : %.2f derajat\n', rmse_rudder_deg);
fprintf('MAE Sudut Kemudi           : %.2f derajat\n', mae_rudder_deg);
fprintf('Max Error Sudut Kemudi     : %.2f derajat\n', max_rudder_deg);
fprintf('Koefisien Korelasi Kemudi  : %.4f\n', corr_rudder);
fprintf('-------------------------------------------------------------------------\n');
fprintf('--- 3. EVALUASI PERUBAHAN SUDUT RUDDER (RATE & SMOOTHNESS) ---\n');
fprintf('Max Perubahan Rudder NMPC  : %.2f deg/step (Batas: +/- %.1f deg/step)\n', ...
    max(abs(du_nmpc_step_deg)), du_max_deg);
fprintf('Max Perubahan Rudder Exp   : %.2f deg/step\n', max(abs(du_exp_step_deg)));
fprintf('Total Variasi Kemudi NMPC  : %.2f deg\n', tot_variation_nmpc);
fprintf('Total Variasi Kemudi Exp   : %.2f deg\n', tot_variation_exp);
if tot_variation_exp > 0
    fprintf('Indeks Kehalusan NMPC      : %.2f%% lebih mulus dibandingkan manual/riil\n', ...
        (1 - tot_variation_nmpc / tot_variation_exp) * 100);
end
fprintf('=========================================================================\n\n');

%% 6. VISUALISASI GRAFIK VALIDASI NMPC (DOCKED DALAM 1 FRAME TAB BERDERET)
set(0, 'DefaultFigureWindowStyle', 'docked');

% FIGURE 1: Validasi Lintasan 2D (Target Excel vs NMPC Closed-Loop)
figure('Name', '1. Validasi Lintasan 2D NMPC', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(x_ref_full, y_ref_full, 'b-', 'LineWidth', 2.2); hold on;
plot(x_nmpc, y_nmpc, 'r--', 'LineWidth', 2.2);
plot(x_ref_full(1), y_ref_full(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
plot(x_ref_full(end), y_ref_full(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Lintasan 2D: NMPC Closed-Loop vs Data Referensi [RMSE = %.3f m]', rmse_2d), ...
    'FontSize', 12, 'FontWeight', 'bold');
legend('Target Referensi (Excel ENU)', 'Respon NMPC Closed-Loop (Model WyNDA)', ...
       'Titik Awal (Start)', 'Titik Akhir (End)', 'Location', 'best');

% FIGURE 2: VALIDASI SUDUT RUDDER (NMPC vs DATA EKSPERIMEN)
figure('Name', '2. Validasi Sudut Kemudi Rudder', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, delta_nmpc_deg, 'r-', 'LineWidth', 1.8); hold on;
plot(time, delta_exp_deg, 'b--', 'LineWidth', 1.4);
yline(u_limit_deg, 'k--', sprintf('+%.1f deg (Upper Limit)', u_limit_deg), 'LineWidth', 1.2);
yline(-u_limit_deg, 'k--', sprintf('-%.1f deg (Lower Limit)', u_limit_deg), 'LineWidth', 1.2);
grid on;
ylabel('Sudut Rudder \delta [derajat]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Sudut Rudder: NMPC vs Data Eksperimen (RMSE = %.2f deg, Korelasi = %.2f)', ...
    rmse_rudder_deg, corr_rudder), 'FontSize', 12, 'FontWeight', 'bold');
legend('Sinyal Kemudi Optimal NMPC (\delta_{nmpc})', 'Sinyal Kemudi Eksperimen (\delta_{exp})', ...
       'Batas Fisik Rudder (\pm 45^\circ)', 'Location', 'best');

subplot(2,1,2);
plot(time, err_rudder_deg, 'm-', 'LineWidth', 1.5); hold on;
yline(0, 'k-', 'LineWidth', 1.0);
yline(mae_rudder_deg, 'b--', sprintf('MAE = %.2f deg', mae_rudder_deg));
yline(-mae_rudder_deg, 'b--');
grid on;
xlabel('Waktu [detik]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Selisih Kemudi [derajat]', 'FontSize', 11, 'FontWeight', 'bold');
title('Selisih Sudut Kemudi (\delta_{exp} - \delta_{nmpc})');
legend('Galat Sudut Kemudi', 'Garis Nol', 'Batas Rata-rata MAE', 'Location', 'best');

% FIGURE 3: Perubahan Sudut Kemudi Antar-Langkah (Delta Rudder Rate per Step)
figure('Name', '3. Perubahan Sudut Rudder Antar-Step', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, du_nmpc_step_deg, 'r-', 'LineWidth', 1.6); hold on;
plot(time, du_exp_step_deg, 'b:', 'LineWidth', 1.2);
yline(du_max_deg, 'k--', sprintf('+%.1f deg/step (Max Rate Limit)', du_max_deg), 'LineWidth', 1.2);
yline(-du_max_deg, 'k--', sprintf('-%.1f deg/step (Min Rate Limit)', du_max_deg), 'LineWidth', 1.2);
grid on;
xlabel('Waktu [detik]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('\Delta\delta per Step [derajat]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Laju Perubahan Sudut Kemudi per Step (Batas Maksimal \\pm %.1f deg/step)', du_max_deg), ...
    'FontSize', 12, 'FontWeight', 'bold');
legend('\Delta\delta NMPC', '\Delta\delta Eksperimen', 'Batas Perubahan (\pm 30 deg/step)', 'Location', 'best');

% FIGURE 4: Respon Tracking Posisi X dan Y terhadap Waktu
figure('Name', '4. Tracking Posisi X dan Y', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, x_ref_full, 'b-', 'LineWidth', 1.8); hold on;
plot(time, x_nmpc, 'r--', 'LineWidth', 1.8);
grid on;
ylabel('Posisi East X [m]', 'FontSize', 10, 'FontWeight', 'bold');
title(sprintf('Tracking Posisi X (RMSE = %.3f m)', rmse_x), 'FontSize', 11, 'FontWeight', 'bold');
legend('Target Referensi', 'Respon NMPC', 'Location', 'best');

subplot(2,1,2);
plot(time, y_ref_full, 'b-', 'LineWidth', 1.8); hold on;
plot(time, y_nmpc, 'r--', 'LineWidth', 1.8);
grid on;
xlabel('Waktu [detik]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Posisi North Y [m]', 'FontSize', 10, 'FontWeight', 'bold');
title(sprintf('Tracking Posisi Y (RMSE = %.3f m)', rmse_y), 'FontSize', 11, 'FontWeight', 'bold');
legend('Target Referensi', 'Respon NMPC', 'Location', 'best');

% FIGURE 5: Respon Yaw Heading & Yaw Rate
figure('Name', '5. Respon Yaw Heading & Yaw Rate', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, rad2deg(psi_ref_full), 'b-', 'LineWidth', 1.8); hold on;
plot(time, rad2deg(psi_nmpc_rad), 'r--', 'LineWidth', 1.8);
grid on;
ylabel('Yaw Heading \psi [derajat]', 'FontSize', 10, 'FontWeight', 'bold');
title(sprintf('Tracking Sudut Yaw Heading Kalibrasi (RMSE = %.2f deg)', rmse_psi_deg), ...
    'FontSize', 11, 'FontWeight', 'bold');
legend('Target Referensi', 'Respon NMPC', 'Location', 'best');

subplot(2,1,2);
plot(time, rad2deg(r_nmpc), 'm-', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(r_exp), 'c:', 'LineWidth', 1.2);
yline(r_limit_deg, 'r--', sprintf('+%.1f deg/s (Limit)', r_limit_deg));
yline(-r_limit_deg, 'r--', sprintf('-%.1f deg/s (Limit)', r_limit_deg));
grid on;
xlabel('Waktu [detik]', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Yaw Rate r [deg/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('Kecepatan Sudut Putar Kapal (Yaw Rate r)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Yaw Rate NMPC', 'Yaw Rate Eksperimen', 'Batas Operasional', 'Location', 'best');

% FIGURE 6: Grafik Eror Jarak Tracking 2D (Euclidean Distance Error)
figure('Name', '6. Eror Jarak Tracking 2D NMPC', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, err_2d, 'r-', 'LineWidth', 1.6); hold on;
yline(rmse_2d, 'b--', sprintf('RMSE 2D = %.3f m', rmse_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(mae_2d, 'g--', sprintf('MAE 2D = %.3f m', mae_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
grid on;
xlabel('Waktu [detik]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Galat Jarak [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title('Galat Jarak Tracking Posisi 2D (Euclidean Distance Error) NMPC', 'FontSize', 12, 'FontWeight', 'bold');
legend('Eror Jarak Tracking 2D', 'Nilai RMSE 2D', 'Nilai MAE 2D', 'Location', 'best');

fprintf('✔ Validasi Closed-Loop NMPC Berhasil Selesai! Semua figure tergabung rapi dalam 1 frame docked.\n');

%% =========================================================================
% FUNGSI-FUNGSI HELPER MODEL WYNDA, KENDALI NMPC & INTEGRATOR EULER
% =========================================================================

%% 1. FUNGSI INTEGRATOR EULER DENGAN MODEL WYNDA
function s_next = euler_step(s, u, theta)
    % s = [v; r; x; y; psi] (nondimensional)
    % u = delta (radian)
    Phi = compute_basis_phi_11(s, u);
    s_next = s + Phi * theta;
end

%% 2. FUNGSI BASIS PHI MODEL WYNDA 11 PARAMETER (Persamaan 4.27 - 4.31)
function Phi = compute_basis_phi_11(s, delta)
    v   = s(1);
    r   = s(2);
    psi = s(5);
    
    Phi = [
        % ---- Eq (4.27): v_dot ---- (theta_1 s.d. theta_3)
        v,  r,  delta, zeros(1,8);

        % ---- Eq (4.28): r_dot ---- (theta_4 s.d. theta_6)
        zeros(1,3), v,  r,  delta, zeros(1,5);

        % ---- Eq (4.29): x_dot ---- (theta_7 s.d. theta_8)
        zeros(1,6), cos(psi), -v*sin(psi), zeros(1,3);

        % ---- Eq (4.30): y_dot ---- (theta_9 s.d. theta_10)
        zeros(1,8), sin(psi),  v*cos(psi), zeros(1,1);

        % ---- Eq (4.31): psi_dot ---- (theta_11)
        zeros(1,10), r
    ];
end

%% 3. FUNGSI BIAYA (COST FUNCTION) NMPC
function J = mpc_cost(U, s0, x_ref_seq, y_ref_seq, psi_ref_seq, theta, Q, R)
    N = length(U);
    s = s0;
    J = 0;
    
    for i = 1:N
        u = U(i);
        s = euler_step(s, u, theta);
        
        % Kalibrasi selisih heading menghindari pembacaan jump wrap-around [-pi, pi]
        psi_err = atan2(sin(s(5) - psi_ref_seq(i)), cos(s(5) - psi_ref_seq(i)));
        
        err = [s(3) - x_ref_seq(i);
               s(4) - y_ref_seq(i);
               psi_err];

        J = J + err' * Q * err + R * (u^2);
    end
end

%% 4. FUNGSI KENDALA NONLINEAR STATE (YAW RATE CONSTRAINT)
function [c, ceq] = state_constraints(U, s0, theta, r_limit_nd)
    N = length(U);
    s = s0;
    c = zeros(2*N, 1);
    idx = 1;
    for i = 1:N
        s = euler_step(s, U(i), theta);
        r = s(2);
        c(idx)   =  r - r_limit_nd;
        c(idx+1) = -r_limit_nd - r;
        idx = idx + 2;
    end
    ceq = [];
end

%% 5. FUNGSI GENERATOR KENDALA PERUBAHAN SUDUT KEMUDI (du_max)
function [A, b] = du_constraints(N, u_prev, du_max)
    A = zeros(2*N, N);
    b = zeros(2*N, 1);
    
    % Batas atas: u(1) - u_prev <= du_max
    A(1,1) = 1;   b(1) = u_prev + du_max;
    for i = 2:N
        A(i, i-1) = -1;   A(i, i) = 1;
        b(i) = du_max;
    end
    
    % Batas bawah: u_prev - u(1) <= du_max  ==> -u(1) <= du_max - u_prev
    A(N+1,1) = -1;   b(N+1) = du_max - u_prev;
    for i = 2:N
        A(N+i, i-1) = 1;   A(N+i, i) = -1;
        b(N+i) = du_max;
    end
end

%% 6. FUNGSI KONVERSI DIMENSIONAL DAN NONDIMENSIONAL
function x_nd = dimensional_to_nondimensional(x, L, u0)
    % x = [v; r; x_pos; y_pos; psi]
    x_nd = [x(1)/u0; x(2)*L/u0; x(3)/L; x(4)/L; x(5)];
end

function x_dim = nondimensional_to_dimensional(x, L, u0)
    % x_nd = [v'; r'; x'; y'; psi']
    x_dim = [x(1)*u0; x(2)*u0/L; x(3)*L; x(4)*L; x(5)];
end
