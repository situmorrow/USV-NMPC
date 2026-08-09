%% =========================================================================
% VALIDASI MODEL MATEMATIKA KAPAL (OPEN-LOOP SIMULATION) - REVISED
% Berdasarkan Dokumen: 09_metode_validasi_model_dan_nmpc_dari_data_rc.md
% Folder: d:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\validasi_model_dan_nmpc\
% =========================================================================

clear; clc; close all;

fprintf('=======================================================\n');
fprintf('  SIMULASI VALIDASI MODEL MATEMATIKA KAPAL (OPEN-LOOP) \n');
fprintf('=======================================================\n\n');

%% 1. KONTROL PILIHAN SKALA KAPAL (USV VS TANKER)
% =========================================================================
% Tipe Kapal:
%   'USV_RC'  : Menggunakan dimensi asli USV/Boat RC (L ~ 1.2 m, u0 ~ 1.08 m/s)
%   'TANKER'  : Menggunakan parameter NMPC_Biasa.m (L = 101 m, u0 = 15.4 m/s)
MODE_KAPAL = 'USV_RC'; 

% Rudder Gain Sign (-1 atau +1 untuk penyesuaian arah kemudi servo)
RUDDER_GAIN_SIGN = 1; 

%% 2. READ & PREPROCESS DATA EKSPERIMEN CSV
csvPath = 'turning_otomatis.csv';
if ~exist(csvPath, 'file')
    error('File %s tidak ditemukan! Pastikan script berada di folder validasi_model_dan_nmpc.', csvPath);
end

fprintf('Membaca data eksperimen dari: %s\n', csvPath);
data = readtable(csvPath);

if ~ismember('x_enu_flat', data.Properties.VariableNames)
    error('Kolom ENU belum ada. Jalankan script konversi_latlon_ecef_enu.m terlebih dahulu!');
end

timestamp = data.timestamp;
time = timestamp - timestamp(1); % Waktu relatif (detik)
N = height(data);

% Posisi eksperimen ENU (Meter)
x_exp = data.x_enu_flat;
y_exp = data.y_enu_flat;

% Input servo kemudi (Calc_deg_servo_1) dalam derajat -> konversi ke radian
delta_deg_exp = RUDDER_GAIN_SIGN * data.Calc_deg_servo_1;
delta_rad_exp = deg2rad(delta_deg_exp);

% Yaw eksperimen (Compass Heading) -> konversi ke Sudut ENU Kartesian (Radian)
yaw_compass_deg = data.yaw;
psi_enu_deg = 90.0 - yaw_compass_deg;
psi_exp_rad = unwrap(deg2rad(psi_enu_deg));

% Kecepatan surge eksperimen dari GPS (speedMps)
speed_exp = data.speedMps;
u_0_exp = mean(speed_exp);
if u_0_exp < 0.05, u_0_exp = 1.0; end

fprintf('Data berhasil dimuat. Total sampel: %d | Durasi: %.2f detik\n', N, time(end));
fprintf('Kecepatan rata-rata eksperimen GPS (u_0): %.4f m/s\n\n', u_0_exp);

%% 3. SETUP PARAMETER KAPAL BERDASARKAN MODE
if strcmpi(MODE_KAPAL, 'USV_RC')
    % PARAMETER UNTUK PROTOTIPE KAPAL USV RC (SKALA REAL RIL)
    L = 1.2;            % Panjang USV (m)
    B = 0.35;           % Lebar USV (m)
    T = 0.10;           % Draft USV (m)
    m = 8.5;            % Massa USV (kg)
    u_0 = u_0_exp;      % Mengikuti kecepatan rata-rata eksperimen (~1.08 m/s)
    C_B = 0.65;         % Koefisien Blok
    x_G = 0.05;         % Pusat massa x (m)
    rho = 1000;         % Massa jenis air tawar/pantai (kg/m^3)
    fprintf('--> Menggunakan Mode Kapal: USV RC (L = %.2f m, u0 = %.2f m/s)\n', L, u_0);
else
    % PARAMETER UNTUK KAPAL TANKER KAMPUS (NMPC_Biasa.m)
    L = 101.07;         % Panjang Kapal (m)
    B = 14;             % Lebar Kapal (m)
    T = 3.7;            % Draft Kapal (m)
    m = 2423*1e3;       % Massa Kapal (kg)
    u_0 = 15.4;         % Kecepatan nominal model (m/s)
    C_B = 0.65;         
    x_G = 5.25;         
    rho = 1024;         
    fprintf('--> Menggunakan Mode Kapal: TANKER LARGE SHIP (L = %.2f m, u0 = %.2f m/s)\n', L, u_0);
end

% Koefisien Hidrodinamika Nondimensional (Fossen / Abkowitz)
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

% Matriks Massa & Sistem Dinamika
M_mat = [m_nd - Y_v_dot , m_nd*x_G_nd - Y_r_dot ;
         m_nd*x_G_nd - N_v_dot , I_z_nd - N_r_dot ];

a11 = ((I_z_nd - N_r_dot)*Y_v - (m_nd*x_G_nd - Y_r_dot)*N_v) / det(M_mat);
a12 = ((I_z_nd - N_r_dot)*(Y_r - m_nd*u_0_nd) - (m_nd*x_G_nd - Y_r_dot)*(N_r - m_nd*x_G_nd*u_0_nd)) / det(M_mat);
a21 = ((m_nd - Y_v_dot)*N_v - (m_nd*x_G_nd - N_v_dot)*Y_v) / det(M_mat);
a22 = ((m_nd - Y_v_dot)*(N_r - m_nd*x_G_nd*u_0_nd) - (m_nd*x_G_nd - N_v_dot)*(Y_r - m_nd*u_0_nd)) / det(M_mat);

A_sys = [a11 , a12 ;
         a21 , a22 ];
B_sys = [0.01 ;
         1];

%% 4. EKSEKUSI SIMULASI OPEN-LOOP (RK4 INTEGRATOR)
history_state_sim_nd = zeros(5, N);

% Inisialisasi kondisi awal simulasi sesuai dengan data eksperimen t=0
s0_dim = [0; 0; x_exp(1); y_exp(1); psi_exp_rad(1)];
s0_nd = dimensional_to_nondimensional(s0_dim, L, u_0);
history_state_sim_nd(:, 1) = s0_nd;

s_nd = s0_nd;

fprintf('Menjalankan integrasi Runge-Kutta 4th Order (RK4) Open-Loop...\n');
for k = 1:N-1
    dt = time(k+1) - time(k);
    if dt <= 0, dt = 0.1; end
    
    dt_nd = dt * u_0 / L;
    u_applied = delta_rad_exp(k); % Input servo kemudi dari data RC
    
    % Integrasi RK4
    s_nd = rk4_step(@(s, u) ship_dynamics(s, u, A_sys, B_sys, u_0_nd), s_nd, u_applied, dt_nd);
    history_state_sim_nd(:, k+1) = s_nd;
end

% Konversi seluruh state hasil simulasi dari nondimensional ke dimensional
history_state_sim_dim = zeros(5, N);
for i = 1:N
    history_state_sim_dim(:, i) = nondimensional_to_dimensional(history_state_sim_nd(:, i), L, u_0);
end

v_sim = history_state_sim_dim(1, :)';
r_sim = history_state_sim_dim(2, :)';
x_sim = history_state_sim_dim(3, :)';
y_sim = history_state_sim_dim(4, :)';
psi_sim_rad = history_state_sim_dim(5, :)';

%% 5. PERHITUNGAN METRIK EVALUASI GALAT (RMSE, MAE, R^2)
err_x = x_exp - x_sim;
err_y = y_exp - y_sim;
err_pos_2d = sqrt(err_x.^2 + err_y.^2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
rmse_pos_2d = sqrt(mean(err_pos_2d.^2));
mae_pos_2d = mean(err_pos_2d);

err_psi_rad = psi_exp_rad - psi_sim_rad;
rmse_psi_rad = sqrt(mean(err_psi_rad.^2));
rmse_psi_deg = rad2deg(rmse_psi_rad);

R2_x = 1 - (sum(err_x.^2) / sum((x_exp - mean(x_exp)).^2));
R2_y = 1 - (sum(err_y.^2) / sum((y_exp - mean(y_exp)).^2));

fprintf('\n=======================================================\n');
fprintf('         HASIL EVALUASI METRIK GALAT MODEL             \n');
fprintf('=======================================================\n');
fprintf('Mode Kapal          : %s\n', MODE_KAPAL);
fprintf('Panjang Kapal (L)   : %.2f meter\n', L);
fprintf('Kecepatan (u_0)     : %.2f m/s\n', u_0);
fprintf('-------------------------------------------------------\n');
fprintf('RMSE Posisi X       : %.4f meter\n', rmse_x);
fprintf('RMSE Posisi Y       : %.4f meter\n', rmse_y);
fprintf('RMSE Posisi 2D      : %.4f meter\n', rmse_pos_2d);
fprintf('MAE Posisi 2D       : %.4f meter\n', mae_pos_2d);
fprintf('RMSE Yaw Heading    : %.4f rad (%.2f derajat)\n', rmse_psi_rad, rmse_psi_deg);
fprintf('R^2 Posisi X        : %.4f\n', R2_x);
fprintf('R^2 Posisi Y        : %.4f\n', R2_y);
fprintf('=======================================================\n\n');

%% 6. VISUALISASI GRAFIK HASIL VALIDASI
% FIGURE 1: Perbandingan Lintasan 2D (Eksperimen vs Simulasi Model)
figure('Name', '1. Validasi Lintasan 2D Kapal', 'NumberTitle', 'off', 'Position', [100, 100, 850, 650]);
plot(x_exp, y_exp, 'b-', 'LineWidth', 2.0); hold on;
plot(x_sim, y_sim, 'r--', 'LineWidth', 2.0);
plot(x_exp(1), y_exp(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot(x_exp(end), y_exp(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % End
grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Open-Loop: Lintasan Eksperimen RC vs Simulasi Model (Mode: %s)', MODE_KAPAL), 'FontSize', 12, 'FontWeight', 'bold');
legend('Data Eksperimen (RC)', 'Simulasi Model Matematika', 'Titik Awal (Start)', 'Titik Akhir', 'Location', 'best');

% FIGURE 2: Perbandingan Posisi X dan Y terhadap Waktu
figure('Name', '2. Respon Posisi X dan Y', 'NumberTitle', 'off', 'Position', [150, 150, 900, 600]);
subplot(2,1,1);
plot(time, x_exp, 'b-', 'LineWidth', 1.8); hold on;
plot(time, x_sim, 'r--', 'LineWidth', 1.8);
grid on; ylabel('Posisi X (East) [m]', 'FontWeight', 'bold');
title(sprintf('Respon Posisi X (RMSE = %.3f m, R^2 = %.3f)', rmse_x, R2_x));
legend('Eksperimen', 'Simulasi Model');

subplot(2,1,2);
plot(time, y_exp, 'b-', 'LineWidth', 1.8); hold on;
plot(time, y_sim, 'r--', 'LineWidth', 1.8);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Posisi Y (North) [m]', 'FontWeight', 'bold');
title(sprintf('Respon Posisi Y (RMSE = %.3f m, R^2 = %.3f)', rmse_y, R2_y));
legend('Eksperimen', 'Simulasi Model');

% FIGURE 3: Perbandingan Sudut Yaw (Heading) & Yaw Rate
figure('Name', '3. Respon Yaw Heading & Yaw Rate', 'NumberTitle', 'off', 'Position', [200, 200, 900, 600]);
subplot(2,1,1);
plot(time, rad2deg(psi_exp_rad), 'b-', 'LineWidth', 1.8); hold on;
plot(time, rad2deg(psi_sim_rad), 'r--', 'LineWidth', 1.8);
grid on; ylabel('Yaw Angle [derajat]', 'FontWeight', 'bold');
title(sprintf('Perbandingan Sudut Yaw Heading \\psi (RMSE = %.2f deg)', rmse_psi_deg));
legend('Eksperimen (IMU)', 'Simulasi Model');

subplot(2,1,2);
plot(time, rad2deg(r_sim), 'm-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Yaw Rate r [deg/s]', 'FontWeight', 'bold');
title('Kecepatan Sudut Putar Kapal (Yaw Rate r) Hasil Simulasi Model');

% FIGURE 4: Input Sinyal Kontrol Kemudi (Servo Rudder)
figure('Name', '4. Input Sinyal Kontrol RC', 'NumberTitle', 'off', 'Position', [250, 250, 850, 450]);
plot(time, delta_deg_exp, 'k-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Sudut Rudder \\delta [derajat]', 'FontWeight', 'bold');
title('Input Sinyal Kemudi RC (Calc\\_deg\\_servo\\_1) yang Diaplikasikan ke Model');

% FIGURE 5: Grafik Eror Jarak Posisi 2D (Euclidean Error)
figure('Name', '5. Eror Jarak Posisi 2D', 'NumberTitle', 'off', 'Position', [300, 300, 850, 450]);
plot(time, err_pos_2d, 'r-', 'LineWidth', 1.5); hold on;
yline(rmse_pos_2d, 'b--', sprintf('RMSE 2D = %.3f m', rmse_pos_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
yline(mae_pos_2d, 'g--', sprintf('MAE 2D = %.3f m', mae_pos_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Galat Jarak [meter]', 'FontWeight', 'bold');
title('Galat Jarak Posisi 2D (Euclidean Distance Error) Open-Loop Simulation');
legend('Eror Jarak per Detik', 'Nilai RMSE 2D', 'Nilai MAE 2D');

fprintf('Validasi Selesai! Semua grafik hasil simulasi berhasil ditampilkan.\n');

%% =========================================================================
% FUNGSI INTEGRATOR RK4 & DINAMIKA KAPAL
% =========================================================================

function x_next = rk4_step(f, x, u, dt)
    k1 = f(x, u);
    k2 = f(x + 0.5 * dt * k1, u);
    k3 = f(x + 0.5 * dt * k2, u);
    k4 = f(x + dt * k3, u);
    x_next = x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
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

function x_nd = dimensional_to_nondimensional(x, L, u0)
    x_nd = [x(1)/u0; x(2)*L/u0; x(3)/L; x(4)/L; x(5)];
end

function x_dim = nondimensional_to_dimensional(x, L, u0)
    x_dim = [x(1)*u0; x(2)*u0/L; x(3)*L; x(4)*L; x(5)];
end
