%% ============================================================================
%            VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA (11 PARAMETER)
%                         SPESIFIKASI SKALA FROUDE (1:100)
% =============================================================================
% * Validasi open-loop model kapal berbasis parameter identifikasi WyNDA 11 basis
%   Persamaan (4.27) s.d. (4.31).
% * Input model: Sudut kemudi rudder delta (radian)
% * Evaluasi: Perbandingan lintasan 2D (x, y), heading (psi), dan perhitungan RMSE
% =============================================================================

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('=========================================================================\n');
fprintf('    SIMULASI VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA (11 PARAMETER) \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%  1. READ & PREPROCESS DATA EKSPERIMEN
% =========================================================================

dataFile = 'hasil_preprocessing_ENU.xlsx';

fprintf('1. Membaca data eksperimen dari: %s\n', dataFile);
data = readtable(dataFile);
N = height(data);

% Ekstraksi State Eksperimen
x_exp         = data.x_ENU;
y_exp         = data.y_ENU;
psi_exp_rad   = data.psi;
delta_rad_exp = data.delta;
delta_deg_exp = rad2deg(delta_rad_exp);

% Waktu eksperimen (detik)
if ismember('time', data.Properties.VariableNames)
    time = data.time;
else
    time = (0:N-1)' * 0.1;
end

% Kecepatan surge nominal u_0 dari data
if ismember('u_exp', data.Properties.VariableNames)
    u_0_exp = data.u_exp(1);
else
    u_0_exp = 0.6114; % Nilai nominal default [m/s]
end

fprintf('   Total sampel data : %d\n', N);
fprintf('   Durasi eksperimen : %.2f detik\n', time(end));
fprintf('   Kecepatan surge nominal u_0 : %.4f m/s\n\n', u_0_exp);

%% =========================================================================
%  2. SPESIFIKASI KAPAL & PARAMETER THETA WYNDA (11 BASIS FUNGSI)
% =========================================================================

L = 1.0107;          % Panjang model kapal [meter]
u_0 = u_0_exp;       % Kecepatan surge referensi [m/s]
u_0_nd = 1.0;        % Kecepatan surge nondimensional

% Vektor Parameter Theta Hasil Identifikasi WyNDA 11 Basis Fungsi:
% Eq (4.27) -> v_dot   = theta_1*v + theta_2*r + theta_3*delta
% Eq (4.28) -> r_dot   = theta_4*v + theta_5*r + theta_6*delta
% Eq (4.29) -> x_dot   = theta_7*u0*cos(psi) - theta_8*v*sin(psi)
% Eq (4.30) -> y_dot   = theta_9*u0*sin(psi) + theta_10*v*cos(psi)
% Eq (4.31) -> psi_dot = theta_11*r

theta = [
   -4.5850e-01;  % theta_1  : v pada v_dot
   -6.4487e-02;  % theta_2  : r pada v_dot
    2.5817e-02;  % theta_3  : delta pada v_dot
    7.7122e-04;  % theta_4  : v pada r_dot
    2.7250e-03;  % theta_5  : r pada r_dot
   -1.1909e-02;  % theta_6  : delta pada r_dot
   -5.7266e-02;  % theta_7  : u0*cos(psi)
   -7.5794e-02;  % theta_8  : v*sin(psi)
   -9.0402e-03;  % theta_9  : u0*sin(psi)
   -1.6046e-02;  % theta_10 : v*cos(psi)
    6.0598e-02   % theta_11 : r pada psi_dot
];

fprintf('2. Parameter Model WyNDA Terdefinisi (11 Parameter Theta)\n');
fprintf('   Panjang Kapal L = %.4f m | u_0 = %.4f m/s\n\n', L, u_0);

%% =========================================================================
%  3. EKSEKUSI SIMULASI MODEL WYNDA (DISCRETE STATE TRANSITION)
% =========================================================================

history_state_sim_nd = zeros(5, N);

% Inisialisasi kondisi awal simulasi t=0
s0_dim = [0; 0; x_exp(1); y_exp(1); psi_exp_rad(1)];
s0_nd  = dimensional_to_nondimensional(s0_dim, L, u_0);
history_state_sim_nd(:, 1) = s0_nd;

s_nd = s0_nd;

fprintf('3. Menjalankan simulasi open-loop model WyNDA (11 Parameter)...\n');
for k = 1:N-1
    delta_k = delta_rad_exp(k);
    
    % Basis fungsi 11 parameter
    Phi_k = compute_basis_phi_11(s_nd, delta_k);
    
    % Update transisi diskret: s(k+1) = s(k) + Phi * theta
    delta_s = Phi_k * theta;
    s_nd = s_nd + delta_s;
    
    history_state_sim_nd(:, k+1) = s_nd;
end

% Konversi seluruh state hasil simulasi ke satuan dimensional
history_state_sim_dim = zeros(5, N);
for i = 1:N
    history_state_sim_dim(:, i) = nondimensional_to_dimensional(history_state_sim_nd(:, i), L, u_0);
end

v_sim       = history_state_sim_dim(1, :)';
r_sim       = history_state_sim_dim(2, :)';
x_sim       = history_state_sim_dim(3, :)';
y_sim       = history_state_sim_dim(4, :)';
psi_sim_rad = history_state_sim_dim(5, :)';

%% =========================================================================
%  4. EVALUASI DAN PERHITUNGAN RMSE
% =========================================================================

err_x = x_exp - x_sim;
err_y = y_exp - y_sim;
err_pos_2d = sqrt(err_x.^2 + err_y.^2);

rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
rmse_pos_2d = sqrt(mean(err_pos_2d.^2));

err_psi_rad = psi_exp_rad - psi_sim_rad;
rmse_psi_rad = sqrt(mean(err_psi_rad.^2));
rmse_psi_deg = rad2deg(rmse_psi_rad);

fprintf('\n=======================================================\n');
fprintf('     HASIL VALIDASI MODEL WYNDA DENGAN DATA EKSPERIMEN   \n');
fprintf('=======================================================\n');
fprintf('RMSE Posisi X (East) : %.4f meter\n', rmse_x);
fprintf('RMSE Posisi Y (North): %.4f meter\n', rmse_y);
fprintf('RMSE Posisi 2D       : %.4f meter\n', rmse_pos_2d);
fprintf('RMSE Yaw Heading     : %.4f rad (%.2f derajat)\n', rmse_psi_rad, rmse_psi_deg);
fprintf('=======================================================\n\n');

%% =========================================================================
%  5. VISUALISASI GRAFIK HASIL VALIDASI
% =========================================================================

% FIGURE 1: Perbandingan Lintasan 2D
figure('Name', '1. Validasi Lintasan 2D Kapal WyNDA', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(x_exp, y_exp, 'b-', 'LineWidth', 2.0); hold on;
plot(x_sim, y_sim, 'r--', 'LineWidth', 2.0);
plot(x_exp(1), y_exp(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2); % Start
plot(x_exp(end), y_exp(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2); % End
grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Lintasan 2D (Model WyNDA 11 Parameter vs Eksperimen) [RMSE = %.3f m]', rmse_pos_2d), 'FontSize', 12, 'FontWeight', 'bold');
legend('Data Eksperimen', 'Simulasi Model WyNDA', 'Titik Awal (Start)', 'Titik Akhir', 'Location', 'best');

% FIGURE 2: Perbandingan Posisi X dan Y terhadap Waktu
figure('Name', '2. Respon Posisi X dan Y', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, x_exp, 'b-', 'LineWidth', 1.8); hold on;
plot(time, x_sim, 'r--', 'LineWidth', 1.8);
grid on; ylabel('Posisi X (East) [m]', 'FontWeight', 'bold');
title(sprintf('Respon Posisi X (RMSE = %.3f m)', rmse_x));
legend('Eksperimen', 'Model WyNDA', 'Location', 'best');

subplot(2,1,2);
plot(time, y_exp, 'b-', 'LineWidth', 1.8); hold on;
plot(time, y_sim, 'r--', 'LineWidth', 1.8);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Posisi Y (North) [m]', 'FontWeight', 'bold');
title(sprintf('Respon Posisi Y (RMSE = %.3f m)', rmse_y));
legend('Eksperimen', 'Model WyNDA', 'Location', 'best');

% FIGURE 3: Perbandingan Sudut Yaw (Heading) & Yaw Rate
figure('Name', '3. Respon Yaw Heading & Yaw Rate', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, rad2deg(psi_exp_rad), 'b-', 'LineWidth', 1.8); hold on;
plot(time, rad2deg(psi_sim_rad), 'r--', 'LineWidth', 1.8);
grid on; ylabel('Yaw Angle [derajat]', 'FontWeight', 'bold');
title(sprintf('Perbandingan Sudut Yaw Heading (RMSE = %.2f deg)', rmse_psi_deg));
legend('Eksperimen', 'Model WyNDA', 'Location', 'best');

subplot(2,1,2);
plot(time, rad2deg(r_sim), 'm-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Yaw Rate [deg/s]', 'FontWeight', 'bold');
title('Kecepatan Sudut Putar Kapal (Yaw Rate) Simulasi WyNDA');

% FIGURE 4: Input Sinyal Kontrol Kemudi (Servo Rudder)
figure('Name', '4. Input Sinyal Kontrol Kemudi', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, delta_deg_exp, 'k-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Sudut Rudder [derajat]', 'FontWeight', 'bold');
title('Input Sinyal Kemudi Eksperimen (\delta)');

% FIGURE 5: Grafik Eror Jarak Posisi 2D (Euclidean Error)
figure('Name', '5. Eror Jarak Posisi 2D', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, err_pos_2d, 'r-', 'LineWidth', 1.5); hold on;
yline(rmse_pos_2d, 'b--', sprintf('RMSE 2D = %.3f m', rmse_pos_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Galat Jarak [meter]', 'FontWeight', 'bold');
title('Galat Jarak Posisi 2D (Euclidean Distance Error) Model WyNDA');
legend('Eror Jarak per Detik', 'Nilai RMSE 2D', 'Location', 'best');

fprintf('✔ Validasi Model WyNDA Selesai!\n');

%% =========================================================================
%  6. FUNGSI BASIS PHI MODEL WYNDA 11 PARAMETER (Persamaan 4.27 - 4.31)
% =========================================================================

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

%% =========================================================================
%  7. FUNGSI KONVERSI DIMENSIONAL DAN NONDIMENSIONAL
% =========================================================================

function x_nd = dimensional_to_nondimensional(x, L, u0)
    x_nd = [x(1)/u0; x(2)*L/u0; x(3)/L; x(4)/L; x(5)];
end

function x_dim = nondimensional_to_dimensional(x, L, u0)
    x_dim = [x(1)*u0; x(2)*u0/L; x(3)*L; x(4)*L; x(5)];
end
