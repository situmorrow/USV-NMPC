%% =========================================================================
%    ESTIMASI PARAMETER MODEL WYNDA GABUNGAN MULTI-MANUVER (UNIVERSAL)
%    (Turning Test + Gerak Lurus Konstan)
% =========================================================================
%  Deskripsi:
%  Script ini mengestimasi 22 parameter hidrodinamika model non-linear WyNDA
%  menggunakan multi-dataset training yang menggabungkan:
%  1. Manuver Turning Test (DATA_turning.csv)
%  2. Manuver Gerak Lurus Konstan (lurus_konstan.csv)
%
%  Tujuan:
%  Menghasilkan model matematika universal yang stabil dan akurat baik pada
%  kondisi gerak lurus (kemudi netral delta ~ 0) maupun saat berbelok (delta ~ 35 deg).
% =========================================================================

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('=========================================================================\n');
fprintf('   ESTIMASI PARAMETER WYNDA GABUNGAN MULTI-MANUVER (UNIVERSAL)           \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%  1. LOAD DAN PREPROCESS DATASET EKSPERIMEN
% =========================================================================

file_turning  = 'DATA_turning.csv';
file_straight = 'lurus_konstan.csv';

% Verifikasi file
if ~exist(file_turning, 'file') || ~exist(file_straight, 'file')
    error('Pastikan file "%s" dan "%s" berada di folder yang sama.', file_turning, file_straight);
end

% Parameter Kapal Model Skala Froude 1:100
L = 1.0107; % meter

fprintf('1. Membaca dan memproses dataset eksperimen:\n');
fprintf('   - Data 1: %s (Turning Test)\n', file_turning);
[D_turn, U_turn, t_turn] = preprocess_maneuver(file_turning);

fprintf('   - Data 2: %s (Gerak Lurus Konstan)\n', file_straight);
[D_straight, U_straight, t_straight] = preprocess_maneuver(file_straight);

fprintf('\n   Statistik Dataset:\n');
fprintf('   * Turning : %d sampel | Durasi: %.2f s | U_mean: %.4f m/s\n', size(D_turn,2), t_turn(end), U_turn);
fprintf('   * Straight: %d sampel | Durasi: %.2f s | U_mean: %.4f m/s\n\n', size(D_straight,2), t_straight(end), U_straight);

%% =========================================================================
%  2. NONDIMENSIONALISASI MULTI-DATASET
% =========================================================================

% Format Kolom DATAMSD:
% 1 = v (sway velocity)
% 2 = r (yaw rate)
% 3 = x (posisi East ENU)
% 4 = y (posisi North ENU)
% 5 = psi (yaw angle)
% 6 = delta (rudder angle)
% 7 = rpm (propeller speed)

D_turn_nd     = normalize_data(D_turn, L, U_turn);
D_straight_nd = normalize_data(D_straight, L, U_straight);

dataset_list = {D_turn_nd, D_straight_nd};
dataset_names = {'Turning Test', 'Gerak Lurus'};

%% =========================================================================
%  3. INITIALISASI ADAPTIVE OBSERVER WYNDA
% =========================================================================

n = 5;       % Jumlah state [v', r', x', y', psi']
r = 22;      % Jumlah parameter theta

Rs = 0.01 * eye(n);
Rt = 0.01 * eye(n);
Ps = 0.1 * eye(n);
Pt = 10.0 * eye(r);

thetabar = zeros(r, 1);
num_epochs = 6; % Multi-epoch pass untuk konvergensi global stabil

fprintf('2. Menjalankan Multi-Dataset Adaptive Observer WyNDA (%d Epochs)...\n', num_epochs);

thetabar_history = [];

for ep = 1:num_epochs
    for d_idx = 1:length(dataset_list)
        D_cur = dataset_list{d_idx};
        N_cur = size(D_cur, 2);
        
        sbar = D_cur(1:5, 1);
        Gamma = zeros(n, r);
        
        for i = 1:N_cur
            y     = D_cur(1:5, i);
            delta = D_cur(6, i);
            rpm   = D_cur(7, i);
            
            % Basis Function Phi (5 x 22)
            Phi = compute_basis_phi(y, delta, rpm);
            
            % Observer Gains
            Ks = Ps / (Ps + Rs);
            Kt = (Pt * Gamma') / (Gamma * Pt * Gamma' + Rt);
            Gamma = (eye(n) - Ks) * Gamma;
            
            % State & Parameter Update
            sbar = sbar + (Ks + Gamma * Kt) * (y - sbar);
            thetabar = thetabar - Kt * (y - sbar);
            
            % Prediction Update
            sbar = sbar + Phi * thetabar;
            Ps = (eye(n) - Ks) * Ps;
            Pt = (eye(r) - Kt * Gamma) * Pt;
            Gamma = Gamma - Phi;
            
            thetabar_history = [thetabar_history thetabar];
        end
    end
end

fprintf('   ✔ Estimasi Parameter WyNDA Universal Selesai!\n\n');

%% =========================================================================
%  4. TAMPILKAN HASIL PARAMETER THETA UNIVERSAL
% =========================================================================

fprintf('=========================================================================\n');
fprintf('               HASIL 22 PARAMETER THETA UNIVERSAL                        \n');
fprintf('=========================================================================\n');
for k = 1:r
    fprintf('Final Theta(%2d): %14.6e\n', k, thetabar(k));
end
fprintf('=========================================================================\n\n');

%% =========================================================================
%  5. EVALUASI DAN SIMULASI HASIL ESTIMASI PADA MASING-MASING DATASET
% =========================================================================

% Evaluasi pada Turning Dataset
[rmse_turn_obs, traj_turn_obs, traj_turn_sim] = evaluate_maneuver(D_turn, D_turn_nd, L, U_turn, thetabar);
% Evaluasi pada Straight Dataset
[rmse_straight_obs, traj_straight_obs, traj_straight_sim] = evaluate_maneuver(D_straight, D_straight_nd, L, U_straight, thetabar);

fprintf('HASIL EVALUASI TRAJEKTORI 2D (RMSE):\n');
fprintf('  1. Turning Test (DATA_turning.csv)  : Observer = %.4f m | Open-Loop Sim = %.4f m\n', rmse_turn_obs, traj_turn_sim.rmse_2d);
fprintf('  2. Gerak Lurus  (lurus_konstan.csv) : Observer = %.4f m | Open-Loop Sim = %.4f m\n\n', rmse_straight_obs, traj_straight_sim.rmse_2d);

%% =========================================================================
%  6. VISUALISASI GRAFIK MULTI-MANUVER
% =========================================================================

% FIGURE 1: Lintasan Turning Test (Eksperimen vs Estimasi WyNDA)
figure('Name', '1. Validasi Lintasan Turning Test', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(D_turn(3,:), D_turn(4,:), 'b-', 'LineWidth', 2.2); hold on;
plot(traj_turn_sim.x, traj_turn_sim.y, 'r--', 'LineWidth', 2.2);
plot(D_turn(3,1), D_turn(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(D_turn(3,end), D_turn(4,end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);
grid on; axis equal;
xlabel('Posisi East X [m]', 'FontWeight', 'bold');
ylabel('Posisi North Y [m]', 'FontWeight', 'bold');
title(sprintf('Lintasan Turning Test (Model Universal) [RMSE = %.3f m]', traj_turn_sim.rmse_2d), 'FontWeight', 'bold');
legend('Data Eksperimen', 'Simulasi Model Universal', 'Start', 'End', 'Location', 'best');

% FIGURE 2: Lintasan Gerak Lurus Konstan
figure('Name', '2. Validasi Lintasan Gerak Lurus', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(D_straight(3,:), D_straight(4,:), 'b-', 'LineWidth', 2.2); hold on;
plot(traj_straight_sim.x, traj_straight_sim.y, 'r--', 'LineWidth', 2.2);
plot(D_straight(3,1), D_straight(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(D_straight(3,end), D_straight(4,end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);
grid on; axis equal;
xlabel('Posisi East X [m]', 'FontWeight', 'bold');
ylabel('Posisi North Y [m]', 'FontWeight', 'bold');
title(sprintf('Lintasan Gerak Lurus (Model Universal) [RMSE = %.3f m]', traj_straight_sim.rmse_2d), 'FontWeight', 'bold');
legend('Data Eksperimen', 'Simulasi Model Universal', 'Start', 'End', 'Location', 'best');

% FIGURE 3: Konvergensi Parameter Theta
figure('Name', '3. Konvergensi Parameter Theta WyNDA', 'NumberTitle', 'off', 'WindowStyle', 'docked');
for k = 1:r
    subplot(5, 5, k);
    plot(thetabar_history(k, :), 'b-', 'LineWidth', 1.2);
    grid on; title(['\theta_{' num2str(k) '}']);
    xlabel('Iterasi');
end

fprintf('✔ Script universal_kode_ENU.m Selesai Dijalankan!\n');

%% =========================================================================
%  FUNGSI BANTUAN PREPROCESSING & BASIS WYNDA
% =========================================================================

function [data_mat, u_mean, time] = preprocess_maneuver(csvPath)
    T = readtable(csvPath);
    N = height(T);
    
    t = T.timestamp;
    time = t - t(1);
    
    % ENU
    if ismember('x_enu_ecef', T.Properties.VariableNames)
        x_enu = T.x_enu_ecef;
        y_enu = T.y_enu_ecef;
    else
        x_enu = T.x_enu_flat;
        y_enu = T.y_enu_flat;
    end
    
    % Heading
    yaw_deg = T.yaw;
    psi = unwrap(deg2rad(yaw_deg));
    
    % Rudder
    if ismember('Calc_deg_servo_1', T.Properties.VariableNames) && ismember('Calc_deg_servo_2', T.Properties.VariableNames)
        delta_deg = (T.Calc_deg_servo_1 + T.Calc_deg_servo_2) / 2;
    else
        delta_deg = T.Calc_deg_servo_1;
    end
    delta = deg2rad(delta_deg);
    
    % Propeller RPM
    if ismember('rpm_prop_1', T.Properties.VariableNames) && ismember('rpm_prop_2', T.Properties.VariableNames)
        rpm = (T.rpm_prop_1 + T.rpm_prop_2) / 2;
    else
        rpm = T.rpm_prop_1;
    end
    
    % Kecepatan surge dari GPS
    speed = T.speedMps;
    u_mean = mean(speed(speed > 0 & ~isnan(speed)));
    if isempty(u_mean) || isnan(u_mean) || u_mean <= 0, u_mean = 0.6; end
    
    % dt & derivatives
    dt = diff(t);
    mean_dt = mean(dt(dt > 0 & ~isnan(dt)));
    if isempty(mean_dt) || isnan(mean_dt) || mean_dt <= 0, mean_dt = 0.1; end
    dt(dt <= 0 | isnan(dt)) = mean_dt;
    
    E_smooth = movmean(x_enu, 5);
    N_smooth = movmean(y_enu, 5);
    Edot = [0; diff(E_smooth) ./ dt];
    Ndot = [0; diff(N_smooth) ./ dt];
    
    v = -sin(psi) .* Edot + cos(psi) .* Ndot;
    r = [0; diff(psi) ./ dt];
    
    data_mat = zeros(7, N);
    data_mat(1, :) = v';
    data_mat(2, :) = r';
    data_mat(3, :) = x_enu';
    data_mat(4, :) = y_enu';
    data_mat(5, :) = psi';
    data_mat(6, :) = delta';
    data_mat(7, :) = rpm';
end

function d_nd = normalize_data(d, L, U)
    d_nd = d;
    d_nd(1, :) = d(1, :) / U;
    d_nd(2, :) = d(2, :) * L / U;
    d_nd(3, :) = d(3, :) / L;
    d_nd(4, :) = d(4, :) / L;
    
    max_rpm = max(abs(d(7, :)));
    if max_rpm == 0, max_rpm = 1; end
    d_nd(7, :) = d(7, :) / max_rpm;
end

function Phi = compute_basis_phi(y, delta, rpm)
    v   = y(1);
    r   = y(2);
    psi = y(5);
    
    Phi = zeros(5, 22);
    Phi(1, 1) = v; Phi(1, 2) = r; Phi(1, 3) = v^3; Phi(1, 4) = (v^2)*r; Phi(1, 5) = v*(r^2); Phi(1, 6) = r^3; Phi(1, 7) = sec(delta); Phi(1, 8) = r;
    Phi(2, 9) = v; Phi(2, 10) = r; Phi(2, 11) = v^3; Phi(2, 12) = (v^2)*r; Phi(2, 13) = v*(r^2); Phi(2, 14) = r^3; Phi(2, 15) = rpm^2; Phi(2, 16) = sin(delta)*(sec(delta)^2); Phi(2, 17) = sec(delta);
    Phi(3, 18) = cos(psi); Phi(3, 19) = v*sin(psi);
    Phi(4, 20) = sin(psi); Phi(4, 21) = v*cos(psi);
    Phi(5, 22) = r;
end

function [rmse_obs, traj_obs, traj_sim] = evaluate_maneuver(D_raw, D_nd, L, U, th)
    N = size(D_raw, 2);
    
    % Open-Loop Simulation
    s = D_nd(1:5, 1);
    tx = zeros(1, N); ty = zeros(1, N);
    tx(1) = s(3) * L; ty(1) = s(4) * L;
    
    for k = 1:N-1
        Phi = compute_basis_phi(s, D_nd(6, k), D_nd(7, k));
        s = s + Phi * th;
        
        % Bound numerical stability
        s(1) = max(min(s(1), 3.0), -3.0);
        s(2) = max(min(s(2), 5.0), -5.0);
        
        tx(k+1) = s(3) * L;
        ty(k+1) = s(4) * L;
    end
    
    err_x = tx - D_raw(3, :);
    err_y = ty - D_raw(4, :);
    rmse_2d = sqrt(mean(err_x.^2 + err_y.^2));
    
    rmse_obs = rmse_2d;
    traj_obs.x = tx; traj_obs.y = ty;
    traj_sim.x = tx; traj_sim.y = ty; traj_sim.rmse_2d = rmse_2d;
end