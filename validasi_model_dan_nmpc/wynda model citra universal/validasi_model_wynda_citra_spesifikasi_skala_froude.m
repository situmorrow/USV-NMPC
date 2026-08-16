%% ============================================================================
%      VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA (UNIVERSAL MODEL)
% =============================================================================
% * Script ini memvalidasi model WyNDA Universal pada dua skenario uji:
%   1. Manuver Turning Test (DATA_turning.csv)
%   2. Manuver Gerak Lurus Konstan (lurus_konstan.csv)
% * Menggunakan parameter theta universal hasil estimasi gabungan.
% * Formulasi integrasi: Discrete State Transition WyNDA:
%   s(k+1) = s(k) + Phi(s(k), u(k)) * theta
% =============================================================================

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('=========================================================================\n');
fprintf('     VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA UNIVERSAL              \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%  1. PARAMETER MODEL WYNDA UNIVERSAL (HASIL ESTIMASI MULTI-DATASET)
% =========================================================================

L = 1.0107; % Panjang kapal model (meter)

% Parameter Theta Universal
theta = [
   7.781192e-02;   % theta_1  (v')
   2.064744e-02;   % theta_2  (r')
  -9.630146e-01;   % theta_3  ((v')^3)
   1.106213e+00;   % theta_4  ((v')^2 * r')
   2.865957e-01;   % theta_5  (v' * (r')^2)
  -4.571177e-01;   % theta_6  ((r')^3)
   2.156263e-03;   % theta_7  (sec(delta))
   2.064745e-02;   % theta_8  (u0' * r')
   8.749092e-02;   % theta_9  (v' pada r_dot)
  -8.885129e-02;   % theta_10 (r' pada r_dot)
  -2.102992e+00;   % theta_11 ((v')^3 pada r_dot)
   3.165925e+00;   % theta_12 ((v')^2 * r' pada r_dot)
  -5.742253e-01;   % theta_13 (v' * (r')^2 pada r_dot)
  -1.779182e-01;   % theta_14 ((r')^3 pada r_dot)
   3.839133e-04;   % theta_15 (n^2)
  -1.064574e-02;   % theta_16 (sin(delta)*sec^2(delta))
  -2.604523e-03;   % theta_17 (sec(delta) pada r_dot)
  -1.109729e-01;   % theta_18 (u0' * cos(psi'))
  -2.138179e-01;   % theta_19 (v' * sin(psi'))
  -1.179252e-01;   % theta_20 (u0' * sin(psi'))
  -6.764104e-02;   % theta_21 (v' * cos(psi'))
   6.433759e-02    % theta_22 (r' pada psi_dot)
];

fprintf('1. Parameter Model Universal WyNDA dimuat (22 Theta).\n\n');

%% =========================================================================
%  2. PILIHAN VALIDASI SKENARIO (ATAU JALANKAN KEDUANYA)
% =========================================================================

maneuver_files = {'DATA_turning.csv', 'lurus_konstan.csv'};
maneuver_names = {'1. Turning Test (Manuver Belok)', '2. Gerak Lurus Konstan'};

for m_idx = 1:length(maneuver_files)
    csvPath = maneuver_files{m_idx};
    m_name  = maneuver_names{m_idx};
    
    if ~exist(csvPath, 'file')
        warning('File %s tidak ditemukan, melewati skenario ini.', csvPath);
        continue;
    end
    
    fprintf('=========================================================================\n');
    fprintf('  VALIDASI SKENARIO %s\n', upper(m_name));
    fprintf('=========================================================================\n');
    fprintf('Membaca data: %s\n', csvPath);
    
    data = readtable(csvPath);
    N = height(data);
    
    timestamp = data.timestamp;
    time = timestamp - timestamp(1);
    
    % ENU
    if ismember('x_enu_ecef', data.Properties.VariableNames)
        x_exp = data.x_enu_ecef;
        y_exp = data.y_enu_ecef;
    else
        x_exp = data.x_enu_flat;
        y_exp = data.y_enu_flat;
    end
    
    % Rudder
    if ismember('Calc_deg_servo_1', data.Properties.VariableNames) && ismember('Calc_deg_servo_2', data.Properties.VariableNames)
        delta_deg_exp = (data.Calc_deg_servo_1 + data.Calc_deg_servo_2) / 2;
    else
        delta_deg_exp = data.Calc_deg_servo_1;
    end
    delta_rad_exp = deg2rad(delta_deg_exp);
    
    % RPM
    if ismember('rpm_prop_1', data.Properties.VariableNames) && ismember('rpm_prop_2', data.Properties.VariableNames)
        rpm_exp = (data.rpm_prop_1 + data.rpm_prop_2) / 2;
    else
        rpm_exp = data.rpm_prop_1;
    end
    max_rpm = max(abs(rpm_exp));
    if max_rpm == 0, max_rpm = 1; end
    rpm_nd_exp = rpm_exp / max_rpm;
    
    % Heading
    yaw_deg_exp = data.yaw;
    psi_exp_rad = unwrap(deg2rad(yaw_deg_exp));
    
    % Kecepatan surge
    speed_exp = data.speedMps;
    u_0_exp = mean(speed_exp(speed_exp > 0 & ~isnan(speed_exp)));
    if isempty(u_0_exp) || isnan(u_0_exp) || u_0_exp <= 0, u_0_exp = 0.6; end
    
    fprintf('Total sampel: %d | Durasi: %.2f s | U_mean: %.4f m/s\n', N, time(end), u_0_exp);
    
    %% SIMULASI OPEN-LOOP WYNDA
    history_sim_dim = zeros(5, N);
    
    % Inisialisasi awal t=0
    s0_dim = [0; 0; x_exp(1); y_exp(1); psi_exp_rad(1)];
    s0_nd  = [s0_dim(1)/u_0_exp; s0_dim(2)*L/u_0_exp; s0_dim(3)/L; s0_dim(4)/L; s0_dim(5)];
    
    s_nd = s0_nd;
    history_sim_dim(:, 1) = s0_dim;
    
    for k = 1:N-1
        Phi_step = compute_basis_phi(s_nd, delta_rad_exp(k), rpm_nd_exp(k));
        s_nd = s_nd + Phi_step * theta;
        
        % Proteksi batas fisik numerik
        s_nd(1) = max(min(s_nd(1), 3.0), -3.0);
        s_nd(2) = max(min(s_nd(2), 5.0), -5.0);
        
        % Konversi ke dimensional
        history_sim_dim(:, k+1) = [s_nd(1)*u_0_exp; s_nd(2)*u_0_exp/L; s_nd(3)*L; s_nd(4)*L; s_nd(5)];
    end
    
    x_sim = history_sim_dim(3, :)';
    y_sim = history_sim_dim(4, :)';
    psi_sim_rad = history_sim_dim(5, :)';
    r_sim = history_sim_dim(2, :)';
    v_sim = history_sim_dim(1, :)';
    
    %% EVALUASI RMSE
    err_x = x_exp - x_sim;
    err_y = y_exp - y_sim;
    rmse_x = sqrt(mean(err_x.^2));
    rmse_y = sqrt(mean(err_y.^2));
    rmse_2d = sqrt(mean(err_x.^2 + err_y.^2));
    
    err_psi = psi_exp_rad - psi_sim_rad;
    rmse_psi_deg = rad2deg(sqrt(mean(err_psi.^2)));
    
    fprintf('Hasil RMSE:\n');
    fprintf('  RMSE Posisi X : %.4f m\n', rmse_x);
    fprintf('  RMSE Posisi Y : %.4f m\n', rmse_y);
    fprintf('  RMSE Posisi 2D: %.4f m\n', rmse_2d);
    fprintf('  RMSE Heading  : %.2f deg\n\n', rmse_psi_deg);
    
    %% VISUALISASI GRAFIK
    fig_title = sprintf('Skenario %d - %s', m_idx, m_name);
    figure('Name', fig_title, 'NumberTitle', 'off', 'WindowStyle', 'docked');
    
    subplot(2, 2, 1);
    plot(x_exp, y_exp, 'b-', 'LineWidth', 2.0); hold on;
    plot(x_sim, y_sim, 'r--', 'LineWidth', 2.0);
    plot(x_exp(1), y_exp(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(x_exp(end), y_exp(end), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    grid on; axis equal;
    xlabel('East X [m]'); ylabel('North Y [m]');
    title(sprintf('Lintasan 2D [RMSE 2D = %.3f m]', rmse_2d));
    legend('Eksperimen', 'Model WyNDA Universal', 'Start', 'End', 'Location', 'best');
    
    subplot(2, 2, 2);
    plot(time, x_exp, 'b-', time, x_sim, 'r--', 'LineWidth', 1.5);
    grid on; xlabel('Waktu [s]'); ylabel('East X [m]');
    title('Respon Posisi X'); legend('Eksperimen', 'WyNDA', 'Location', 'best');
    
    subplot(2, 2, 3);
    plot(time, y_exp, 'b-', time, y_sim, 'r--', 'LineWidth', 1.5);
    grid on; xlabel('Waktu [s]'); ylabel('North Y [m]');
    title('Respon Posisi Y'); legend('Eksperimen', 'WyNDA', 'Location', 'best');
    
    subplot(2, 2, 4);
    plot(time, rad2deg(psi_exp_rad), 'b-', time, rad2deg(psi_sim_rad), 'r--', 'LineWidth', 1.5);
    grid on; xlabel('Waktu [s]'); ylabel('Yaw [deg]');
    title(sprintf('Respon Yaw [RMSE = %.2f deg]', rmse_psi_deg)); legend('Eksperimen', 'WyNDA', 'Location', 'best');
end

fprintf('✔ Seluruh Validasi Skenario Selesai Tanpa Error!\n');

%% =========================================================================
%  FUNGSI BASIS PHI
% =========================================================================

function Phi = compute_basis_phi(s, delta, rpm_nd)
    v   = s(1);
    r   = s(2);
    psi = s(5);
    
    Phi = zeros(5, 22);
    Phi(1, 1) = v; Phi(1, 2) = r; Phi(1, 3) = v^3; Phi(1, 4) = (v^2)*r; Phi(1, 5) = v*(r^2); Phi(1, 6) = r^3; Phi(1, 7) = sec(delta); Phi(1, 8) = r;
    Phi(2, 9) = v; Phi(2, 10) = r; Phi(2, 11) = v^3; Phi(2, 12) = (v^2)*r; Phi(2, 13) = v*(r^2); Phi(2, 14) = r^3; Phi(2, 15) = rpm_nd^2; Phi(2, 16) = sin(delta)*(sec(delta)^2); Phi(2, 17) = sec(delta);
    Phi(3, 18) = cos(psi); Phi(3, 19) = v*sin(psi);
    Phi(4, 20) = sin(psi); Phi(4, 21) = v*cos(psi);
    Phi(5, 22) = r;
end
