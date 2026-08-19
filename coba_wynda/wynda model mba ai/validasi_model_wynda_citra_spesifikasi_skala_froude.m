%% ============================================================================
%            VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA (SKALA FROUDE)
% =============================================================================
% * Validasi open-loop model WyNDA berbasis persamaan (4.27) - (4.31)
%   hasil estimasi parameter adaptive observer (Theta 1 s.d. Theta 11).
% * Model mencakup 5 state:
%   s = [v'; r'; x'; y'; psi']
%   - v'   : Sway velocity nondimensional
%   - r'   : Yaw rate nondimensional
%   - x'   : Posisi East ENU nondimensional
%   - y'   : Posisi North ENU nondimensional
%   - psi' : Yaw heading (radian)
% * Input model:
%   u = [delta; n]
%   - delta : Sudut rudder kemudi (radian)
%   - n     : Propeller speed (nondimensional RPM)
% * Formulasi integrasi: Discrete State Transition WyNDA:
%   s(k+1) = s(k) + Phi(s(k), u(k)) * theta
% =============================================================================

clear; clc; close all;
set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock visualisasi figure

fprintf('=========================================================================\n');
fprintf('    SIMULASI VALIDASI MODEL MATEMATIKA KAPAL METODE WYNDA                \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%  1. READ & PREPROCESS DATA EKSPERIMEN CSV
% =========================================================================

csvPath = 'ZIGZAG.csv';
if ~exist(csvPath, 'file')
    if exist(fullfile('WyNDA model citra', 'DATA_FIKS.csv'), 'file')
        csvPath = fullfile('WyNDA model citra', 'DATA_FIKS.csv');
    else
        error('File %s tidak ditemukan! Pastikan file berada di direktori yang sesuai.', csvPath);
    end
end

fprintf('1. Membaca data eksperimen dari: %s\n', csvPath);
data = readtable(csvPath);
N = height(data);

% Waktu
timestamp = data.timestamp;
time = timestamp - timestamp(1); % Waktu relatif (detik)

% Posisi eksperimen ENU (Meter)
if ismember('x_enu_ecef', data.Properties.VariableNames) && ismember('y_enu_ecef', data.Properties.VariableNames)
    x_exp = data.x_enu_ecef;
    y_exp = data.y_enu_ecef;
elseif ismember('x_enu_flat', data.Properties.VariableNames) && ismember('y_enu_flat', data.Properties.VariableNames)
    x_exp = data.x_enu_flat;
    y_exp = data.y_enu_flat;
else
    error('Kolom koordinat ENU (x_enu_ecef / x_enu_flat) belum ada di file CSV!');
end

% Input Sudut Rudder Eksperimen (delta) dalam radian
if ismember('Calc_deg_servo_1', data.Properties.VariableNames) && ismember('Calc_deg_servo_2', data.Properties.VariableNames)
    delta_deg_exp = (data.Calc_deg_servo_1 + data.Calc_deg_servo_2) / 2;
elseif ismember('Calc_deg_servo_1', data.Properties.VariableNames)
    delta_deg_exp = data.Calc_deg_servo_1;
else
    delta_deg_exp = zeros(N, 1);
end
delta_rad_exp = deg2rad(delta_deg_exp);

% Input Propeller RPM Eksperimen
if ismember('rpm_prop_1', data.Properties.VariableNames) && ismember('rpm_prop_2', data.Properties.VariableNames)
    rpm_exp = (data.rpm_prop_1 + data.rpm_prop_2) / 2;
elseif ismember('rpm_prop_1', data.Properties.VariableNames)
    rpm_exp = data.rpm_prop_1;
else
    rpm_exp = ones(N, 1);
end
% Nondimensionalisasi RPM (scaling terhadap nilai maksimum)
max_rpm = max(abs(rpm_exp));
if max_rpm == 0, max_rpm = 1; end
rpm_nd_exp = rpm_exp / max_rpm;

% Yaw Eksperimen (Compass Heading) -> konversi ke radian unwrapped
yaw_deg_exp = data.yaw;
psi_exp_rad = unwrap(deg2rad(yaw_deg_exp));

% Kecepatan surge eksperimen dari GPS (speedMps)
if ismember('speedMps', data.Properties.VariableNames)
    speed_exp = data.speedMps;
    u_0_exp = mean(speed_exp(speed_exp > 0 & ~isnan(speed_exp)));
    if isempty(u_0_exp) || isnan(u_0_exp) || u_0_exp <= 0
        u_0_exp = 0.5923;
    end
else
    u_0_exp = 0.5923;
end

fprintf('   Total sampel data : %d\n', N);
fprintf('   Durasi eksperimen : %.2f detik\n', time(end));
fprintf('   Kecepatan rata-rata surge u_0 : %.4f m/s\n\n', u_0_exp);

%% =========================================================================
%  2. SPESIFIKASI KAPAL & PARAMETER THETA WYNDA (Persamaan 4.27 - 4.31)
% =========================================================================

L = 1.0107;          % Panjang kapal model [meter]
u_0 = u_0_exp;       % Kecepatan surge referensi kapal model [m/s]
u_0_nd = 1.0;        % Kecepatan surge nondimensional

% Vektor Parameter Theta Hasil Identifikasi WyNDA (11 Parameter)
theta = [
   -1.7298e-02;  % theta_1  : v'          pada v_dot (Eq 4.27)
   -1.2976e-02;  % theta_2  : r'          pada v_dot (Eq 4.27)
   -1.8184e-02;  % theta_3  : delta       pada v_dot (Eq 4.27)
   -3.6624e-02;  % theta_4  : v'          pada r_dot (Eq 4.28)
   -1.3674e-02;  % theta_5  : r'          pada r_dot (Eq 4.28)
   -2.6956e-02;  % theta_6  : delta       pada r_dot (Eq 4.28)
   -6.0670e-02;  % theta_7  : u0*cos(psi) pada x_dot (Eq 4.29)
    9.6901e-02;  % theta_8  : -v*sin(psi) pada x_dot (Eq 4.29)
   -5.7507e-02;  % theta_9  : u0*sin(psi) pada y_dot (Eq 4.30)
    3.3611e-02;  % theta_10 : v*cos(psi)  pada y_dot (Eq 4.30)
    6.1929e-02   % theta_11 : r'          pada psi_dot (Eq 4.31)
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

fprintf('3. Menjalankan simulasi model WyNDA...\n');
for k = 1:N-1
    % Input kontrol pada step k: delta_rad dan rpm_nd
    delta_k = delta_rad_exp(k);
    rpm_k   = rpm_nd_exp(k);
    
    % Perhitungan basis fungsi Phi_step untuk state s_nd saat ini
    Phi_step = compute_basis_phi(s_nd, delta_k, rpm_k, u_0_nd);
    
    % Update state transisi diskret: s(k+1) = s(k) + Phi * theta
    delta_s = Phi_step * theta;
    s_nd = s_nd + delta_s;
    
    history_state_sim_nd(:, k+1) = s_nd;
end

% Konversi seluruh state hasil simulasi dari nondimensional ke dimensional
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

% FIGURE 1: Perbandingan Lintasan 2D (Eksperimen vs Model WyNDA)
figure('Name', '1. Validasi Lintasan 2D Kapal WyNDA', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(x_exp, y_exp, 'b-', 'LineWidth', 2.2); hold on;
plot(x_sim, y_sim, 'r--', 'LineWidth', 2.2);
plot(x_exp(1), y_exp(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2); % Start
plot(x_exp(end), y_exp(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2); % End
grid on; axis equal;
xlabel('Posisi East X [meter]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Posisi North Y [meter]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Validasi Lintasan 2D Kapal (Model WyNDA vs Eksperimen) [RMSE 2D = %.3f m]', rmse_pos_2d), 'FontSize', 12, 'FontWeight', 'bold');
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
title('Kecepatan Sudut Putar Kapal (Yaw Rate r) Hasil Simulasi WyNDA');

% FIGURE 4: Respon Kecepatan Sway (v) Hasil WyNDA
figure('Name', '4. Respon Kecepatan Sway (v)', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, v_sim, 'r-', 'LineWidth', 1.8);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Sway Velocity v [m/s]', 'FontWeight', 'bold');
title('Respon Kecepatan Samping Kapal (Sway Velocity v) Model WyNDA');

% FIGURE 5: Input Sinyal Kontrol Kemudi & Propeller
figure('Name', '5. Input Sinyal Kontrol RC', 'NumberTitle', 'off', 'WindowStyle', 'docked');
subplot(2,1,1);
plot(time, delta_deg_exp, 'k-', 'LineWidth', 1.5);
grid on; ylabel('Sudut Rudder [derajat]', 'FontWeight', 'bold');
title('Input Sinyal Kemudi Rudder (\delta)');

subplot(2,1,2);
plot(time, rpm_exp, 'c-', 'LineWidth', 1.5);
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Propeller RPM', 'FontWeight', 'bold');
title('Input Putaran Propeller (RPM)');

% FIGURE 6: Grafik Eror Jarak Posisi 2D (Euclidean Error)
figure('Name', '6. Eror Jarak Posisi 2D', 'NumberTitle', 'off', 'WindowStyle', 'docked');
plot(time, err_pos_2d, 'r-', 'LineWidth', 1.5); hold on;
yline(rmse_pos_2d, 'b--', sprintf('RMSE 2D = %.3f m', rmse_pos_2d), 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
grid on; xlabel('Waktu [detik]', 'FontWeight', 'bold'); ylabel('Galat Jarak [meter]', 'FontWeight', 'bold');
title('Galat Jarak Posisi 2D (Euclidean Distance Error) Model WyNDA');
legend('Eror Jarak per Detik', 'Nilai RMSE 2D', 'Location', 'best');

fprintf('✔ Validasi Model WyNDA Selesai!\n');

%% =========================================================================
%  6. FUNGSI BASIS PHI MODEL WYNDA (Persamaan 4.27 - 4.31)
% =========================================================================

function Phi = compute_basis_phi(s, delta, rpm_nd, u0_nd)
    % State:
    % v   = s(1) : sway velocity nondimensional (v')
    % r   = s(2) : yaw rate nondimensional (r')
    % x   = s(3) : x ENU nondimensional (x')
    % y   = s(4) : y ENU nondimensional (y')
    % psi = s(5) : yaw heading (psi')
    
    v   = s(1);
    r   = s(2);
    psi = s(5);
    
    Phi = zeros(5, 11);
    
    % Persamaan (4.27) -> Row 1 (v_dot)
    Phi(1, 1) = v;
    Phi(1, 2) = r;
    Phi(1, 3) = delta;
    
    % Persamaan (4.28) -> Row 2 (r_dot)
    Phi(2, 4) = v;
    Phi(2, 5) = r;
    Phi(2, 6) = delta;
    
    % Persamaan (4.29) -> Row 3 (x_dot)
    Phi(3, 7) = u0_nd * cos(psi);
    Phi(3, 8) = -v * sin(psi);
    
    % Persamaan (4.30) -> Row 4 (y_dot)
    Phi(4, 9)  = u0_nd * sin(psi);
    Phi(4, 10) = v * cos(psi);
    
    % Persamaan (4.31) -> Row 5 (psi_dot)
    Phi(5, 11) = r;
end

%% =========================================================================
%  7. FUNGSI KONVERSI DIMENSIONAL DAN NONDIMENSIONAL
% =========================================================================

function x_nd = dimensional_to_nondimensional(x, L, u0)
    % x = [v; r; x_pos; y_pos; psi]
    x_nd = [x(1)/u0; x(2)*L/u0; x(3)/L; x(4)/L; x(5)];
end

function x_dim = nondimensional_to_dimensional(x, L, u0)
    % x = [v'; r'; x'; y'; psi']
    x_dim = [x(1)*u0; x(2)*u0/L; x(3)*L; x(4)*L; x(5)];
end
