clc; clear; close all;
% Uji coba dengan koordinat berupa lat lon dan mengubahnya menjadi ENU
% Uji coba dengan waypoints yang sudah ada

fprintf('Memulai simulasi NMPC...\n');
timer_val = tic;

%% 1. Parameter Simulasi & Trajektori Referensi
N = 30;             % Horizon prediksi (harus sama dengan Tp di nmpc_kapal.m)
T_sim = 1.0;        % Waktu sampling (detik)
T_final = 150;      % Durasi simulasi
N_steps = round(T_final / T_sim);

L = 101.07;         % Panjang kapal (meter)
u_0 = 15.4;         % Kecepatan nominal (m/s)
u_0_nd = 1;         % Kecepatan non-dimensional
dt_nd = T_sim * u_0 / L;

% Generasi Trajektori Referensi (Bentuk Sinusoidal)
time_vector = (0:N_steps-1)' * T_sim;
x_ref_dim = u_0 * time_vector; 
y_ref_dim = 80 * sin(0.015 * x_ref_dim);

dx_ref = u_0 * ones(size(time_vector));
dy_ref = 80 * 0.015 * cos(0.015 * x_ref_dim) * u_0;
psi_ref_full = atan2(dy_ref, dx_ref);

x_ref_nd_full = x_ref_dim / L;
y_ref_nd_full = y_ref_dim / L;

%% 2. Setup State Awal & Penampung Data Simulasi
% State non-dimensional: [v; r; x; y; psi]
current_state_nd = [0; 0; x_ref_nd_full(1); y_ref_nd_full(1); psi_ref_full(1)];
u_prev = 0;

history_input = zeros(1, N_steps);
hist_state_nd = zeros(5, N_steps);

%% 3. Matriks Sistem (Digunakan untuk Update Plant Fisik Kapal)
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

%% 4. Loop Utama Simulasi (Memanggil Entry Point nmpc_kapal)
for k = 1:N_steps
    % Ekstrak sekuens horizon N x 1 untuk dikirim ke NMPC
    if k + N - 1 <= N_steps
        x_ref_seq   = x_ref_nd_full(k : k + N - 1);
        y_ref_seq   = y_ref_nd_full(k : k + N - 1);
        psi_ref_seq = psi_ref_full(k : k + N - 1);
    else
        num_valid = N_steps - k + 1;
        num_pad   = N - num_valid;
        x_ref_seq   = [x_ref_nd_full(k:end); repmat(x_ref_nd_full(end), num_pad, 1)];
        y_ref_seq   = [y_ref_nd_full(k:end); repmat(y_ref_nd_full(end), num_pad, 1)];
        psi_ref_seq = [psi_ref_full(k:end); repmat(psi_ref_full(end), num_pad, 1)];
    end

    % --- PEMANGGILAN ENTRY POINT FUNCTION ---
    % Catatan: Setelah membuat MEX file, ganti 'nmpc_kapal' menjadi 'nmpc_kapal_mex'
    [u_opt, exitflag] = nmpc_kapal(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq);

    % Simpan riwayat
    hist_state_nd(:, k) = current_state_nd;
    history_input(k)    = u_opt;

    % Update state dinamika plant kapal (Simulasi pergerakan fisik)
    current_state_nd = euler_step(@(s, u) ship_dynamics(s, u, A_sys, B_sys, u_0_nd), ...
                                  current_state_nd, u_opt, dt_nd);
    u_prev = u_opt;
end

waktu = toc(timer_val);
fprintf('SIMULASI SELESAI\n');
fprintf('Total Waktu Komputasi: %.4f detik\n', waktu);

%% 5. Konversi Output Ke Satuan Dimensional & Perhitungan RMSE
hist_dim = [
    hist_state_nd(3, :) * L;          % X (m)
    hist_state_nd(4, :) * L;          % Y (m)
    hist_state_nd(5, :);              % Psi (rad)
    hist_state_nd(1, :) * u_0;        % v (m/s)
    hist_state_nd(2, :) * (u_0 / L)   % r (rad/s)
];

err_x   = hist_dim(1, :) - x_ref_dim';
err_y   = hist_dim(2, :) - y_ref_dim';
err_psi = atan2(sin(hist_dim(3, :) - psi_ref_full'), cos(hist_dim(3, :) - psi_ref_full'));

rmse_data = [
    sqrt(mean(err_x.^2));
    sqrt(mean(err_y.^2));
    sqrt(mean(err_psi.^2))
];

fprintf('\n Hasil Perhitungan RMSE \n');
fprintf('RMSE X   : %.4f meter\n', rmse_data(1));
fprintf('RMSE Y   : %.4f meter\n', rmse_data(2));
fprintf('RMSE Psi : %.4f rad (%.4f derajat)\n\n', rmse_data(3), rad2deg(rmse_data(3)));

%% 6. Plotting Hasil Simulasi
% Figure 1: Trajektori Kapal
figure('Name', 'Trajektori Kapal', 'Color', 'w');
plot(x_ref_dim, y_ref_dim, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Referensi'); hold on;
plot(hist_dim(1,:), hist_dim(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'NMPC Track');
grid on; axis equal;
xlabel('Posisi X (m)'); ylabel('Posisi Y (m)');
title('Trajektori Posisi Kapal'); legend('Location', 'best');

% Figure 2: Dynamic States
figure('Name', 'Dynamic States', 'Color', 'w');
subplot(3,1,1);
plot(time_vector, hist_dim(4,:), 'b', 'LineWidth', 1.5); grid on;
ylabel('v (m/s)'); title('Kecepatan Sway (v)');

subplot(3,1,2);
plot(time_vector, rad2deg(hist_dim(5,:)), 'b', 'LineWidth', 1.5); grid on;
ylabel('r (deg/s)'); title('Kecepatan Yaw Rate (r)');

subplot(3,1,3);
plot(time_vector, rad2deg(hist_dim(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, rad2deg(psi_ref_full), 'r--', 'LineWidth', 1.5); grid on;
ylabel('\psi (deg)'); xlabel('Waktu (detik)'); title('Sudut Heading (\psi)');
legend('Aktual', 'Referensi', 'Location', 'best');

% Figure 3: Control Input
figure('Name', 'Control Input', 'Color', 'w');
plot(time_vector, rad2deg(history_input), 'k-', 'LineWidth', 1.5); grid on;
xlabel('Waktu (detik)'); ylabel('Sudut Kemudi \delta (derajat)');
title('Sinyal Kontrol Kemudi Rudder');

% Figure 4: Tracking Error
figure('Name', 'Tracking Errors', 'Color', 'w');
subplot(3,1,1); plot(time_vector, err_x, 'r', 'LineWidth', 1.5); grid on; ylabel('Error X (m)');
subplot(3,1,2); plot(time_vector, err_y, 'g', 'LineWidth', 1.5); grid on; ylabel('Error Y (m)');
subplot(3,1,3); plot(time_vector, rad2deg(err_psi), 'b', 'LineWidth', 1.5); grid on; ylabel('Error \psi (deg)'); xlabel('Waktu (s)');

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