%% Research code by Agus Hasan
clear;
clc;
close all;

%% ==============================
% LOAD DATA
%% ==============================
DATAMSD = readmatrix('hasil_preprocessing_ENU.xlsx','Sheet','Sheet1')';
DATAMSD = DATAMSD(:,55:200);

%% ==============================
% PARAMETER REFERENSI
%% ==============================
L = 101.07;      % meter
U = 15.4;        % m/s

%% ==============================
% NONDIMENSIONALISASI
% 1 = v (m/s)
% 2 = r (rad/s)
% 3 = x (m)
% 4 = y (m)
% 5 = psi (rad)
% 6 = delta (rad)
% 7 = rpm
%% ==============================

DATAMSD(1,:) = DATAMSD(1,:) / U;        % v'
DATAMSD(2,:) = DATAMSD(2,:) * L / U;    % r'
DATAMSD(3,:) = DATAMSD(3,:) / L;        % x'
DATAMSD(4,:) = DATAMSD(4,:) / L;        % y'

rpm_all = DATAMSD(7,:);
rpm_all = rpm_all / max(abs(rpm_all));  % scaling rpm sekali saja

%% ==============================
% Tampilkan Nilai Awal Sebelum & Sesudah Nondimensional
%% ==============================

disp('==============================================')
disp('Nilai Inisialisasi Sebelum Nondimensional:')
disp('==============================================')

v_dim   = DATAMSD(1,1) * U;
r_dim   = DATAMSD(2,1) * U / L;
x_dim   = DATAMSD(3,1) * L;
y_dim   = DATAMSD(4,1) * L;
psi_dim = DATAMSD(5,1);

fprintf('v  (m/s)   = %.6f\n', v_dim);
fprintf('r  (rad/s) = %.6f\n', r_dim);
fprintf('x  (m)     = %.6f\n', x_dim);
fprintf('y  (m)     = %.6f\n', y_dim);
fprintf('psi (rad)  = %.6f\n', psi_dim);

disp(' ')
disp('==============================================')
disp('Nilai Inisialisasi Setelah Nondimensional:')
disp('==============================================')

fprintf('v''   = %.6f\n', DATAMSD(1,1));
fprintf('r''   = %.6f\n', DATAMSD(2,1));
fprintf('x''   = %.6f\n', DATAMSD(3,1));
fprintf('y''   = %.6f\n', DATAMSD(4,1));
fprintf('psi  = %.6f\n', DATAMSD(5,1));

%% ==============================
% Scaling Rudder dan RPM
%% ==============================

delta_dim = DATAMSD(6,1);          % rad (sudah dimensional)
rpm_dim   = DATAMSD(7,1);          % rpm asli

% --- Nondimensional ---
delta_nd = delta_dim;              % rad sudah nondimensional
rpm_nd   = rpm_dim / max(abs(DATAMSD(7,:)));  % scaling relatif

disp('==============================================')
disp('RUDDER & RPM (Dimensional dan Nondimensional)')
disp('==============================================')

fprintf('delta_dim (rad) = %.6f\n', delta_dim);
fprintf('delta_nd        = %.6f\n', delta_nd);

fprintf('rpm_dim         = %.6f\n', rpm_dim);
fprintf('rpm_nd          = %.6f\n', rpm_nd);
%% ==============================
% SIMULATION HORIZON
%% ==============================
N = size(DATAMSD,2);

dt_dim = 0.1;                 % detik
dt = dt_dim * U / L;          % nondimensional time

t = (0:N-1)*dt;
tf = t(end);

% INFORMASI TIME STEP

fprintf('\n==============================\n');
fprintf('INFORMASI TIME STEP\n');
fprintf('==============================\n');

fprintf('dt dimensional  (detik)          = %.6f s\n', dt_dim);
fprintf('dt nondimensional (U/L * dt)     = %.6f \n', dt);
fprintf('Skala waktu U/L                  = %.6f 1/s\n', U/L);
fprintf('Total waktu dimensional          = %.6f s\n', (N-1)*dt_dim);
fprintf('Total waktu nondimensional       = %.6f \n', t(end));

%% ==============================
% STATE & PARAMETER INIT
%% ==============================
n = 5;
r = 22;

s = DATAMSD(1:5,1);
sbar = s;
thetabar = zeros(r,1);

%% ==============================
% STORAGE
%% ==============================
sArray        = [];
sbarArray     = [];
thetabarArray = [];
uArray        = [];

%% ==============================
% ESTIMATOR PARAMETER
%% ==============================
lambdav = 0.3;
lambdat = 0.8;

Rs = eye(n);
Rt = eye(n);
Ps = 0.1*eye(n);
Pt = 0.1*eye(r);
Gamma = zeros(n,r);

%% ==============================
% MAIN LOOP
%% ==============================
for i = 1:N

    y = DATAMSD(1:5,i);
    delta = DATAMSD(6,i);
    rpm   = rpm_all(i);

    sArray        = [sArray s];
    sbarArray     = [sbarArray sbar];
    thetabarArray = [thetabarArray thetabar];
    u = [delta; rpm]
    uArray        = [uArray u];

    % ===== BASIS FUNCTION =====
    Phi = [

    % ---- Eq 1 ---- (8 + 14 = 22)
    y(1) y(2) y(1)^3 (y(1)^2)*y(2) y(1)*(y(2)^2) y(2)^3 ...
    sec(delta) y(2) zeros(1,14);

    % ---- Eq 2 ---- (8 + 9 + 5 = 22)
    zeros(1,8) ...
    y(1) y(2) y(1)^3 (y(1)^2)*y(2) y(1)*(y(2)^2) y(2)^3 ...
    rpm^2 sec(delta)*tan(delta) sec(delta) ...
    zeros(1,5);

    % ---- Eq 3 ----
    zeros(1,17) cos(y(5)) y(1)*sin(y(5)) zeros(1,3);

    % ---- Eq 4 ----
    zeros(1,19) sin(y(5)) y(1)*cos(y(5)) zeros(1,1);

    % ---- Eq 5 ----
    zeros(1,21) y(2)
    ];

    % Cek ukuran
    % disp(size(Phi))  % harus 5 x 22

    %% ===== ADAPTIVE OBSERVER (TIDAK DIUBAH) =====

    Ks = Ps*inv(Ps+Rs);
    Kt = Pt*Gamma' / (Gamma*Pt*Gamma' + Rt);
    Gamma = (eye(n)-Ks)*Gamma;

    sbar = sbar + (Ks+Gamma*Kt)*(y-sbar);
    thetabar = thetabar - Kt*(y-sbar);

    sbar = sbar + Phi*thetabar;
    thetabar = thetabar;
    Ps = (1/lambdav)*(eye(n)-Ks)*Ps;
    Pt = (1/lambdat)*(eye(r)-Kt*Gamma)*Pt;
    Gamma = Gamma - Phi;

end

%% ==============================
% KEMBALIKAN KE DIMENSIONAL UNTUK PLOT
%% ==============================

x_meas = DATAMSD(3,:) * L;
y_meas = DATAMSD(4,:) * L;

x_est  = sbarArray(3,:) * L;
y_est  = sbarArray(4,:) * L;

%% ==============================
% RMSE TRAJEKTORI 2D
%% ==============================

error_x = x_meas - x_est;
error_y = y_meas - y_est;

error_traj = sqrt(error_x.^2 + error_y.^2);   % error jarak tiap titik

RMSE_traj = sqrt(mean(error_traj.^2));

fprintf('\n==============================\n')
fprintf('RMSE Trajectory (meter)\n')
fprintf('==============================\n')
fprintf('RMSE_traj = %.6f m\n', RMSE_traj);

%% SMOOTHING
window = 5;
x_est = movmean(x_est,window);
y_est = movmean(y_est,window);

%% ==============================
% PLOT TRAJECTORY
%% ==============================

figure
plot(x_meas,y_meas,'b','LineWidth',2)
hold on
plot(x_est,y_est,'--r','LineWidth',2)
grid on
axis equal
ylabel('n [m]','FontSize',24)
xlabel('e [m]')
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('Measured','WyNDA Estimated')
grid on; grid minor;

%% ==============================
% RMSE
%% ==============================

error = DATAMSD(1:5,:) - sbarArray;
RMSE  = sqrt(mean(error.^2,2));

disp('==============================')
fprintf('Final Theta: %.4e\n', thetabar)
disp('==============================')
disp('RMSE:')
disp(RMSE)

% Plot RMSE tiap state
figure
bar(RMSE)
set(gca,'FontSize',14)
grid on
ylabel('RMSE')
title('RMSE tiap State')

xticklabels({'y_1','y_2','y_3','y_4','y_5'})

%% ==========================================
% ESTIMASI POSISI Y (meter) dan YAW RATE r

y_meas_posY = DATAMSD(4,:)*L;        % posisi Y terukur (meter)
y_est_posY  = sbarArray(4,:)*L;      % posisi Y hasil WyNDA

r_meas = DATAMSD(2,:)*U/L;             % yaw rate terukur (rad/s)
r_est  = sbarArray(2,:)&U/L;           % yaw rate hasil WyNDA

%% ==============================
% Plot Posisi Y (meter)

figure
plot(t, y_meas_posY, 'b','LineWidth',10)
hold on
plot(t, y_est_posY, '--r','LineWidth',10)
grid on
ylabel('n [m]','FontSize',24)
xlabel('Time (s)')
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('Measured','WyNDA Estimated')
grid on; grid minor;

%%
figure(4)

plot(t, DATAMSD(1,:)*U ,'-b','LineWidth',10);
hold on
plot(t, sbarArray(1,:)*U ,':r','LineWidth',10);
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('measured','estimated')
grid on; grid minor;
ylabel('v [m/s]','FontSize',24)
xlabel('Time (s)')

%%
figure(5)

plot(t, DATAMSD(2,:)*U/L ,'-b','LineWidth',10);
hold on
plot(t, sbarArray(2,:)*U/L ,':r','LineWidth',10);
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('measured','estimated')
grid on; grid minor;
ylabel('r [rad/s]','FontSize',24)
xlabel('Time (s)')

%%
figure(6)

rows = 5;
cols = 5;

for k = 1:r
    subplot(rows,cols,k)
    plot(t,thetabarArray(k,:),'-b','LineWidth',5);
    set(gca,'color','white','LineWidth',2,'FontSize',14)
    grid on; grid minor;
    title(['\theta_{' num2str(k) '}'])
end
%% ==============================
% Plot Posisi X (meter)
%% ==============================

x_meas = DATAMSD(3,:) * L;        % kembalikan ke meter
x_est  = sbarArray(3,:) * L;      % estimasi ke meter

figure
plot(t, x_meas, 'b','LineWidth',10)
hold on
plot(t, x_est, '--r','LineWidth',10)
grid on
ylabel('e [m]','FontSize',24)
xlabel('Time (s)')
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('Measured','WyNDA Estimated')
grid on; grid minor;

figure(8)
plot(t, DATAMSD(5,:), '-b', 'LineWidth', 10);
hold on;
plot(t, sbarArray(5,:), ':r', 'LineWidth', 10);
set(gca,'color','white','LineWidth',3,'FontSize',24)
legend('measured','estimated')
grid on;
grid minor;
ylabel('\psi [rad]','FontSize',24)
xlabel('t (s)')