%% SCRIPT UJI VALIDASI DATA YAW
data = readtable('gerak_random.csv');
t = data.timestamp - data.timestamp(1);
yaw_deg = data.yaw;
speed = data.speedMps;
x_gps = data.x_enu_flat;
y_gps = data.y_enu_flat;
N = height(data);

% 1. Dead Reckoning dari Speed & Yaw
x_dr = zeros(N,1); y_dr = zeros(N,1);
x_dr(1) = x_gps(1); y_dr(1) = y_gps(1);

for k = 1:N-1
    dt = t(k+1) - t(k);
    if dt <= 0, dt = 0.1; end
    psi_enu_rad = deg2rad(yaw_deg(k)+180);
    x_dr(k+1) = x_dr(k) + speed(k) * cos(psi_enu_rad) * dt;
    y_dr(k+1) = y_dr(k) + speed(k) * sin(psi_enu_rad) * dt;
end

% 2. Plot Visualisasi Validasi
figure('Name', 'Validasi Data Yaw via Dead Reckoning');
plot(x_gps, y_gps, 'b-', 'LineWidth', 2); hold on;
plot(x_dr, y_dr, 'r--', 'LineWidth', 2);
grid on; axis equal;
xlabel('East X [m]'); ylabel('North Y [m]');
title('Validasi Yaw: GPS Trajectory vs Dead Reckoning (Speed + Yaw)');
legend('GPS ENU Real', 'Dead Reckoning (Integrasi Yaw)');
