%% =====================================
% PREPROCESSING GPS → ENU + EXPORT EXCEL
% =====================================
clear; clc;

%% LOAD DATA
filename = 'TURNING OTOMATIS 35.xlsx';
T = readtable(filename);

t_raw   = T{:,1}; % timestamp
lat_deg = T{:,2}; % lat
lon_deg = T{:,3}; % lon

yaw_deg = T{:,8}; % heading
rpm_prop_1 = T{:,10}; % rpm prop 1
rpm_prop_2 = T{:,11}; % rpm prop 2

delta_deg = (T{:,4} + T{:,5})/2; % delta (sudut rudder)

%% Konversi Lat-Lon ke ECEF ke ENU
% CONSTANT WGS84
a  = 6378137;
f  = 1/298.257223563;
e2 = f*(2-f);

% REF POINT (ENU ORIGIN)
lat0 = deg2rad(lat_deg(1));
lon0 = deg2rad(lon_deg(1));

% LLA → ECEF
lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

N = a ./ sqrt(1 - e2*sin(lat).^2);

X = N .* cos(lat) .* cos(lon);
Y = N .* cos(lat) .* sin(lon);
Z = (N*(1-e2)) .* sin(lat);

% SHIFT ORIGIN
dX = X - X(1);
dY = Y - Y(1);
dZ = Z - Z(1);

% ECEF → ENU
R = [ -sin(lon0),              cos(lon0),             0;
      -sin(lat0)*cos(lon0),   -sin(lat0)*sin(lon0),  cos(lat0);
       cos(lat0)*cos(lon0),    cos(lat0)*sin(lon0),  sin(lat0) ];

ENU = R * [dX dY dZ]';

x = ENU(1,:)';   % East
y = ENU(2,:)';   % North
z = ENU(3,:)';   % Up

%% Merubah format data
% HEADING
psi = unwrap(deg2rad(yaw_deg));

% RUDDER
delta = deg2rad(delta_deg);

% PROPELLER
n_P = (rpm_prop_1 + rpm_prop_2) / 120;
% n_S = rpm_prop_2 / 60;

% TIME STEP
dt = diff(t_raw);
dt(dt <= 0 | isnan(dt)) = mean(dt(dt > 0));

% VELOCITY (ENU FRAME)
vx = [0; diff(x)./dt];
vy = [0; diff(y)./dt];

% BODY-FIXED VELOCITY
u =  cos(psi).*vx + sin(psi).*vy;
v = -sin(psi).*vx + cos(psi).*vy;

% YAW RATE
r = [0; diff(psi)./dt];

% FORCE COLUMN VECTOR
u = u(:); v = v(:); r = r(:);
x = x(:); y = y(:); psi = psi(:);
delta = delta(:); n_P = n_P(:);

% STATE VECTOR
state = [v r x y psi delta n_P];

%% TABLE OUTPUT
T_out = array2table(state, ...
    'VariableNames', ...
    {'v','r','x_ENU','y_ENU','psi','delta','n_P'});

%% SAVE TO EXCEL
writetable(T_out, 'hasil_preprocessing_ENU.xlsx');

disp('✔ BERHASIL: DATA ENU DISIMPAN KE EXCEL');