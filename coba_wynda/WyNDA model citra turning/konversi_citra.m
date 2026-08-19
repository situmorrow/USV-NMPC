%% =====================================
% PREPROCESSING GPS → ENU + EXPORT EXCEL
% =====================================
clear; clc;

%% LOAD DATA
filename = 'DATA_turning.xlsx';
if ~exist(filename, 'file')
    if exist('DATA_turning.csv', 'file')
        filename = 'DATA_turning.csv';
    else
        error('File data tidak ditemukan!');
    end
end
T = readtable(filename);

%% HELPER UNTUK KONVERSI KE NUMERIK
to_num = @(x) to_double_vector(x);

%% EKSTRAKSI DATA DENGAN NAMA KOLOM / INDEKS
if ismember('timestamp', T.Properties.VariableNames)
    t_raw = to_num(T.timestamp);
else
    t_raw = to_num(T{:,1});
end

if ismember('latitude', T.Properties.VariableNames)
    lat_deg = to_num(T.latitude);
elseif ismember('lat', T.Properties.VariableNames)
    lat_deg = to_num(T.lat);
else
    lat_deg = to_num(T{:,2});
end

if ismember('longitude', T.Properties.VariableNames)
    lon_deg = to_num(T.longitude);
elseif ismember('lon', T.Properties.VariableNames)
    lon_deg = to_num(T.lon);
else
    lon_deg = to_num(T{:,3});
end

% Heading / Yaw (Kolom 9 pada data)
if ismember('yaw', T.Properties.VariableNames)
    yaw_deg = to_num(T.yaw);
else
    yaw_deg = to_num(T{:,9});
end

% Propeller RPM (Kolom 11 & 12 pada data)
if ismember('rpm_prop_1', T.Properties.VariableNames)
    rpm_prop_1 = to_num(T.rpm_prop_1);
else
    rpm_prop_1 = to_num(T{:,11});
end

if ismember('rpm_prop_2', T.Properties.VariableNames)
    rpm_prop_2 = to_num(T.rpm_prop_2);
else
    rpm_prop_2 = to_num(T{:,12});
end

% Rudder Servo (Kolom 5 & 6 pada data)
if ismember('Calc_deg_servo_1', T.Properties.VariableNames) && ismember('Calc_deg_servo_2', T.Properties.VariableNames)
    delta1_deg = to_num(T.Calc_deg_servo_1);
    delta2_deg = to_num(T.Calc_deg_servo_2);
elseif ismember('Calc_deg_servo_1', T.Properties.VariableNames)
    delta1_deg = to_num(T.Calc_deg_servo_1);
    delta2_deg = delta1_deg;
else
    delta1_deg = to_num(T{:,5});
    delta2_deg = to_num(T{:,6});
end

delta_deg = (delta1_deg + delta2_deg) / 2;

%% CONSTANT WGS84
a  = 6378137;
f  = 1/298.257223563;
e2 = f*(2-f);

%% REF POINT (ENU ORIGIN)
lat0 = deg2rad(lat_deg(1));
lon0 = deg2rad(lon_deg(1));

%% LLA → ECEF
lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

N = a ./ sqrt(1 - e2*sin(lat).^2);

X = N .* cos(lat) .* cos(lon);
Y = N .* cos(lat) .* sin(lon);
Z = (N*(1-e2)) .* sin(lat);

%% SHIFT ORIGIN
dX = X - X(1);
dY = Y - Y(1);
dZ = Z - Z(1);

%% ECEF → ENU
R = [ -sin(lon0),              cos(lon0),             0;
      -sin(lat0)*cos(lon0),   -sin(lat0)*sin(lon0),  cos(lat0);
       cos(lat0)*cos(lon0),    cos(lat0)*sin(lon0),  sin(lat0) ];

ENU = R * [dX dY dZ]';

x = ENU(1,:)';   % East
y = ENU(2,:)';   % North
z = ENU(3,:)';   % Up

%% HEADING
psi = unwrap(deg2rad(yaw_deg));

%% RUDDER
delta = deg2rad(delta_deg);

%% PROPELLER
n_P = rpm_prop_1 / 60;
n_S = rpm_prop_2 / 60;

%% TIME STEP
dt = diff(t_raw);
dt_valid = dt(dt > 0 & ~isnan(dt));
if isempty(dt_valid)
    mean_dt = 0.1;
else
    mean_dt = mean(dt_valid);
end
dt(dt <= 0 | isnan(dt)) = mean_dt;

%% VELOCITY (ENU FRAME)
vx = [0; diff(x)./dt];
vy = [0; diff(y)./dt];

%% BODY-FIXED VELOCITY
u =  cos(psi).*vx + sin(psi).*vy;
v = -sin(psi).*vx + cos(psi).*vy;

%% YAW RATE
r = [0; diff(psi)./dt];

%% FORCE COLUMN VECTOR
u = u(:); v = v(:); r = r(:);
x = x(:); y = y(:); psi = psi(:);
delta = delta(:); n_P = n_P(:);

%% STATE VECTOR
state = [v r x y psi delta n_P];

%% TABLE OUTPUT
T_out = array2table(state, ...
    'VariableNames', ...
    {'v','r','x_ENU','y_ENU','psi','delta','n_P'});

%% SAVE TO EXCEL
writetable(T_out, 'hasil_preprocessing_ENU.xlsx');

disp('✔ BERHASIL: DATA ENU DISIMPAN KE EXCEL');

%% =====================================
% LOCAL FUNCTION
% =====================================
function out = to_double_vector(in)
    if isnumeric(in)
        out = double(in);
    elseif iscell(in) || isstring(in) || ischar(in)
        out = str2double(in);
    else
        out = double(in);
    end
end