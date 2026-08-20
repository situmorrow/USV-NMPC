%% =====================================
% PREPROCESSING GPS → ENU + EXPORT EXCEL
% =====================================
clear; clc;

%% LOAD DATA
filename = 'lurus_konstan.xlsx';
T = readtable(filename);

% Helper konversi ke numerik
to_num = @(x) to_double_vector(x);

t_raw   = to_num(T{:,1}); % timestamp
lat_deg = to_num(T{:,2}); % lat
lon_deg = to_num(T{:,3}); % lon
yaw_deg = to_num(T{:,9}); % heading
delta_deg = (to_num(T{:,5}) + to_num(T{:,6}))/2; % delta (sudut rudder)
u_0     = to_num(T{:,4}); % kecepatan speedMps

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
psi = unwrap(deg2rad(yaw_deg + 180));

% RUDDER
delta = deg2rad(delta_deg);

% VELOCITY
u_exp = repmat(mean(u_0(u_0 > 0 & ~isnan(u_0))), length(x), 1);

% TIME STEP
dt = diff(t_raw);
dt(dt <= 0 | isnan(dt)) = mean(dt(dt > 0 & ~isnan(dt)));

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
delta = delta(:); u_exp = u_exp(:);

% STATE VECTOR
state = [v r x y psi delta u_exp];

%% TABLE OUTPUT
T_out = array2table(state, ...
    'VariableNames', ...
    {'v','r','x_ENU','y_ENU','psi','delta','u_exp'});

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