function [u_opt, exitflag] = nmpc_kapal_waypoint_danau8its(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq)
%#codegen
% NMPC_KAPAL_WAYPOINT_DANAU8ITS - Menghitung 1 langkah sinyal kontrol NMPC untuk Waypoint Tracking USV RC di Danau 8 ITS
% Inputs:
%   current_state_nd : [5 x 1] state kapal non-dimensional [v; r; x; y; psi] (skala L=1.2 m, u_0=1.08 m/s)
%   u_prev           : [1 x 1] sudut kemudi sebelumnya (radian)
%   x_ref_seq        : [N x 1] posisi X referensi horizon (non-dimensional x / 1.2)
%   y_ref_seq        : [N x 1] posisi Y referensi horizon (non-dimensional y / 1.2)
%   psi_ref_seq      : [N x 1] heading referensi horizon (radian)
% Outputs:
%   u_opt            : [1 x 1] perintah sudut kemudi optimal (radian)
%   exitflag         : status konvergensi fmincon (>0: sukses)

%% 1. Parameter Utama Kapal USV RC (Spesifikasi Fisik Danau 8 ITS)
L = 1.2;            % Panjang kapal USV (m)
B = 0.35;           % Lebar kapal USV (m)
T = 0.10;           % Draft kapal USV (m)
m = 8.5;            % Massa kapal USV (kg)
u_0 = 1.08;         % Kecepatan surge kapal (m/s)
C_B = 0.65;         % Koefisien Blok
x_G = 0.05;         % Pusat massa sumbu-x (m)
rho = 1000;         % Massa jenis air tawar danau (kg/m^3)

%% 2. Koefisien Hidrodinamika & Matriks Sistem
Y_v_dot = -(1 + 0.16*C_B*B/T - 5.1*(B/L)^2)*pi*(T/L)^2;
Y_r_dot = -(0.67*(B/L) - 0.0033*(B/T)^2)*pi*(T/L)^2;
N_v_dot = -(1.1*B/L - 0.041*B/T)*pi*(T/L)^2;
N_r_dot = -((1/12) + 0.017*C_B*B/T - 0.33*B/L)*pi*(T/L)^2;
Y_v = -(1 + 0.4*C_B*B/T)*pi*(T/L)^2;
Y_r = -(-0.5 + 2.2*B/L - 0.08*B/T)*pi*(T/L)^2;
N_v = -(0.5 + 2.4*T/L)*pi*(T/L)^2;
N_r = -(0.25 + 0.039*B/T - 0.56*B/L)*pi*(T/L)^2;

m_nd   = 2*m/(rho*L^3);
x_G_nd = x_G/L;
I_z_nd = 1.2392*10^(-4);
u_0_nd = 1;

M_mat = [m_nd - Y_v_dot , m_nd*x_G_nd - Y_r_dot ;
         m_nd*x_G_nd - N_v_dot , I_z_nd - N_r_dot ];

a11 = ((I_z_nd - N_r_dot)*Y_v - (m_nd*x_G_nd - Y_r_dot)*N_v)/det(M_mat);
a12 = ((I_z_nd - N_r_dot)*(Y_r - m_nd*u_0_nd) - (m_nd*x_G_nd - Y_r_dot)*(N_r - m_nd*x_G_nd*u_0_nd))/det(M_mat);
a21 = ((m_nd - Y_v_dot)*N_v - (m_nd*x_G_nd - N_v_dot)*Y_v)/det(M_mat);
a22 = ((m_nd - Y_v_dot)*(N_r - m_nd*x_G_nd*u_0_nd) - (m_nd*x_G_nd - N_v_dot)*(Y_r - m_nd*u_0_nd))/det(M_mat);

A_sys = [a11 , a12 ; a21 , a22 ];
B_sys = [0.01 ; 1];

%% 3. Setup Horizon & Kendala Operasional USV RC
T_sim = 0.1;        % Sampling time (10 Hz)
N     = length(x_ref_seq); % Prediksi horizon step (N)

Q = diag([100, 100, 100]);       
R = 0.05;                      

r_limit    = 0.0932;          % 0.0932 rad/s (5.34 deg/s)
r_limit_nd = r_limit * (L / u_0);
u_limit    = deg2rad(35.0);   % 35 deg
du_max     = deg2rad(5.0);    % Perubahan sudut kemudi maks (5 deg/step)

% Generasi Matriks Kendala Rate Offsets (A_du * U <= b_du)
[A_du, b_du] = du_constraints(N, u_prev, du_max); 

lb = -u_limit * ones(N,1);
ub =  u_limit * ones(N,1);

%% 4. Konfigurasi Optimization Solver (fmincon SQP)
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', ...
                       'MaxIterations', 30, 'OptimalityTolerance', 1e-3, 'ConstraintTolerance', 1e-3);

cost_fun = @(U) mpc_cost(U, current_state_nd, x_ref_seq, y_ref_seq, psi_ref_seq, T_sim, L, u_0, A_sys, B_sys, u_0_nd, Q, R);
nonlcon  = @(U) state_constraints(U, current_state_nd, T_sim, L, u_0, A_sys, B_sys, u_0_nd, r_limit_nd);

U0 = u_prev * ones(N,1);

%% 5. Eksekusi Solver NMPC 1-Step
[U_opt, ~, exitflag] = fmincon(cost_fun, U0, A_du, b_du, [], [], lb, ub, nonlcon, options);

if exitflag <= 0
    u_opt = u_prev;
else
    u_opt = U_opt(1);
end

end

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

function J = mpc_cost(U, s0, x_ref_seq, y_ref_seq, psi_ref_seq, T_sim, L, u0, A_sys, B_sys, u0_nd, Q, R)
    N = length(U); 
    s = s0;
    dt_nd = T_sim * u0 / L;
    J = 0; 
    for i = 1:N
        u = U(i);
        s = euler_step(@(s,u) ship_dynamics(s, u, A_sys, B_sys, u0_nd), s, u, dt_nd);
        
        psi_err = atan2(sin(s(5) - psi_ref_seq(i)), cos(s(5) - psi_ref_seq(i)));
        
        err = [s(3) - x_ref_seq(i);
               s(4) - y_ref_seq(i);
               psi_err];
        J = J + err' * Q * err + R * u^2;
    end
end

function [c, ceq] = state_constraints(U, s0, T_sim, L, u0, A_sys, B_sys, u0_nd, r_limit_nd)
    N = length(U);
    s = s0;
    dt_nd = T_sim * u0 / L;
    c = zeros(2*N, 1); 
    idx = 1;
    for i = 1:N
        s = euler_step(@(s,u) ship_dynamics(s, u, A_sys, B_sys, u0_nd), s, U(i), dt_nd);
        r = s(2);    
        c(idx) = r - r_limit_nd;
        c(idx+1) = -r_limit_nd - r;
        idx = idx + 2;
    end
    ceq = [];
end

function [A, b] = du_constraints(N, u_prev, du_max)
    A = zeros(2*N, N);
    b = zeros(2*N, 1);
    A(1,1) = 1; b(1) = u_prev + du_max;
    for i = 2:N
        A(i, i-1) = -1; A(i, i) = 1;
        b(i) = du_max;
    end
    A(N+1,1) = -1; b(N+1) = du_max - u_prev;
    for i = 2:N
        A(N+i, i-1) = 1; A(N+i, i) = -1;
        b(N+i) = du_max;
    end
end
