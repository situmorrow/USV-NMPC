function [u_opt, exitflag] = nmpc_kapal_waypoint(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq)
%#codegen
% NMPC_KAPAL_WAYPOINT - Menghitung 1 langkah sinyal kontrol NMPC untuk Waypoint Tracking
% Inputs:
%   current_state_nd : [5 x 1] state kapal [v; r; x; y; psi]
%   u_prev           : [1 x 1] sudut kemudi sebelumnya (radian)
%   x_ref_seq        : [N x 1] posisi X referensi horizon (non-dimensional)
%   y_ref_seq        : [N x 1] posisi Y referensi horizon (non-dimensional)
%   psi_ref_seq      : [N x 1] heading referensi horizon (radian)
% Outputs:
%   u_opt            : [1 x 1] perintah sudut kemudi optimal (radian)
%   exitflag         : status konvergensi fmincon (>0: sukses)

% Terdapat beberapa modifikasi model agar sesuai dengan data eksperimen
% 1. Parameter utama kapal diubah ke skala froude
% 2. Model yang digunakan adalah model wynda citra (wynda_1)
% 3. Model yang digunakan adalah model wynda mba ai (wynda_2)
% 4. Model Wynda pakai data yang ada lurus dan melingkar (wynda_3)

%% 1. Parameter Utama Kapal & Kontrol
% L = 101.07;         
% B = 14;             
% T = 3.7;            
% m = 2423*1e3;       
% u_0 = 15.4;         
% C_B = 0.65;         
% x_G = 5.25;         
% rho = 1024;

% Parameter Kapal Eksperimen Penskalaan 1:100
L = 1.0107;         % Panjang kapal (m)
B = 0.14;           % Lebar kapal (m)
T = 0.037;          % Draft kapal (m)
m = 2.423;          % Massa kapal (kg)
u_0 = u_0_exp;      % Kecepatan surge kapal (berdasarkan data eksperimen)
C_B = 0.65;         % Koefisien Blok
x_G = 0.0525;       % Pusat massa sumbu-x
rho = 1000;         % Massa jenis air tawar (kg/m^3)
r = 0.1577;         % Kisaran jari-jari girisa

%% 2. Koefisien Hidrodinamika & Matriks Sistem
Y_v_dot = -(1 + 0.16*C_B*B/T - 5.1*(B/L)^2)*pi*(T/L)^2;
Y_r_dot = -(0.67*(B/L) - 0.0033*(B/T)^2)*pi*(T/L)^2;
N_v_dot = -(1.1*B/L - 0.041*B/T)*pi*(T/L)^2;
N_r_dot = -((1/12) + 0.017*C_B*B/T - 0.33*B/L)*pi*(T/L)^2;
Y_v = -(1 + 0.4*C_B*B/T)*pi*(T/L)^2;
Y_r = -(-0.5 + 2.2*B/L - 0.08*B/T)*pi*(T/L)^2;
N_v = -(0.5 + 2.4*T/L)*pi*(T/L)^2;
N_r = -(0.25 + 0.039*B/T - 0.56*B/L)*pi*(T/L)^2;

m_nd = 2*m/(rho*L^3);
x_G_nd = x_G/L;
I_z_nd = 1.2392*10^(-4);
u_0_nd = 1;

M_mat = [m_nd-Y_v_dot , m_nd*x_G_nd-Y_r_dot ;
         m_nd*x_G_nd - N_v_dot , I_z_nd - N_r_dot ];

a11 = ((I_z_nd - N_r_dot)*Y_v - (m_nd*x_G_nd - Y_r_dot)*N_v)/det(M_mat);
a12 = ((I_z_nd - N_r_dot)*(Y_r - m_nd*u_0_nd) - (m_nd*x_G_nd - Y_r_dot)*(N_r - m_nd*x_G_nd*u_0_nd))/det(M_mat);
a21 = ((m_nd - Y_v_dot)*N_v - (m_nd*x_G_nd - N_v_dot)*Y_v)/det(M_mat);
a22 = ((m_nd - Y_v_dot)*(N_r - m_nd*x_G_nd*u_0_nd) - (m_nd*x_G_nd - N_v_dot)*(Y_r - m_nd*u_0_nd))/det(M_mat);

A_sys = [a11 , a12 ; a21 , a22 ];
B_sys = [0.01 ; 1];

%% 3. Setup Horizon & Kendala Operasional
Tp = 30;            
T_sim = 1.0;        
N = Tp;

Q = diag([10, 10, 10]);       
R = 1;                      

r_limit = 0.0932; 
r_limit_nd = r_limit * (L / u_0);
u_limit = deg2rad(35);
u_rate_limit = deg2rad(5);
du_max = u_rate_limit * T_sim;

% Generasi Matriks Kendala Rate Offsets (A_du * U <= b_du)
[A_du, b_du] = du_constraints(N, u_prev, du_max); 

lb = -u_limit * ones(N,1);
ub =  u_limit * ones(N,1);

%% 4. Konfigurasi Optimization Solver (fmincon SQP)
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', 'MaxIterations', 100, 'OptimalityTolerance', 1e-5);

cost_fun = @(U) mpc_cost(U, current_state_nd, x_ref_seq, y_ref_seq, psi_ref_seq, T_sim, L, u_0, A_sys, B_sys, u_0_nd, Q, R);
nonlcon = @(U) state_constraints(U, current_state_nd, T_sim, L, u_0, A_sys, B_sys, u_0_nd, r_limit_nd);

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

function s_dot = ship_dynamics_wynda_1(s, u, A_sys, B_sys, u0_nd)
    v = s(1); r = s(2); psi = s(5);
    v_r_dot = A_sys * [v; r] + B_sys * u;
    x_dot = u0_nd*cos(psi) - v*sin(psi);
    y_dot = u0_nd*sin(psi) + v*cos(psi);
    s_dot = [v_r_dot(1); v_r_dot(2); x_dot; y_dot; r];
end

function s_dot = ship_dynamics_wynda_2(s, u, A_sys, B_sys, u0_nd)
    v = s(1); r = s(2); psi = s(5);
    v_r_dot = A_sys * [v; r] + B_sys * u;
    x_dot = u0_nd*cos(psi) - v*sin(psi);
    y_dot = u0_nd*sin(psi) + v*cos(psi);
    s_dot = [v_r_dot(1); v_r_dot(2); x_dot; y_dot; r];
end

function s_dot = ship_dynamics_wynda_3(s, u, A_sys, B_sys, u0_nd)
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
        
        % Menghindari pembacaan error sudut yang wrap-around di +-pi
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