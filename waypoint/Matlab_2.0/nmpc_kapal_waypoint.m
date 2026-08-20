function [u_opt, exitflag] = nmpc_kapal_waypoint(current_state_nd, u_prev, u_0, x_ref_seq, y_ref_seq, psi_ref_seq)
%#codegen
% NMPC_KAPAL_WAYPOINT - Menghitung 1 langkah sinyal kontrol NMPC untuk Waypoint Tracking
% Menggunakan Model Matematika Kapal WyNDA (11 Parameter Basis)
%
% Inputs:
%   current_state_nd : [5 x 1] state kapal [v'; r'; x'; y'; psi'] (nondimensional)
%   u_prev           : [1 x 1] sudut kemudi sebelumnya (radian)
%   u_0              : [1 x 1] kecepatan surge
%   x_ref_seq        : [N x 1] posisi X referensi horizon (nondimensional)
%   y_ref_seq        : [N x 1] posisi Y referensi horizon (nondimensional)
%   psi_ref_seq      : [N x 1] heading referensi horizon (radian)
% Outputs:
%   u_opt            : [1 x 1] perintah sudut kemudi optimal (radian)
%   exitflag         : status konvergensi fmincon (>0: sukses)

%% 1. Parameter Model Kapal WyNDA (11 Basis Parameter)
L = 1.0107;         % Panjang kapal model [meter]
u_0_exp = u_0;      % Kecepatan surge nominal [m/s]

% Vektor Parameter Theta Hasil Identifikasi WyNDA (11 Basis):
% v_dot   = theta_1*v + theta_2*r + theta_3*delta
% r_dot   = theta_4*v + theta_5*r + theta_6*delta
% x_dot   = theta_7*u0*cos(psi) - theta_8*v*sin(psi)
% y_dot   = theta_9*u0*sin(psi) + theta_10*v*cos(psi)
% psi_dot = theta_11*r
theta = [
   -9.2816e-01;  % theta_1  : v pada v_dot
   -2.6644e-01;  % theta_2  : r pada v_dot
    1.2074e-01;  % theta_3  : delta pada v_dot
    2.6348e-03;  % theta_4  : v pada r_dot
   -1.0577e-02;  % theta_5  : r pada r_dot
   -1.3502e-02;  % theta_6  : delta pada r_dot
    5.8118e-02;  % theta_7  : u0*cos(psi)
    1.4903e-03;  % theta_8  : v*sin(psi)
    4.7426e-02;  % theta_9  : u0*sin(psi)
   -4.6814e-03;  % theta_10 : v*cos(psi)
    4.5806e-02   % theta_11 : r pada psi_dot
];

%% 2. Setup Horizon & Kendala Operasional (Constraints)
N = length(x_ref_seq);

% Bobot Matriks Biaya (Cost Function)
Q = diag([10, 10, 10]);       % Bobot tracking [x_err, y_err, psi_err]
R = 1;                        % Bobot penalti besaran kemudi

% Batasan Sudut Rudder (+/- 45 derajat)
u_limit = deg2rad(45.0);
lb = -u_limit * ones(N, 1);
ub =  u_limit * ones(N, 1);

% Batasan Laju Perubahan Kemudi (+/- 30 derajat per step)
du_max = deg2rad(30.0);
[A_du, b_du] = du_constraints(N, u_prev, du_max);

% Batasan Yaw Rate Kapal (+/- 45 deg/s)
r_limit = deg2rad(45.0);
r_limit_nd = r_limit * (L / u_0_exp);

%% 3. Konfigurasi Optimization Solver (fmincon SQP)
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none', ...
    'MaxIterations', 60, 'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-4);

cost_fun = @(U) mpc_cost(U, current_state_nd, x_ref_seq, y_ref_seq, psi_ref_seq, theta, Q, R);
nonlcon  = @(U) state_constraints(U, current_state_nd, theta, r_limit_nd);

U0 = u_prev * ones(N, 1);

%% 4. Eksekusi Solver NMPC 1-Step
[U_opt, ~, exitflag] = fmincon(cost_fun, U0, A_du, b_du, [], [], lb, ub, nonlcon, options);

if exitflag <= 0
    u_opt = u_prev;
else
    u_opt = U_opt(1);
end

end

%% =========================================================================
% FUNGSI-FUNGSI HELPER MODEL WYNDA & KENDALI NMPC
% =========================================================================

function s_next = euler_step(s, u, theta)
    Phi = compute_basis_phi_11(s, u);
    s_next = s + Phi * theta;
end

function Phi = compute_basis_phi_11(s, delta)
    v   = s(1);
    r   = s(2);
    psi = s(5);
    
    Phi = [
        v,  r,  delta, zeros(1,8);
        zeros(1,3), v,  r,  delta, zeros(1,5);
        zeros(1,6), cos(psi), -v*sin(psi), zeros(1,3);
        zeros(1,8), sin(psi),  v*cos(psi), zeros(1,1);
        zeros(1,10), r
    ];
end

function J = mpc_cost(U, s0, x_ref_seq, y_ref_seq, psi_ref_seq, theta, Q, R)
    N = length(U);
    s = s0;
    J = 0;
    for i = 1:N
        u = U(i);
        s = euler_step(s, u, theta);
        
        % Kalibrasi error heading shortest-angular distance
        psi_err = atan2(sin(s(5) - psi_ref_seq(i)), cos(s(5) - psi_ref_seq(i)));
        
        err = [s(3) - x_ref_seq(i);
               s(4) - y_ref_seq(i);
               psi_err];

        J = J + err' * Q * err + R * (u^2);
    end
end

function [c, ceq] = state_constraints(U, s0, theta, r_limit_nd)
    N = length(U);
    s = s0;
    c = zeros(2*N, 1);
    idx = 1;
    for i = 1:N
        s = euler_step(s, U(i), theta);
        r = s(2);
        c(idx)   =  r - r_limit_nd;
        c(idx+1) = -r_limit_nd - r;
        idx = idx + 2;
    end
    ceq = [];
end

function [A, b] = du_constraints(N, u_prev, du_max)
    A = zeros(2*N, N);
    b = zeros(2*N, 1);
    
    A(1,1) = 1;   b(1) = u_prev + du_max;
    for i = 2:N
        A(i, i-1) = -1;   A(i, i) = 1;
        b(i) = du_max;
    end
    
    A(N+1,1) = -1;   b(N+1) = du_max - u_prev;
    for i = 2:N
        A(N+i, i-1) = 1;   A(N+i, i) = -1;
        b(N+i) = du_max;
    end
end