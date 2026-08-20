//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal_waypoint.cpp
//
// Code generation for function 'nmpc_kapal_waypoint'
//

// Include files
#include "nmpc_kapal_waypoint.h"
#include "anonymous_function.h"
#include "atan2.h"
#include "fmincon.h"
#include "nmpc_kapal_waypoint_internal_types1.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static const double dv[11]{-0.92816,  -0.26644,   0.12074,  0.0026348,
                           -0.010577, -0.013502,  0.058118, 0.0014903,
                           0.047426,  -0.0046814, 0.045806};

// Function Definitions
void nmpc_kapal_waypoint(const double current_state_nd[5], double u_prev,
                         double u_0, const double x_ref_seq[20],
                         const double y_ref_seq[20],
                         const double psi_ref_seq[20], double *u_opt,
                         double *exitflag)
{
  coder::anonymous_function nonlcon;
  coder::b_anonymous_function cost_fun;
  double A_du[800];
  double b_du[40];
  double U_opt[20];
  double b_u_prev[20];
  double b_exitflag;
  //  NMPC_KAPAL_WAYPOINT - Menghitung 1 langkah sinyal kontrol NMPC untuk
  //  Waypoint Tracking Menggunakan Model Matematika Kapal WyNDA (11 Parameter
  //  Basis)
  //
  //  Inputs:
  //    current_state_nd : [5 x 1] state kapal [v'; r'; x'; y'; psi']
  //    (nondimensional) u_prev           : [1 x 1] sudut kemudi sebelumnya
  //    (radian) u_0              : [1 x 1] kecepatan surge (nondimensional)
  //    x_ref_seq        : [N x 1] posisi X referensi horizon (nondimensional)
  //    y_ref_seq        : [N x 1] posisi Y referensi horizon (nondimensional)
  //    psi_ref_seq      : [N x 1] heading referensi horizon (radian)
  //  Outputs:
  //    u_opt            : [1 x 1] perintah sudut kemudi optimal (radian)
  //    exitflag         : status konvergensi fmincon (>0: sukses)
  //  1. Parameter Model Kapal WyNDA (11 Basis Parameter)
  //  Panjang kapal model [meter]
  //  Kecepatan surge nominal [m/s]
  //  Vektor Parameter Theta Hasil Identifikasi WyNDA (11 Basis):
  //  v_dot   = theta_1*v + theta_2*r + theta_3*delta
  //  r_dot   = theta_4*v + theta_5*r + theta_6*delta
  //  x_dot   = theta_7*u0*cos(psi) - theta_8*v*sin(psi)
  //  y_dot   = theta_9*u0*sin(psi) + theta_10*v*cos(psi)
  //  psi_dot = theta_11*r
  //  theta_1  : v pada v_dot
  //  theta_2  : r pada v_dot
  //  theta_3  : delta pada v_dot
  //  theta_4  : v pada r_dot
  //  theta_5  : r pada r_dot
  //  theta_6  : delta pada r_dot
  //  theta_7  : u0*cos(psi)
  //  theta_8  : v*sin(psi)
  //  theta_9  : u0*sin(psi)
  //  theta_10 : v*cos(psi)
  //  theta_11 : r pada psi_dot
  //  2. Setup Horizon & Kendala Operasional (Constraints)
  //  Bobot Matriks Biaya (Cost Function)
  //  Bobot tracking [x_err, y_err, psi_err]
  //  Bobot penalti besaran kemudi
  //  Batasan Sudut Rudder (+/- 45 derajat)
  //  Batasan Laju Perubahan Kemudi (+/- 30 derajat per step)
  std::memset(&A_du[0], 0, 800U * sizeof(double));
  std::memset(&b_du[0], 0, 40U * sizeof(double));
  A_du[0] = 1.0;
  b_du[0] = u_prev + 0.52359877559829882;
  b_du[20] = 0.52359877559829882 - u_prev;
  for (int i{0}; i < 19; i++) {
    int A_du_tmp;
    int b_A_du_tmp;
    A_du_tmp = i + 40 * i;
    A_du[A_du_tmp + 1] = -1.0;
    b_A_du_tmp = i + 40 * (i + 1);
    A_du[b_A_du_tmp + 1] = 1.0;
    b_du[i + 1] = 0.52359877559829882;
    A_du[A_du_tmp + 21] = 1.0;
    A_du[b_A_du_tmp + 21] = -1.0;
    b_du[i + 21] = 0.52359877559829882;
  }
  A_du[20] = -1.0;
  //  Batasan Yaw Rate Kapal (+/- 45 deg/s)
  nonlcon.workspace.r_limit_nd = 0.78539816339744828 * (1.0107 / u_0);
  //  3. Konfigurasi Optimization Solver (fmincon SQP)
  for (int i{0}; i < 5; i++) {
    cost_fun.workspace.current_state_nd[i] = current_state_nd[i];
  }
  std::copy(&x_ref_seq[0], &x_ref_seq[20], &cost_fun.workspace.x_ref_seq[0]);
  std::copy(&y_ref_seq[0], &y_ref_seq[20], &cost_fun.workspace.y_ref_seq[0]);
  std::copy(&psi_ref_seq[0], &psi_ref_seq[20],
            &cost_fun.workspace.psi_ref_seq[0]);
  for (int i{0}; i < 5; i++) {
    nonlcon.workspace.current_state_nd[i] = current_state_nd[i];
  }
  //  4. Eksekusi Solver NMPC 1-Step
  for (int i{0}; i < 20; i++) {
    b_u_prev[i] = u_prev;
  }
  coder::fmincon(cost_fun, b_u_prev, A_du, b_du, nonlcon, U_opt, b_exitflag);
  *exitflag = b_exitflag;
  if (b_exitflag <= 0.0) {
    *u_opt = u_prev;
  } else {
    *u_opt = U_opt[0];
  }
}

double nmpc_kapal_waypoint_anonFcn1(const double current_state_nd[5],
                                    const double x_ref_seq[20],
                                    const double y_ref_seq[20],
                                    const double psi_ref_seq[20],
                                    const double U[20])
{
  static const signed char iv[9]{10, 0, 0, 0, 10, 0, 0, 0, 10};
  double s[5];
  double b_err[3];
  double varargout_1;
  for (int i{0}; i < 5; i++) {
    s[i] = current_state_nd[i];
  }
  varargout_1 = 0.0;
  for (int b_i{0}; b_i < 20; b_i++) {
    double b_s[55];
    double err[3];
    double Phi_tmp;
    double b_Phi_tmp;
    double d;
    //  =========================================================================
    //  FUNGSI-FUNGSI HELPER MODEL WYNDA & KENDALI NMPC
    //  =========================================================================
    Phi_tmp = std::sin(s[4]);
    b_Phi_tmp = std::cos(s[4]);
    b_s[0] = s[0];
    b_s[5] = s[1];
    d = U[b_i];
    b_s[10] = d;
    for (int i{0}; i < 8; i++) {
      b_s[5 * (i + 3)] = 0.0;
    }
    b_s[1] = 0.0;
    b_s[6] = 0.0;
    b_s[11] = 0.0;
    b_s[16] = s[0];
    b_s[21] = s[1];
    b_s[26] = d;
    for (int i{0}; i < 5; i++) {
      b_s[5 * (i + 6) + 1] = 0.0;
    }
    for (int i{0}; i < 6; i++) {
      b_s[5 * i + 2] = 0.0;
    }
    b_s[32] = b_Phi_tmp;
    b_s[37] = -s[0] * Phi_tmp;
    b_s[42] = 0.0;
    b_s[47] = 0.0;
    b_s[52] = 0.0;
    for (int i{0}; i < 8; i++) {
      b_s[5 * i + 3] = 0.0;
    }
    b_s[43] = Phi_tmp;
    b_s[48] = s[0] * b_Phi_tmp;
    b_s[53] = 0.0;
    for (int i{0}; i < 10; i++) {
      b_s[5 * i + 4] = 0.0;
    }
    b_s[54] = s[1];
    for (int i{0}; i < 5; i++) {
      Phi_tmp = 0.0;
      for (int c_i{0}; c_i < 11; c_i++) {
        Phi_tmp += b_s[i + 5 * c_i] * dv[c_i];
      }
      s[i] += Phi_tmp;
    }
    //  Kalibrasi error heading shortest-angular distance
    err[0] = s[2] - x_ref_seq[b_i];
    err[1] = s[3] - y_ref_seq[b_i];
    Phi_tmp = s[4] - psi_ref_seq[b_i];
    err[2] = coder::b_atan2(std::sin(Phi_tmp), std::cos(Phi_tmp));
    std::memset(&b_err[0], 0, 3U * sizeof(double));
    Phi_tmp = 0.0;
    for (int i{0}; i < 3; i++) {
      b_Phi_tmp = ((b_err[i] + err[0] * static_cast<double>(iv[3 * i])) +
                   err[1] * static_cast<double>(iv[3 * i + 1])) +
                  err[2] * static_cast<double>(iv[3 * i + 2]);
      b_err[i] = b_Phi_tmp;
      Phi_tmp += b_Phi_tmp * err[i];
    }
    varargout_1 = (varargout_1 + Phi_tmp) + d * d;
  }
  return varargout_1;
}

void nmpc_kapal_waypoint_anonFcn2(const double current_state_nd[5],
                                  double r_limit_nd, const double U[20],
                                  double varargout_1[40])
{
  double s[5];
  for (int i{0}; i < 5; i++) {
    s[i] = current_state_nd[i];
  }
  std::memset(&varargout_1[0], 0, 40U * sizeof(double));
  for (int b_i{0}; b_i < 20; b_i++) {
    double b_s[55];
    double Phi_tmp;
    double b_Phi_tmp;
    double d;
    //  =========================================================================
    //  FUNGSI-FUNGSI HELPER MODEL WYNDA & KENDALI NMPC
    //  =========================================================================
    Phi_tmp = std::sin(s[4]);
    b_Phi_tmp = std::cos(s[4]);
    b_s[0] = s[0];
    b_s[5] = s[1];
    d = U[b_i];
    b_s[10] = d;
    for (int i{0}; i < 8; i++) {
      b_s[5 * (i + 3)] = 0.0;
    }
    b_s[1] = 0.0;
    b_s[6] = 0.0;
    b_s[11] = 0.0;
    b_s[16] = s[0];
    b_s[21] = s[1];
    b_s[26] = d;
    for (int i{0}; i < 5; i++) {
      b_s[5 * (i + 6) + 1] = 0.0;
    }
    for (int i{0}; i < 6; i++) {
      b_s[5 * i + 2] = 0.0;
    }
    b_s[32] = b_Phi_tmp;
    b_s[37] = -s[0] * Phi_tmp;
    b_s[42] = 0.0;
    b_s[47] = 0.0;
    b_s[52] = 0.0;
    for (int i{0}; i < 8; i++) {
      b_s[5 * i + 3] = 0.0;
    }
    b_s[43] = Phi_tmp;
    b_s[48] = s[0] * b_Phi_tmp;
    b_s[53] = 0.0;
    for (int i{0}; i < 10; i++) {
      b_s[5 * i + 4] = 0.0;
    }
    b_s[54] = s[1];
    for (int i{0}; i < 5; i++) {
      Phi_tmp = 0.0;
      for (int c_i{0}; c_i < 11; c_i++) {
        Phi_tmp += b_s[i + 5 * c_i] * dv[c_i];
      }
      s[i] += Phi_tmp;
    }
    int varargout_1_tmp;
    varargout_1_tmp = b_i << 1;
    varargout_1[varargout_1_tmp] = s[1] - r_limit_nd;
    varargout_1[varargout_1_tmp + 1] = -r_limit_nd - s[1];
  }
}

// End of code generation (nmpc_kapal_waypoint.cpp)
