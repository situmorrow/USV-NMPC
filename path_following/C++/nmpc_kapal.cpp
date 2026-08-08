//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal.cpp
//
// Code generation for function 'nmpc_kapal'
//

// Include files
#include "nmpc_kapal.h"
#include "anonymous_function.h"
#include "fmincon.h"
#include "nmpc_kapal_internal_types1.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
void nmpc_kapal(const double current_state_nd[5], double u_prev,
                const double x_ref_seq[30], const double y_ref_seq[30],
                const double psi_ref_seq[30], double *u_opt, double *exitflag)
{
  coder::anonymous_function cost_fun;
  coder::b_anonymous_function nonlcon;
  double A_du[1800];
  double b_du[60];
  double U_opt[30];
  double b_u_prev[30];
  double b_exitflag;
  //  NMPC_KAPAL - Menghitung 1 langkah sinyal kontrol NMPC
  //  Inputs:
  //    current_state_nd : [5 x 1] state kapal non-dimensional [v; r; x; y; psi]
  //    u_prev           : [1 x 1] sudut kemudi sebelumnya (radian)
  //    x_ref_seq        : [N x 1] posisi X referensi horizon (non-dimensional)
  //    y_ref_seq        : [N x 1] posisi Y referensi horizon (non-dimensional)
  //    psi_ref_seq      : [N x 1] heading referensi horizon (radian)
  //  Outputs:
  //    u_opt            : [1 x 1] perintah sudut kemudi optimal (radian)
  //    exitflag         : status konvergensi fmincon (>0: sukses)
  //  1. Parameter Utama Kapal & Kontrol
  //  2. Koefisien Hidrodinamika & Matriks Sistem
  //  3. Setup Horizon & Kendala Operasional
  //  Generasi Matriks Kendala Rate Offsets (A_du * U <= b_du)
  std::memset(&A_du[0], 0, 1800U * sizeof(double));
  std::memset(&b_du[0], 0, 60U * sizeof(double));
  A_du[0] = 1.0;
  b_du[0] = u_prev + 0.087266462599716474;
  b_du[30] = 0.087266462599716474 - u_prev;
  for (int i{0}; i < 29; i++) {
    int A_du_tmp;
    int b_A_du_tmp;
    A_du_tmp = i + 60 * i;
    A_du[A_du_tmp + 1] = -1.0;
    b_A_du_tmp = i + 60 * (i + 1);
    A_du[b_A_du_tmp + 1] = 1.0;
    b_du[i + 1] = 0.087266462599716474;
    A_du[A_du_tmp + 31] = 1.0;
    A_du[b_A_du_tmp + 31] = -1.0;
    b_du[i + 31] = 0.087266462599716474;
  }
  A_du[30] = -1.0;
  //  4. Konfigurasi Optimization Solver (fmincon SQP)
  for (int i{0}; i < 5; i++) {
    cost_fun.workspace.current_state_nd[i] = current_state_nd[i];
  }
  std::copy(&x_ref_seq[0], &x_ref_seq[30], &cost_fun.workspace.x_ref_seq[0]);
  std::copy(&y_ref_seq[0], &y_ref_seq[30], &cost_fun.workspace.y_ref_seq[0]);
  std::copy(&psi_ref_seq[0], &psi_ref_seq[30],
            &cost_fun.workspace.psi_ref_seq[0]);
  for (int i{0}; i < 5; i++) {
    nonlcon.workspace.current_state_nd[i] = current_state_nd[i];
  }
  //  5. Eksekusi Solver NMPC 1-Step
  for (int i{0}; i < 30; i++) {
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

double nmpc_kapal_anonFcn1(const double current_state_nd[5],
                           const double x_ref_seq[30],
                           const double y_ref_seq[30],
                           const double psi_ref_seq[30], const double U[30])
{
  static const signed char iv[9]{10, 0, 0, 0, 10, 0, 0, 0, 10};
  double s[5];
  double b_err[3];
  double err[3];
  double varargout_1;
  for (int i{0}; i < 5; i++) {
    s[i] = current_state_nd[i];
  }
  varargout_1 = 0.0;
  for (int i{0}; i < 30; i++) {
    double b_s;
    double b_x_dot_tmp;
    double c_s;
    double d;
    double d1;
    double d_s;
    double e_s;
    double f_s;
    double x_dot_tmp;
    b_s = s[0];
    c_s = s[1];
    d = U[i];
    d_s = s[4];
    x_dot_tmp = std::sin(s[4]);
    b_x_dot_tmp = std::cos(s[4]);
    e_s = s[2];
    f_s = s[3];
    s[0] = b_s + 0.15236964480063322 *
                     ((-0.61373138167832486 * b_s + -0.1017805438031183 * c_s) +
                      0.01 * d);
    s[1] =
        c_s + 0.15236964480063322 *
                  ((-5.0966239122212551 * b_s + -3.4085423899828422 * c_s) + d);
    d1 = e_s + 0.15236964480063322 * (b_x_dot_tmp - b_s * x_dot_tmp);
    s[2] = d1;
    b_s = f_s + 0.15236964480063322 * (x_dot_tmp + b_s * b_x_dot_tmp);
    s[3] = b_s;
    e_s = d_s + 0.15236964480063322 * c_s;
    s[4] = e_s;
    err[0] = d1 - x_ref_seq[i];
    err[1] = b_s - y_ref_seq[i];
    err[2] = e_s - psi_ref_seq[i];
    std::memset(&b_err[0], 0, 3U * sizeof(double));
    e_s = 0.0;
    for (int b_i{0}; b_i < 3; b_i++) {
      b_s = ((b_err[b_i] + err[0] * static_cast<double>(iv[3 * b_i])) +
             err[1] * static_cast<double>(iv[3 * b_i + 1])) +
            err[2] * static_cast<double>(iv[3 * b_i + 2]);
      b_err[b_i] = b_s;
      e_s += b_s * err[b_i];
    }
    varargout_1 = (varargout_1 + e_s) + d * d;
  }
  return varargout_1;
}

void nmpc_kapal_anonFcn2(const double current_state_nd[5], const double U[30],
                         double varargout_1[60])
{
  double s[5];
  for (int i{0}; i < 5; i++) {
    s[i] = current_state_nd[i];
  }
  std::memset(&varargout_1[0], 0, 60U * sizeof(double));
  for (int i{0}; i < 30; i++) {
    double b_s;
    double c_s;
    double d;
    int varargout_1_tmp;
    b_s = s[0];
    c_s = s[1];
    d = U[i];
    s[0] = b_s + 0.15236964480063322 *
                     ((-0.61373138167832486 * b_s + -0.1017805438031183 * c_s) +
                      0.01 * d);
    b_s =
        c_s + 0.15236964480063322 *
                  ((-5.0966239122212551 * b_s + -3.4085423899828422 * c_s) + d);
    s[1] = b_s;
    varargout_1_tmp = i << 1;
    varargout_1[varargout_1_tmp] = b_s - 0.61167038961038955;
    varargout_1[varargout_1_tmp + 1] = -0.61167038961038955 - b_s;
  }
}

// End of code generation (nmpc_kapal.cpp)
