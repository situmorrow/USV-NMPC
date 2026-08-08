//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlarfg.cpp
//
// Code generation for function 'xzlarfg'
//

// Include files
#include "xzlarfg.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xzlarfg(int n, double &alpha1, double x[90601], int ix0)
{
  double tau;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = blas::xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = std::abs(alpha1);
      xnorm = std::abs(xnorm);
      if (beta1 < xnorm) {
        beta1 /= xnorm;
        beta1 = xnorm * std::sqrt(beta1 * beta1 + 1.0);
      } else if (beta1 > xnorm) {
        xnorm /= beta1;
        beta1 *= std::sqrt(xnorm * xnorm + 1.0);
      } else if (std::isnan(xnorm)) {
        beta1 = rtNaN;
      } else {
        beta1 *= 1.4142135623730951;
      }
      if (alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        int i;
        int knt;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          for (int k{ix0}; k <= i; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        xnorm = std::abs(alpha1);
        beta1 = std::abs(blas::xnrm2(n - 1, x, ix0));
        if (xnorm < beta1) {
          xnorm /= beta1;
          beta1 *= std::sqrt(xnorm * xnorm + 1.0);
        } else if (xnorm > beta1) {
          beta1 /= xnorm;
          beta1 = xnorm * std::sqrt(beta1 * beta1 + 1.0);
        } else if (std::isnan(beta1)) {
          beta1 = rtNaN;
        } else {
          beta1 = xnorm * 1.4142135623730951;
        }
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        for (int k{ix0}; k <= i; k++) {
          x[k - 1] *= xnorm;
        }
        for (int k{0}; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        alpha1 = beta1;
      } else {
        int i;
        tau = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        i = (ix0 + n) - 2;
        for (int k{ix0}; k <= i; k++) {
          x[k - 1] *= xnorm;
        }
        alpha1 = beta1;
      }
    }
  }
  return tau;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlarfg.cpp)
