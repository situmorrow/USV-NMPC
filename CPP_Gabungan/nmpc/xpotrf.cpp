//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xpotrf.cpp
//
// Code generation for function 'xpotrf'
//

// Include files
#include "xpotrf.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
int xpotrf(int n, double A[40401])
{
  int info;
  int j;
  bool exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    idxA1j = j * 201;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (int k{0}; k < j; k++) {
        c = A[idxA1j + k];
        ssq += c * c;
      }
    }
    ssq = A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A[idxAjj] = ssq;
      if (j + 1 < n) {
        int i1;
        int ia0;
        int idxAjjp1;
        int nmj;
        nmj = (n - j) - 2;
        ia0 = idxA1j + 202;
        idxAjjp1 = idxAjj + 202;
        if ((j != 0) && (nmj + 1 != 0)) {
          int i;
          i = (idxA1j + 201 * nmj) + 202;
          for (int iac{ia0}; iac <= i; iac += 201) {
            c = 0.0;
            i1 = iac + j;
            for (int k{iac}; k < i1; k++) {
              c += A[k - 1] * A[(idxA1j + k) - iac];
            }
            i1 = (idxAjj + div_nzp_s32_floor((iac - idxA1j) - 202, 201) * 201) +
                 201;
            A[i1] -= c;
          }
        }
        ssq = 1.0 / ssq;
        i1 = (idxAjj + 201 * nmj) + 202;
        for (int k{idxAjjp1}; k <= i1; k += 201) {
          A[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xpotrf.cpp)
