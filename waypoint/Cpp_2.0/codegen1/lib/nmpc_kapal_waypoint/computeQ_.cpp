//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeQ_.cpp
//
// Code generation for function 'computeQ_'
//

// Include files
#include "computeQ_.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void computeQ_(c_struct_T &obj, int nrows)
{
  double work[201];
  int iQR0;
  int k;
  int m;
  int n;
  k = obj.minRowCol;
  for (int idx{0}; idx < k; idx++) {
    iQR0 = 201 * idx + idx;
    n = obj.mrows - idx;
    if (n - 2 >= 0) {
      std::copy(&obj.QR[iQR0 + 1], &obj.QR[iQR0 + n], &obj.Q[iQR0 + 1]);
    }
  }
  m = obj.mrows;
  if (nrows >= 1) {
    int itau;
    for (int idx{k}; idx < nrows; idx++) {
      iQR0 = idx * 201;
      std::memset(&obj.Q[iQR0], 0,
                  static_cast<unsigned int>(m) * sizeof(double));
      obj.Q[iQR0 + idx] = 1.0;
    }
    itau = obj.minRowCol - 1;
    std::memset(&work[0], 0, 201U * sizeof(double));
    for (int i{obj.minRowCol}; i >= 1; i--) {
      int iaii;
      iaii = i + (i - 1) * 201;
      if (i < nrows) {
        int lastc;
        int lastv;
        obj.Q[iaii - 1] = 1.0;
        k = iaii + 201;
        if (obj.tau[itau] != 0.0) {
          bool exitg2;
          lastv = m - i;
          iQR0 = (iaii + m) - i;
          while ((lastv + 1 > 0) && (obj.Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }
          lastc = (nrows - i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            iQR0 = (iaii + lastc * 201) + 201;
            n = iQR0;
            do {
              exitg1 = 0;
              if (n <= iQR0 + lastv) {
                if (obj.Q[n - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  n++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }
        if (lastv + 1 > 0) {
          double c;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              std::memset(&work[0], 0,
                          static_cast<unsigned int>(lastc + 1) *
                              sizeof(double));
            }
            n = (iaii + 201 * lastc) + 201;
            for (int iac{k}; iac <= n; iac += 201) {
              c = 0.0;
              iQR0 = iac + lastv;
              for (int idx{iac}; idx <= iQR0; idx++) {
                c += obj.Q[idx - 1] * obj.Q[((iaii + idx) - iac) - 1];
              }
              iQR0 = div_nzp_s32_floor((iac - iaii) - 201, 201);
              work[iQR0] += c;
            }
          }
          if (!(-obj.tau[itau] == 0.0)) {
            iQR0 = iaii;
            for (int idx{0}; idx <= lastc; idx++) {
              c = work[idx];
              if (c != 0.0) {
                c *= -obj.tau[itau];
                n = iQR0 + 201;
                k = lastv + iQR0;
                for (int iac{n}; iac <= k + 201; iac++) {
                  obj.Q[iac - 1] += obj.Q[((iaii + iac) - iQR0) - 202] * c;
                }
              }
              iQR0 += 201;
            }
          }
        }
      }
      if (i < m) {
        iQR0 = iaii + 1;
        n = (iaii + m) - i;
        for (int idx{iQR0}; idx <= n; idx++) {
          obj.Q[idx - 1] *= -obj.tau[itau];
        }
      }
      obj.Q[iaii - 1] = 1.0 - obj.tau[itau];
      for (int idx{0}; idx <= i - 2; idx++) {
        obj.Q[(iaii - idx) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeQ_.cpp)
