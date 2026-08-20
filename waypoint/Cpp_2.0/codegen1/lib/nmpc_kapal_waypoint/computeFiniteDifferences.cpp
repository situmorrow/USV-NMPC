//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFiniteDifferences.cpp
//
// Code generation for function 'computeFiniteDifferences'
//

// Include files
#include "computeFiniteDifferences.h"
#include "anonymous_function.h"
#include "finDiffEvalAndChkErr.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
bool computeFiniteDifferences(l_struct_T &obj, double fCurrent,
                              const double cIneqCurrent[80], double xk[20],
                              double gradf[101], double JacCineqTrans[8080])
{
  int idx;
  bool evalOK;
  bool exitg1;
  evalOK = true;
  obj.numEvals = 0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx < 20)) {
    double deltaX;
    double deltaX_tmp;
    bool guard1;
    bool modifiedStep;
    deltaX_tmp = 1.4901161193847656E-8 *
                 (1.0 - 2.0 * static_cast<double>(xk[idx] < 0.0)) *
                 std::fmax(std::abs(xk[idx]), 1.0);
    modifiedStep = false;
    if ((xk[idx] >= -0.78539816339744828) && (xk[idx] <= 0.78539816339744828)) {
      deltaX = xk[idx] + deltaX_tmp;
      if ((deltaX > 0.78539816339744828) || (deltaX < -0.78539816339744828)) {
        deltaX_tmp = -deltaX_tmp;
        modifiedStep = true;
        deltaX = xk[idx] + deltaX_tmp;
        if ((deltaX > 0.78539816339744828) || (deltaX < -0.78539816339744828)) {
          deltaX = xk[idx] - -0.78539816339744828;
          deltaX_tmp = 0.78539816339744828 - xk[idx];
          if (deltaX <= deltaX_tmp) {
            deltaX_tmp = -deltaX;
          }
        }
      }
    }
    deltaX = deltaX_tmp;
    evalOK = internal::finDiffEvalAndChkErr(obj.objfun, obj.nonlin, obj.cIneq_1,
                                            idx + 1, deltaX_tmp, xk, obj.f_1);
    obj.numEvals++;
    guard1 = false;
    if (!evalOK) {
      if (!modifiedStep) {
        double d;
        deltaX = -deltaX_tmp;
        d = xk[idx] - deltaX_tmp;
        if ((d >= -0.78539816339744828) && (d <= 0.78539816339744828)) {
          modifiedStep = true;
        } else {
          modifiedStep = false;
        }
        if ((!obj.hasBounds) || modifiedStep) {
          evalOK = internal::finDiffEvalAndChkErr(obj.objfun, obj.nonlin,
                                                  obj.cIneq_1, idx + 1,
                                                  -deltaX_tmp, xk, obj.f_1);
          obj.numEvals++;
        }
      }
      if (!evalOK) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      gradf[idx] = (obj.f_1 - fCurrent) / deltaX;
      for (int idx_row{0}; idx_row < 40; idx_row++) {
        JacCineqTrans[idx + 101 * (idx_row + 40)] =
            (obj.cIneq_1[idx_row] - cIneqCurrent[idx_row + 40]) / deltaX;
      }
      idx++;
    }
  }
  return evalOK;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeFiniteDifferences.cpp)
