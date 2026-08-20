//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// finDiffEvalAndChkErr.cpp
//
// Code generation for function 'finDiffEvalAndChkErr'
//

// Include files
#include "finDiffEvalAndChkErr.h"
#include "anonymous_function.h"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_internal_types1.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
namespace internal {
bool finDiffEvalAndChkErr(const b_anonymous_function &obj_objfun,
                          const anonymous_function obj_nonlin,
                          double cIneqPlus[40], int dim, double delta,
                          double xk[20], double &fplus)
{
  double temp;
  bool evalOK;
  temp = xk[dim - 1];
  xk[dim - 1] = temp + delta;
  fplus = nmpc_kapal_waypoint_anonFcn1(
      obj_objfun.workspace.current_state_nd, obj_objfun.workspace.x_ref_seq,
      obj_objfun.workspace.y_ref_seq, obj_objfun.workspace.psi_ref_seq, xk);
  evalOK = ((!std::isinf(fplus)) && (!std::isnan(fplus)));
  if (evalOK) {
    int idx;
    nmpc_kapal_waypoint_anonFcn2(obj_nonlin.workspace.current_state_nd,
                                 obj_nonlin.workspace.r_limit_nd, xk,
                                 cIneqPlus);
    idx = 0;
    while (evalOK && (idx + 1 <= 40)) {
      evalOK = ((!std::isinf(cIneqPlus[idx])) && (!std::isnan(cIneqPlus[idx])));
      idx++;
    }
    if (evalOK) {
      xk[dim - 1] = temp;
    }
  }
  return evalOK;
}

} // namespace internal
} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (finDiffEvalAndChkErr.cpp)
