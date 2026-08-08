//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstr.cpp
//
// Code generation for function 'evalObjAndConstr'
//

// Include files
#include "evalObjAndConstr.h"
#include "anonymous_function.h"
#include "computeConstraints_.h"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_internal_types1.h"
#include "rt_nonfinite.h"
#include "stickyStruct.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
double evalObjAndConstr(const ::coder::internal::i_stickyStruct &obj,
                        const double x[30], double Cineq_workspace[120],
                        int &status)
{
  double fval;
  int b_status;
  bool b;
  fval = nmpc_kapal_waypoint_anonFcn1(
      obj.next.next.next.next.next.next.next.next.value.workspace
          .current_state_nd,
      obj.next.next.next.next.next.next.next.next.value.workspace.x_ref_seq,
      obj.next.next.next.next.next.next.next.next.value.workspace.y_ref_seq,
      obj.next.next.next.next.next.next.next.next.value.workspace.psi_ref_seq,
      x);
  b_status = 1;
  b = std::isnan(fval);
  if (std::isinf(fval) || b) {
    if (b) {
      b_status = -3;
    } else if (fval < 0.0) {
      b_status = -1;
    } else {
      b_status = -2;
    }
  }
  status = b_status;
  if (b_status == 1) {
    status = computeConstraints_(obj, x, Cineq_workspace);
  }
  return fval;
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (evalObjAndConstr.cpp)
