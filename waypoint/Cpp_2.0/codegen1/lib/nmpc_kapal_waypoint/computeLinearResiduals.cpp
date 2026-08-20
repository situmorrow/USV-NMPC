//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeLinearResiduals.cpp
//
// Code generation for function 'computeLinearResiduals'
//

// Include files
#include "computeLinearResiduals.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void computeLinearResiduals(const double x[20], int nVar,
                            double workspaceIneq[80], const double AineqT[8080],
                            const double bineq[40])
{
  for (int k{0}; k < 40; k++) {
    workspaceIneq[k] = -bineq[k];
  }
  for (int iac{0}; iac <= 3939; iac += 101) {
    double c;
    int i;
    c = 0.0;
    i = iac + nVar;
    for (int k{iac + 1}; k <= i; k++) {
      c += AineqT[k - 1] * x[(k - iac) - 1];
    }
    i = div_nzp_s32_floor(iac, 101);
    workspaceIneq[i] += c;
  }
}

} // namespace internal
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeLinearResiduals.cpp)
