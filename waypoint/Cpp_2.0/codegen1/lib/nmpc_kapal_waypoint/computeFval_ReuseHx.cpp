//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval_ReuseHx.cpp
//
// Code generation for function 'computeFval_ReuseHx'
//

// Include files
#include "computeFval_ReuseHx.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const struct_T &obj, double workspace[20301],
                           const double f[101], const double x[101])
{
  double val;
  val = 0.0;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x[obj.nvar - 1];
    break;
  case 3: {
    if (obj.hasLinear) {
      int ixlast;
      ixlast = obj.nvar;
      for (int i{0}; i < ixlast; i++) {
        workspace[i] = 0.5 * obj.Hx[i] + f[i];
      }
      if (obj.nvar >= 1) {
        for (int i{0}; i < ixlast; i++) {
          val += x[i] * workspace[i];
        }
      }
    } else {
      if (obj.nvar >= 1) {
        int ixlast;
        ixlast = obj.nvar;
        for (int i{0}; i < ixlast; i++) {
          val += x[i] * obj.Hx[i];
        }
      }
      val *= 0.5;
    }
  } break;
  case 4: {
    if (obj.hasLinear) {
      int ixlast;
      ixlast = obj.nvar;
      if (ixlast - 1 >= 0) {
        std::copy(&f[0], &f[ixlast], &workspace[0]);
      }
      ixlast = 100 - obj.nvar;
      for (int i{0}; i < ixlast; i++) {
        workspace[obj.nvar + i] = obj.rho;
      }
      for (int i{0}; i < 100; i++) {
        double d;
        d = workspace[i] + 0.5 * obj.Hx[i];
        workspace[i] = d;
        val += x[i] * d;
      }
    } else {
      int ixlast;
      for (int i{0}; i < 100; i++) {
        val += x[i] * obj.Hx[i];
      }
      val *= 0.5;
      ixlast = obj.nvar + 1;
      for (int i{ixlast}; i < 101; i++) {
        val += x[i - 1] * obj.rho;
      }
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeFval_ReuseHx.cpp)
