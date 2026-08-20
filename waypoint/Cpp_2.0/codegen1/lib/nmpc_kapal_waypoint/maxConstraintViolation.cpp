//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// maxConstraintViolation.cpp
//
// Code generation for function 'maxConstraintViolation'
//

// Include files
#include "maxConstraintViolation.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double b_maxConstraintViolation(g_struct_T &obj, const double x[101])
{
  double v;
  int idxUB;
  if (obj.probType == 2) {
    double c;
    v = 0.0;
    for (int k{0}; k < 80; k++) {
      obj.maxConstrWorkspace[k] = -obj.bineq[k];
    }
    for (int iac{0}; iac <= 7979; iac += 101) {
      c = 0.0;
      idxUB = iac + 20;
      for (int k{iac + 1}; k <= idxUB; k++) {
        c += obj.Aineq[k - 1] * x[(k - iac) - 1];
      }
      idxUB = div_nzp_s32_floor(iac, 101);
      obj.maxConstrWorkspace[idxUB] += c;
    }
    for (int k{0}; k < 80; k++) {
      c = obj.maxConstrWorkspace[k] - x[k + 20];
      obj.maxConstrWorkspace[k] = c;
      v = std::fmax(v, c);
    }
  } else {
    v = 0.0;
    for (int k{0}; k < 80; k++) {
      obj.maxConstrWorkspace[k] = -obj.bineq[k];
    }
    for (int iac{0}; iac <= 7979; iac += 101) {
      double c;
      c = 0.0;
      idxUB = iac + obj.nVar;
      for (int k{iac + 1}; k <= idxUB; k++) {
        c += obj.Aineq[k - 1] * x[(k - iac) - 1];
      }
      idxUB = div_nzp_s32_floor(iac, 101);
      obj.maxConstrWorkspace[idxUB] += c;
    }
    for (int k{0}; k < 80; k++) {
      v = std::fmax(v, obj.maxConstrWorkspace[k]);
    }
  }
  idxUB = static_cast<unsigned char>(obj.sizes[3]);
  for (int k{0}; k < idxUB; k++) {
    int idxLB;
    idxLB = obj.indexLB[k] - 1;
    v = std::fmax(v, -x[idxLB] - obj.lb[idxLB]);
  }
  for (int k{0}; k < 20; k++) {
    idxUB = obj.indexUB[k] - 1;
    v = std::fmax(v, x[idxUB] - obj.ub[idxUB]);
  }
  return v;
}

double maxConstraintViolation(g_struct_T &obj, const double x[20301])
{
  double v;
  int idxUB;
  if (obj.probType == 2) {
    double c;
    v = 0.0;
    for (int k{0}; k < 80; k++) {
      obj.maxConstrWorkspace[k] = -obj.bineq[k];
    }
    for (int iac{0}; iac <= 7979; iac += 101) {
      c = 0.0;
      idxUB = iac + 20;
      for (int k{iac + 1}; k <= idxUB; k++) {
        c += obj.Aineq[k - 1] * x[(k - iac) + 200];
      }
      idxUB = div_nzp_s32_floor(iac, 101);
      obj.maxConstrWorkspace[idxUB] += c;
    }
    for (int k{0}; k < 80; k++) {
      c = obj.maxConstrWorkspace[k] - x[k + 221];
      obj.maxConstrWorkspace[k] = c;
      v = std::fmax(v, c);
    }
  } else {
    v = 0.0;
    for (int k{0}; k < 80; k++) {
      obj.maxConstrWorkspace[k] = -obj.bineq[k];
    }
    for (int iac{0}; iac <= 7979; iac += 101) {
      double c;
      c = 0.0;
      idxUB = iac + obj.nVar;
      for (int k{iac + 1}; k <= idxUB; k++) {
        c += obj.Aineq[k - 1] * x[(k - iac) + 200];
      }
      idxUB = div_nzp_s32_floor(iac, 101);
      obj.maxConstrWorkspace[idxUB] += c;
    }
    for (int k{0}; k < 80; k++) {
      v = std::fmax(v, obj.maxConstrWorkspace[k]);
    }
  }
  idxUB = static_cast<unsigned char>(obj.sizes[3]);
  for (int k{0}; k < idxUB; k++) {
    int idxLB;
    idxLB = obj.indexLB[k];
    v = std::fmax(v, -x[idxLB + 200] - obj.lb[idxLB - 1]);
  }
  for (int k{0}; k < 20; k++) {
    idxUB = obj.indexUB[k];
    v = std::fmax(v, x[idxUB + 200] - obj.ub[idxUB - 1]);
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (maxConstraintViolation.cpp)
