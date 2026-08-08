//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGrad_StoreHx.cpp
//
// Code generation for function 'computeGrad_StoreHx'
//

// Include files
#include "computeGrad_StoreHx.h"
#include "nmpc_kapal_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void computeGrad_StoreHx(struct_T &obj, const double H[900],
                         const double f[151], const double x[151])
{
  switch (obj.objtype) {
  case 5: {
    int ixlast;
    ixlast = obj.nvar;
    if (ixlast - 2 >= 0) {
      std::memset(&obj.grad[0], 0,
                  static_cast<unsigned int>(ixlast - 1) * sizeof(double));
    }
    obj.grad[obj.nvar - 1] = obj.gammaScalar;
  } break;
  case 3: {
    int ixlast;
    int lda;
    ixlast = obj.nvar - 1;
    lda = obj.nvar;
    if (obj.nvar != 0) {
      int i;
      int ix;
      if (ixlast >= 0) {
        std::memset(&obj.Hx[0], 0,
                    static_cast<unsigned int>(ixlast + 1) * sizeof(double));
      }
      ix = 0;
      i = obj.nvar * (obj.nvar - 1) + 1;
      for (int idx{1}; lda < 0 ? idx >= i : idx <= i; idx += lda) {
        int i1;
        i1 = idx + ixlast;
        for (int ia{idx}; ia <= i1; ia++) {
          int i2;
          i2 = ia - idx;
          obj.Hx[i2] += H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    ixlast = obj.nvar;
    if (ixlast - 1 >= 0) {
      std::copy(&obj.Hx[0], &obj.Hx[ixlast], &obj.grad[0]);
    }
    if (obj.hasLinear && (obj.nvar >= 1)) {
      for (int idx{0}; idx < ixlast; idx++) {
        obj.grad[idx] += f[idx];
      }
    }
  } break;
  case 4: {
    int ix;
    int ixlast;
    int lda;
    ixlast = obj.nvar - 1;
    lda = obj.nvar;
    if (obj.nvar != 0) {
      int i;
      if (ixlast >= 0) {
        std::memset(&obj.Hx[0], 0,
                    static_cast<unsigned int>(ixlast + 1) * sizeof(double));
      }
      ix = 0;
      i = obj.nvar * (obj.nvar - 1) + 1;
      for (int idx{1}; lda < 0 ? idx >= i : idx <= i; idx += lda) {
        int i1;
        i1 = idx + ixlast;
        for (int ia{idx}; ia <= i1; ia++) {
          int i2;
          i2 = ia - idx;
          obj.Hx[i2] += H[ia - 1] * x[ix];
        }
        ix++;
      }
    }
    ixlast = obj.nvar + 1;
    for (int idx{ixlast}; idx < 151; idx++) {
      obj.Hx[idx - 1] = obj.beta * x[idx - 1];
    }
    std::copy(&obj.Hx[0], &obj.Hx[150], &obj.grad[0]);
    if (obj.hasLinear && (obj.nvar >= 1)) {
      ixlast = obj.nvar;
      for (int idx{0}; idx < ixlast; idx++) {
        obj.grad[idx] += f[idx];
      }
    }
    if (150 - obj.nvar >= 1) {
      ixlast = obj.nvar;
      lda = 149 - obj.nvar;
      for (int idx{0}; idx <= lda; idx++) {
        ix = ixlast + idx;
        obj.grad[ix] += obj.rho;
      }
    }
  } break;
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeGrad_StoreHx.cpp)
