//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BFGSUpdate.cpp
//
// Code generation for function 'BFGSUpdate'
//

// Include files
#include "BFGSUpdate.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool BFGSUpdate(int nvar, double Bk[900], const double sk[151], double yk[151],
                double workspace[45451])
{
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int ix;
  bool success;
  dotSY = 0.0;
  i = static_cast<unsigned char>(nvar);
  for (int k{0}; k < i; k++) {
    dotSY += sk[k] * yk[k];
    workspace[k] = 0.0;
  }
  ix = 0;
  i1 = 30 * (nvar - 1) + 1;
  for (int k{1}; k <= i1; k += 30) {
    int i2;
    i2 = k + nvar;
    for (int ia{k}; ia < i2; ia++) {
      int i3;
      i3 = ia - k;
      workspace[i3] += Bk[ia - 1] * sk[ix];
    }
    ix++;
  }
  curvatureS = 0.0;
  if (nvar >= 1) {
    for (int k{0}; k < nvar; k++) {
      curvatureS += sk[k] * workspace[k];
    }
  }
  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    for (int k{0}; k < i; k++) {
      yk[k] *= theta;
    }
    if (!(1.0 - theta == 0.0)) {
      for (int k{0}; k < nvar; k++) {
        yk[k] += (1.0 - theta) * workspace[k];
      }
    }
    dotSY = 0.0;
    for (int k{0}; k < i; k++) {
      dotSY += sk[k] * yk[k];
    }
  }
  if ((curvatureS > 2.2204460492503131E-16) &&
      (dotSY > 2.2204460492503131E-16)) {
    success = true;
  } else {
    success = false;
  }
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 1;
      for (int k{0}; k < i; k++) {
        theta = workspace[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = nvar + ix;
          for (int ia{ix}; ia < i1; ia++) {
            Bk[ia - 1] += workspace[ia - ix] * theta;
          }
        }
        ix += 30;
      }
    }
    curvatureS = 1.0 / dotSY;
    if (!(curvatureS == 0.0)) {
      ix = 1;
      for (int k{0}; k < i; k++) {
        theta = yk[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = nvar + ix;
          for (int ia{ix}; ia < i1; ia++) {
            Bk[ia - 1] += yk[ia - ix] * theta;
          }
        }
        ix += 30;
      }
    }
  }
  return success;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (BFGSUpdate.cpp)
