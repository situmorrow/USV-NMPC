//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fullColLDL2_.cpp
//
// Code generation for function 'fullColLDL2_'
//

// Include files
#include "fullColLDL2_.h"
#include "nmpc_kapal_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(i_struct_T &obj, int LD_offset, int NColsRemain)
{
  for (int k{0}; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int jA;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = LD_offset + 302 * k;
    alpha1 = -1.0 / obj.FMat[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 1;
    y = obj.workspace_;
    for (int b_k{0}; b_k <= subMatrixDim; b_k++) {
      y = obj.FMat[LD_diagOffset + b_k];
    }
    obj.workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (int b_k{0}; b_k <= subMatrixDim; b_k++) {
        if (y != 0.0) {
          double temp;
          int i;
          int i1;
          temp = y * alpha1;
          i = jA + 302;
          i1 = subMatrixDim + jA;
          for (int ijA{i}; ijA <= i1 + 302; ijA++) {
            obj.FMat[ijA - 1] += y * temp;
          }
        }
        jA += 301;
      }
    }
    alpha1 = 1.0 / obj.FMat[LD_diagOffset - 1];
    jA = (LD_diagOffset + subMatrixDim) + 1;
    for (int b_k{offset1}; b_k <= jA; b_k++) {
      obj.FMat[b_k - 1] *= alpha1;
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (fullColLDL2_.cpp)
