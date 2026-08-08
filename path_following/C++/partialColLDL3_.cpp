//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// partialColLDL3_.cpp
//
// Code generation for function 'partialColLDL3_'
//

// Include files
#include "partialColLDL3_.h"
#include "nmpc_kapal_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void partialColLDL3_(i_struct_T &obj, int LD_offset, int NColsRemain)
{
  double d;
  int LD_diagOffset;
  int br;
  int i;
  int i1;
  int ix;
  int subRows;
  for (int k{0}; k < 48; k++) {
    double y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LD_offset + 302 * k) - 1;
    for (int idx{0}; idx <= subRows; idx++) {
      obj.workspace_ = obj.FMat[LD_diagOffset + idx];
    }
    for (int idx{0}; idx < NColsRemain; idx++) {
      obj.workspace2_ = obj.workspace_;
    }
    y = obj.workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      br = 301 * (k - 1) + 1;
      for (int idx{1}; idx <= br; idx += 301) {
        i1 = idx + NColsRemain;
        for (int ia{idx}; ia < i1; ia++) {
          y += obj.workspace_ * -obj.FMat[ix - 1];
        }
        ix += 301;
      }
    }
    obj.workspace2_ = y;
    for (int idx{0}; idx < NColsRemain; idx++) {
      obj.workspace_ = y;
    }
    for (int idx{0}; idx <= subRows; idx++) {
      obj.FMat[LD_diagOffset + idx] = obj.workspace_;
    }
    for (int idx{0}; idx < subRows; idx++) {
      ix = (LD_diagOffset + idx) + 1;
      obj.FMat[ix] /= obj.FMat[LD_diagOffset];
    }
  }
  i = NColsRemain - 1;
  for (int j{48}; j <= i; j += 48) {
    int m;
    int subBlockSize;
    subRows = NColsRemain - j;
    if (subRows >= 48) {
      subBlockSize = 48;
    } else {
      subBlockSize = subRows;
    }
    LD_diagOffset = j + subBlockSize;
    for (int k{j}; k < LD_diagOffset; k++) {
      m = LD_diagOffset - k;
      for (int idx{0}; idx < 48; idx++) {
        d = obj.FMat[((LD_offset + k) + idx * 301) - 1];
      }
      obj.workspace2_ = d;
      ix = k + 1;
      if (m != 0) {
        br = k + 14148;
        for (int idx{ix}; idx <= br; idx += 301) {
          i1 = idx + m;
          for (int ia{idx}; ia < i1; ia++) {
            // Check node always fails. would cause program termination and was
            // eliminated
          }
        }
      }
    }
    if (LD_diagOffset < NColsRemain) {
      int ic0;
      m = subRows - subBlockSize;
      ic0 = ((LD_offset + subBlockSize) + 302 * j) - 1;
      for (int idx{0}; idx < 48; idx++) {
        ix = (LD_offset + j) + idx * 301;
        for (int ia{0}; ia < subBlockSize; ia++) {
          obj.workspace2_ = obj.FMat[(ix + ia) - 1];
        }
      }
      if ((m != 0) && (subBlockSize != 0)) {
        ix = ic0 + 301 * (subBlockSize - 1);
        br = 0;
        for (int idx{ic0}; idx <= ix; idx += 301) {
          br++;
          i1 = br + 14147;
          for (int ia{br}; ia <= i1; ia += 301) {
            subRows = idx + 1;
            LD_diagOffset = idx + m;
            for (int k{subRows}; k <= LD_diagOffset; k++) {
              obj.FMat[k - 1] += -obj.workspace2_ * obj.workspace_;
            }
          }
        }
      }
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (partialColLDL3_.cpp)
