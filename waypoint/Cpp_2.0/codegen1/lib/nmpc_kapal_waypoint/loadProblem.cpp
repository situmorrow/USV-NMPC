//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// loadProblem.cpp
//
// Code generation for function 'loadProblem'
//

// Include files
#include "loadProblem.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void loadProblem(g_struct_T &obj, const double Aineq[800])
{
  static const signed char iv[6]{1, 0, 0, 80, 20, 20};
  static const signed char iv4[6]{1, 0, 0, 80, 21, 20};
  static const signed char iv5[6]{1, 0, 0, 80, 100, 20};
  static const signed char iv6[6]{1, 0, 0, 80, 101, 20};
  static const signed char iv1[5]{0, 0, 80, 21, 20};
  static const signed char iv2[5]{0, 0, 80, 100, 20};
  static const signed char iv3[5]{0, 0, 80, 101, 20};
  static const signed char obj_tmp[5]{0, 0, 80, 20, 20};
  obj.mConstr = 120;
  obj.mConstrOrig = 120;
  obj.mConstrMax = 201;
  for (int i{0}; i < 5; i++) {
    signed char b_i;
    b_i = obj_tmp[i];
    obj.sizes[i] = b_i;
    obj.sizesNormal[i] = b_i;
    obj.sizesPhaseOne[i] = iv1[i];
    obj.sizesRegularized[i] = iv2[i];
    obj.sizesRegPhaseOne[i] = iv3[i];
  }
  for (int i{0}; i < 6; i++) {
    obj.isActiveIdxRegPhaseOne[i] = iv[i];
  }
  for (int i{0}; i < 5; i++) {
    obj.isActiveIdxRegPhaseOne[i + 1] += obj.isActiveIdxRegPhaseOne[i];
  }
  for (int i{0}; i < 6; i++) {
    int i1;
    i1 = obj.isActiveIdxRegPhaseOne[i];
    obj.isActiveIdx[i] = i1;
    obj.isActiveIdxNormal[i] = i1;
    obj.isActiveIdxRegPhaseOne[i] = iv4[i];
  }
  for (int i{0}; i < 5; i++) {
    obj.isActiveIdxRegPhaseOne[i + 1] += obj.isActiveIdxRegPhaseOne[i];
  }
  for (int i{0}; i < 6; i++) {
    obj.isActiveIdxPhaseOne[i] = obj.isActiveIdxRegPhaseOne[i];
    obj.isActiveIdxRegPhaseOne[i] = iv5[i];
  }
  for (int i{0}; i < 5; i++) {
    obj.isActiveIdxRegPhaseOne[i + 1] += obj.isActiveIdxRegPhaseOne[i];
  }
  for (int i{0}; i < 6; i++) {
    obj.isActiveIdxRegularized[i] = obj.isActiveIdxRegPhaseOne[i];
    obj.isActiveIdxRegPhaseOne[i] = iv6[i];
  }
  for (int i{0}; i < 5; i++) {
    obj.isActiveIdxRegPhaseOne[i + 1] += obj.isActiveIdxRegPhaseOne[i];
  }
  for (int i{0}; i < 40; i++) {
    for (int idx_row{0}; idx_row < 20; idx_row++) {
      obj.Aineq[idx_row + 101 * i] = Aineq[i + 40 * idx_row];
    }
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (loadProblem.cpp)
