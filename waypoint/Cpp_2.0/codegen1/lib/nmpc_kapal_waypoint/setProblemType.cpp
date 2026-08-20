//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// setProblemType.cpp
//
// Code generation for function 'setProblemType'
//

// Include files
#include "setProblemType.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(g_struct_T &obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    obj.nVar = 20;
    obj.mConstr = 120;
    if (obj.nWConstr[4] > 0) {
      int idxStartIneq;
      idxStartIneq = obj.isActiveIdx[4] - 2;
      for (int idx_col{0}; idx_col < 20; idx_col++) {
        int idx_lb;
        idx_lb = (idxStartIneq + idx_col) + 1;
        obj.isActiveConstr[(obj.isActiveIdxNormal[4] + idx_col) - 1] =
            obj.isActiveConstr[idx_lb];
        obj.isActiveConstr[idx_lb] = false;
      }
    }
    for (int idx_col{0}; idx_col < 5; idx_col++) {
      obj.sizes[idx_col] = obj.sizesNormal[idx_col];
    }
    for (int idx_col{0}; idx_col < 6; idx_col++) {
      obj.isActiveIdx[idx_col] = obj.isActiveIdxNormal[idx_col];
    }
  } break;
  case 1: {
    int idxStartIneq;
    int idx_lb;
    obj.nVar = 21;
    obj.mConstr = 121;
    for (int idx_col{0}; idx_col < 5; idx_col++) {
      obj.sizes[idx_col] = obj.sizesPhaseOne[idx_col];
    }
    for (int idx_col{0}; idx_col < 80; idx_col++) {
      obj.Aineq[101 * idx_col + 20] = -1.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = 21;
    obj.lb[20] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    idx_lb = obj.nActiveConstr;
    for (int idx_col{idxStartIneq}; idx_col <= idx_lb; idx_col++) {
      obj.ATwset[101 * (idx_col - 1) + 20] = -1.0;
    }
    idx_lb = obj.isActiveIdx[4];
    if (obj.nWConstr[4] > 0) {
      for (int idx_col{19}; idx_col >= 0; idx_col--) {
        idxStartIneq = idx_lb + idx_col;
        obj.isActiveConstr[idxStartIneq] = obj.isActiveConstr[idxStartIneq - 1];
      }
    } else {
      obj.isActiveConstr[obj.isActiveIdx[4] + 19] = false;
    }
    obj.isActiveConstr[obj.isActiveIdx[4] - 1] = false;
    for (int idx_col{0}; idx_col < 6; idx_col++) {
      obj.isActiveIdx[idx_col] = obj.isActiveIdxPhaseOne[idx_col];
    }
  } break;
  case 2: {
    obj.nVar = 100;
    obj.mConstr = 200;
    for (int idx_col{0}; idx_col < 5; idx_col++) {
      obj.sizes[idx_col] = obj.sizesRegularized[idx_col];
    }
    if (obj.probType != 4) {
      int colOffsetAineq;
      int idxStartIneq;
      int idx_lb;
      idx_lb = 20;
      for (int idx_col{0}; idx_col < 80; idx_col++) {
        colOffsetAineq = 101 * idx_col - 1;
        idxStartIneq = idx_col + 20;
        if (idxStartIneq >= 21) {
          std::memset(&obj.Aineq[colOffsetAineq + 21], 0,
                      static_cast<unsigned int>(idxStartIneq - 20) *
                          sizeof(double));
        }
        obj.Aineq[(idx_col + colOffsetAineq) + 21] = -1.0;
        idxStartIneq = idx_col + 22;
        if (idxStartIneq <= 100) {
          std::memset(
              &obj.Aineq[idxStartIneq + colOffsetAineq], 0,
              static_cast<unsigned int>(
                  ((colOffsetAineq - idxStartIneq) - colOffsetAineq) + 101) *
                  sizeof(double));
        }
        idx_lb++;
        obj.indexLB[idx_col + 20] = idx_lb;
      }
      if (obj.nWConstr[4] > 0) {
        for (int idx_col{0}; idx_col < 20; idx_col++) {
          obj.isActiveConstr[obj.isActiveIdxRegularized[4] + idx_col] =
              obj.isActiveConstr[(obj.isActiveIdx[4] + idx_col) - 1];
        }
      }
      idxStartIneq = obj.isActiveIdx[4];
      idx_lb = obj.isActiveIdxRegularized[4];
      if (idxStartIneq <= idx_lb - 1) {
        std::memset(&obj.isActiveConstr[idxStartIneq + -1], 0,
                    static_cast<unsigned int>(idx_lb - idxStartIneq) *
                        sizeof(bool));
      }
      std::memset(&obj.lb[20], 0, 80U * sizeof(double));
      idx_lb = obj.isActiveIdx[2];
      colOffsetAineq = obj.nActiveConstr;
      for (int idx_col{idx_lb}; idx_col <= colOffsetAineq; idx_col++) {
        int colOffsetATw;
        colOffsetATw = 101 * (idx_col - 1) - 1;
        if (obj.Wid[idx_col - 1] == 3) {
          idxStartIneq = obj.Wlocalidx[idx_col - 1] + 19;
          if (idxStartIneq >= 21) {
            std::memset(&obj.ATwset[colOffsetATw + 21], 0,
                        static_cast<unsigned int>(idxStartIneq - 20) *
                            sizeof(double));
          }
          obj.ATwset[(obj.Wlocalidx[idx_col - 1] + colOffsetATw) + 20] = -1.0;
          idxStartIneq = obj.Wlocalidx[idx_col - 1] + 21;
          if (idxStartIneq <= 100) {
            std::memset(
                &obj.ATwset[idxStartIneq + colOffsetATw], 0,
                static_cast<unsigned int>(
                    ((colOffsetATw - idxStartIneq) - colOffsetATw) + 101) *
                    sizeof(double));
          }
        } else {
          std::memset(&obj.ATwset[colOffsetATw + 21], 0, 80U * sizeof(double));
        }
      }
    }
    for (int idx_col{0}; idx_col < 6; idx_col++) {
      obj.isActiveIdx[idx_col] = obj.isActiveIdxRegularized[idx_col];
    }
  } break;
  default: {
    int idxStartIneq;
    int idx_lb;
    obj.nVar = 101;
    obj.mConstr = 201;
    for (int idx_col{0}; idx_col < 5; idx_col++) {
      obj.sizes[idx_col] = obj.sizesRegPhaseOne[idx_col];
    }
    for (int idx_col{0}; idx_col < 80; idx_col++) {
      obj.Aineq[101 * idx_col + 100] = -1.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = 101;
    obj.lb[100] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    idx_lb = obj.nActiveConstr;
    for (int idx_col{idxStartIneq}; idx_col <= idx_lb; idx_col++) {
      obj.ATwset[101 * (idx_col - 1) + 100] = -1.0;
    }
    idxStartIneq = obj.isActiveIdx[4];
    if (obj.nWConstr[4] > 0) {
      for (int idx_col{19}; idx_col >= 0; idx_col--) {
        idx_lb = idxStartIneq + idx_col;
        obj.isActiveConstr[idx_lb] = obj.isActiveConstr[idx_lb - 1];
      }
    } else {
      obj.isActiveConstr[obj.isActiveIdx[4] + 19] = false;
    }
    obj.isActiveConstr[obj.isActiveIdx[4] - 1] = false;
    for (int idx_col{0}; idx_col < 6; idx_col++) {
      obj.isActiveIdx[idx_col] = obj.isActiveIdxRegPhaseOne[idx_col];
    }
  } break;
  }
  obj.probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (setProblemType.cpp)
