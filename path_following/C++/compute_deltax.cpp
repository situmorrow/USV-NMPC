//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compute_deltax.cpp
//
// Code generation for function 'compute_deltax'
//

// Include files
#include "compute_deltax.h"
#include "fullColLDL2_.h"
#include "nmpc_kapal_internal_types.h"
#include "nmpc_kapal_rtwutil.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const double H[900], f_struct_T &solution,
                    b_struct_T &memspace, const c_struct_T &qrmanager,
                    i_struct_T &cholmanager, const struct_T &objective,
                    bool alwaysPositiveDef)
{
  int mNull;
  int nVar;
  nVar = qrmanager.mrows - 1;
  mNull = qrmanager.mrows - qrmanager.ncols;
  if (mNull <= 0) {
    if (nVar >= 0) {
      std::memset(&solution.searchDir[0], 0,
                  static_cast<unsigned int>(nVar + 1) * sizeof(double));
    }
  } else {
    for (int idx{0}; idx <= nVar; idx++) {
      solution.searchDir[idx] = -objective.grad[idx];
    }
    if (qrmanager.ncols <= 0) {
      switch (objective.objtype) {
      case 5:
        break;
      case 3: {
        int ix;
        int jA;
        int jjA;
        int k;
        if (alwaysPositiveDef) {
          jA = qrmanager.mrows;
          cholmanager.ndims = qrmanager.mrows;
          for (int idx{0}; idx < jA; idx++) {
            k = qrmanager.mrows * idx;
            jjA = 301 * idx;
            for (int b_k{0}; b_k < jA; b_k++) {
              cholmanager.FMat[jjA + b_k] = H[k + b_k];
            }
          }
          cholmanager.info =
              internal::lapack::xpotrf(qrmanager.mrows, cholmanager.FMat);
        } else {
          ix = qrmanager.mrows;
          cholmanager.ndims = qrmanager.mrows;
          for (int idx{0}; idx < ix; idx++) {
            k = qrmanager.mrows * idx;
            jjA = 301 * idx;
            for (int b_k{0}; b_k < ix; b_k++) {
              cholmanager.FMat[jjA + b_k] = H[k + b_k];
            }
          }
          if (qrmanager.mrows > 128) {
            bool exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < ix)) {
              jA = 302 * k + 1;
              jjA = ix - k;
              if (k + 48 <= ix) {
                DynamicRegCholManager::partialColLDL3_(cholmanager, jA, jjA);
                k += 48;
              } else {
                DynamicRegCholManager::fullColLDL2_(cholmanager, jA, jjA);
                exitg1 = true;
              }
            }
          } else {
            DynamicRegCholManager::fullColLDL2_(cholmanager, 1,
                                                qrmanager.mrows);
          }
          if (cholmanager.ConvexCheck) {
            jA = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (jA <= ix - 1) {
                if (cholmanager.FMat[jA + 301 * jA] <= 0.0) {
                  cholmanager.info = -jA - 1;
                  exitg2 = 1;
                } else {
                  jA++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else if (alwaysPositiveDef) {
          CholManager::solve(cholmanager, solution.searchDir);
        } else {
          k = cholmanager.ndims - 2;
          if (cholmanager.ndims != 0) {
            for (int idx{0}; idx <= k + 1; idx++) {
              jjA = idx + idx * 301;
              jA = k - idx;
              for (int b_k{0}; b_k <= jA; b_k++) {
                ix = (idx + b_k) + 1;
                solution.searchDir[ix] -=
                    solution.searchDir[idx] * cholmanager.FMat[(jjA + b_k) + 1];
              }
            }
          }
          jjA = cholmanager.ndims;
          for (int idx{0}; idx < jjA; idx++) {
            solution.searchDir[idx] /= cholmanager.FMat[idx + 301 * idx];
          }
          if (cholmanager.ndims != 0) {
            for (int idx{jjA}; idx >= 1; idx--) {
              double temp;
              jA = (idx - 1) * 301;
              temp = solution.searchDir[idx - 1];
              k = idx + 1;
              for (int b_k{jjA}; b_k >= k; b_k--) {
                temp -= cholmanager.FMat[(jA + b_k) - 1] *
                        solution.searchDir[b_k - 1];
              }
              solution.searchDir[idx - 1] = temp;
            }
          }
        }
      } break;
      case 4: {
        if (alwaysPositiveDef) {
          int jA;
          int k;
          jA = objective.nvar;
          cholmanager.ndims = objective.nvar;
          for (int idx{0}; idx < jA; idx++) {
            int jjA;
            k = objective.nvar * idx;
            jjA = 301 * idx;
            for (int b_k{0}; b_k < jA; b_k++) {
              cholmanager.FMat[jjA + b_k] = H[k + b_k];
            }
          }
          cholmanager.info =
              internal::lapack::xpotrf(objective.nvar, cholmanager.FMat);
          if (cholmanager.info != 0) {
            solution.state = -6;
          } else {
            double temp;
            CholManager::solve(cholmanager, solution.searchDir);
            temp = 1.0 / objective.beta;
            jA = objective.nvar + 1;
            k = qrmanager.mrows;
            for (int idx{jA}; idx <= k; idx++) {
              solution.searchDir[idx - 1] *= temp;
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx;
      nullStartIdx = 301 * qrmanager.ncols + 1;
      if (objective.objtype == 5) {
        for (int idx{0}; idx < mNull; idx++) {
          memspace.workspace_float[idx] =
              -qrmanager.Q[nVar + 301 * (qrmanager.ncols + idx)];
        }
        if (qrmanager.mrows != 0) {
          int ix;
          int k;
          if (nVar >= 0) {
            std::memset(&solution.searchDir[0], 0,
                        static_cast<unsigned int>(nVar + 1) * sizeof(double));
          }
          ix = 0;
          k = nullStartIdx + 301 * (mNull - 1);
          for (int idx{nullStartIdx}; idx <= k; idx += 301) {
            int jA;
            jA = idx + nVar;
            for (int b_k{idx}; b_k <= jA; b_k++) {
              int jjA;
              jjA = b_k - idx;
              solution.searchDir[jjA] +=
                  qrmanager.Q[b_k - 1] * memspace.workspace_float[ix];
            }
            ix++;
          }
        }
      } else {
        int jA;
        int jjA;
        int k;
        if (objective.objtype == 3) {
          internal::blas::xgemm(qrmanager.mrows, mNull, qrmanager.mrows, H,
                                qrmanager.mrows, qrmanager.Q, nullStartIdx,
                                memspace.workspace_float);
          internal::blas::xgemm(mNull, mNull, qrmanager.mrows, qrmanager.Q,
                                nullStartIdx, memspace.workspace_float,
                                cholmanager.FMat);
        } else if (alwaysPositiveDef) {
          k = qrmanager.mrows;
          internal::blas::xgemm(objective.nvar, mNull, objective.nvar, H,
                                objective.nvar, qrmanager.Q, nullStartIdx,
                                memspace.workspace_float);
          jA = objective.nvar + 1;
          for (int idx{0}; idx < mNull; idx++) {
            for (int b_k{jA}; b_k <= k; b_k++) {
              memspace.workspace_float[(b_k + 301 * idx) - 1] =
                  objective.beta *
                  qrmanager.Q[(b_k + 301 * (idx + qrmanager.ncols)) - 1];
            }
          }
          internal::blas::xgemm(mNull, mNull, qrmanager.mrows, qrmanager.Q,
                                nullStartIdx, memspace.workspace_float,
                                cholmanager.FMat);
        }
        if (alwaysPositiveDef) {
          cholmanager.ndims = mNull;
          cholmanager.info = internal::lapack::xpotrf(mNull, cholmanager.FMat);
        } else {
          cholmanager.ndims = mNull;
          if (mNull > 128) {
            bool exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < mNull)) {
              jA = 302 * k + 1;
              jjA = mNull - k;
              if (k + 48 <= mNull) {
                DynamicRegCholManager::partialColLDL3_(cholmanager, jA, jjA);
                k += 48;
              } else {
                DynamicRegCholManager::fullColLDL2_(cholmanager, jA, jjA);
                exitg1 = true;
              }
            }
          } else {
            DynamicRegCholManager::fullColLDL2_(cholmanager, 1, mNull);
          }
          if (cholmanager.ConvexCheck) {
            jA = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (jA <= mNull - 1) {
                if (cholmanager.FMat[jA + 301 * jA] <= 0.0) {
                  cholmanager.info = -jA - 1;
                  exitg2 = 1;
                } else {
                  jA++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else {
          double temp;
          int ix;
          if (qrmanager.mrows != 0) {
            std::memset(&memspace.workspace_float[0], 0,
                        static_cast<unsigned int>(mNull) * sizeof(double));
            jA = nullStartIdx + 301 * (mNull - 1);
            for (int b_k{nullStartIdx}; b_k <= jA; b_k += 301) {
              temp = 0.0;
              k = b_k + nVar;
              for (int idx{b_k}; idx <= k; idx++) {
                temp += qrmanager.Q[idx - 1] * objective.grad[idx - b_k];
              }
              k = div_nzp_s32_floor(b_k - nullStartIdx, 301);
              memspace.workspace_float[k] -= temp;
            }
          }
          if (alwaysPositiveDef) {
            jjA = cholmanager.ndims;
            if (cholmanager.ndims != 0) {
              for (int idx{0}; idx < jjA; idx++) {
                k = idx * 301;
                temp = memspace.workspace_float[idx];
                for (int b_k{0}; b_k < idx; b_k++) {
                  temp -=
                      cholmanager.FMat[k + b_k] * memspace.workspace_float[b_k];
                }
                memspace.workspace_float[idx] =
                    temp / cholmanager.FMat[k + idx];
              }
            }
            if (cholmanager.ndims != 0) {
              for (int idx{jjA}; idx >= 1; idx--) {
                k = (idx + (idx - 1) * 301) - 1;
                memspace.workspace_float[idx - 1] /= cholmanager.FMat[k];
                for (int b_k{0}; b_k <= idx - 2; b_k++) {
                  jA = (idx - b_k) - 2;
                  memspace.workspace_float[jA] -=
                      memspace.workspace_float[idx - 1] *
                      cholmanager.FMat[(k - b_k) - 1];
                }
              }
            }
          } else {
            k = cholmanager.ndims - 2;
            if (cholmanager.ndims != 0) {
              for (int idx{0}; idx <= k + 1; idx++) {
                jA = idx + idx * 301;
                jjA = k - idx;
                for (int b_k{0}; b_k <= jjA; b_k++) {
                  ix = (idx + b_k) + 1;
                  memspace.workspace_float[ix] -=
                      memspace.workspace_float[idx] *
                      cholmanager.FMat[(jA + b_k) + 1];
                }
              }
            }
            jjA = cholmanager.ndims;
            for (int idx{0}; idx < jjA; idx++) {
              memspace.workspace_float[idx] /=
                  cholmanager.FMat[idx + 301 * idx];
            }
            if (cholmanager.ndims != 0) {
              for (int idx{jjA}; idx >= 1; idx--) {
                k = (idx - 1) * 301;
                temp = memspace.workspace_float[idx - 1];
                jA = idx + 1;
                for (int b_k{jjA}; b_k >= jA; b_k--) {
                  temp -= cholmanager.FMat[(k + b_k) - 1] *
                          memspace.workspace_float[b_k - 1];
                }
                memspace.workspace_float[idx - 1] = temp;
              }
            }
          }
          if (qrmanager.mrows != 0) {
            if (nVar >= 0) {
              std::memset(&solution.searchDir[0], 0,
                          static_cast<unsigned int>(nVar + 1) * sizeof(double));
            }
            ix = 0;
            k = nullStartIdx + 301 * (mNull - 1);
            for (int idx{nullStartIdx}; idx <= k; idx += 301) {
              jA = idx + nVar;
              for (int b_k{idx}; b_k <= jA; b_k++) {
                jjA = b_k - idx;
                solution.searchDir[jjA] +=
                    qrmanager.Q[b_k - 1] * memspace.workspace_float[ix];
              }
              ix++;
            }
          }
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (compute_deltax.cpp)
