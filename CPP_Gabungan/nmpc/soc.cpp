//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// soc.cpp
//
// Code generation for function 'soc'
//

// Include files
#include "soc.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
bool b_soc(const double Hessian[400], const double grad[101],
           f_struct_T &b_TrialState, b_struct_T &memspace,
           g_struct_T &WorkingSet, c_struct_T &b_QRManager,
           i_struct_T &b_CholManager, struct_T &QPObjective,
           const k_struct_T &qpoptions)
{
  double c;
  int i;
  int idxIneqOffset;
  int idx_Aineq;
  int idx_Partition;
  int idx_lower;
  int idx_upper;
  int nVar;
  int nWIneq_old;
  int nWLower_old;
  int nWUpper_old;
  bool success;
  nWIneq_old = WorkingSet.nWConstr[2];
  nWLower_old = WorkingSet.nWConstr[3];
  nWUpper_old = WorkingSet.nWConstr[4];
  nVar = WorkingSet.nVar;
  idx_Aineq = static_cast<unsigned char>(WorkingSet.nVar);
  std::copy(&b_TrialState.xstarsqp_old[0],
            &b_TrialState.xstarsqp_old[idx_Aineq], &b_TrialState.xstarsqp[0]);
  idx_Aineq = static_cast<unsigned char>(WorkingSet.nVar);
  std::copy(&b_TrialState.xstar[0], &b_TrialState.xstar[idx_Aineq],
            &b_TrialState.socDirection[0]);
  std::copy(&b_TrialState.lambda[0], &b_TrialState.lambda[201],
            &b_TrialState.lambdaStopTest[0]);
  idxIneqOffset = WorkingSet.isActiveIdx[2];
  for (int idx{0}; idx < 80; idx++) {
    WorkingSet.bineq[idx] = -b_TrialState.cIneq[idx];
  }
  for (int iac{0}; iac <= 7979; iac += 101) {
    c = 0.0;
    idx_Aineq = iac + WorkingSet.nVar;
    for (int idx{iac + 1}; idx <= idx_Aineq; idx++) {
      c += WorkingSet.Aineq[idx - 1] * b_TrialState.searchDir[(idx - iac) - 1];
    }
    idx_Aineq = div_nzp_s32_floor(iac, 101);
    WorkingSet.bineq[idx_Aineq] += c;
  }
  idx_Aineq = 1;
  idx_lower = 81;
  idx_upper = WorkingSet.sizes[3] + 81;
  i = WorkingSet.nActiveConstr;
  for (int idx{idxIneqOffset}; idx <= i; idx++) {
    switch (WorkingSet.Wid[idx - 1]) {
    case 3:
      idx_Partition = idx_Aineq;
      idx_Aineq++;
      WorkingSet.bwset[idx - 1] =
          WorkingSet.bineq[WorkingSet.Wlocalidx[idx - 1] - 1];
      break;
    case 4:
      idx_Partition = idx_lower;
      idx_lower++;
      break;
    default:
      idx_Partition = idx_upper;
      idx_upper++;
      break;
    }
    b_TrialState.workingset_old[idx_Partition - 1] =
        WorkingSet.Wlocalidx[idx - 1];
  }
  idx_Aineq = static_cast<unsigned char>(WorkingSet.nVar);
  std::copy(&b_TrialState.xstarsqp[0], &b_TrialState.xstarsqp[idx_Aineq],
            &b_TrialState.xstar[0]);
  ::coder::optim::coder::qpactiveset::driver(
      Hessian, grad, b_TrialState, memspace, WorkingSet, b_QRManager,
      b_CholManager, QPObjective, qpoptions, qpoptions);
  idx_Aineq = static_cast<unsigned char>(nVar);
  for (int idx{0}; idx < idx_Aineq; idx++) {
    double oldDirIdx;
    c = b_TrialState.socDirection[idx];
    oldDirIdx = c;
    c = b_TrialState.xstar[idx] - c;
    b_TrialState.socDirection[idx] = c;
    b_TrialState.xstar[idx] = oldDirIdx;
  }
  success = (::coder::internal::blas::xnrm2(nVar, b_TrialState.socDirection) <=
             2.0 * ::coder::internal::blas::xnrm2(nVar, b_TrialState.xstar));
  idx_Partition = WorkingSet.sizes[3];
  for (int idx{0}; idx < 80; idx++) {
    WorkingSet.bineq[idx] = -b_TrialState.cIneq[idx];
  }
  if (!success) {
    idx_Aineq = (WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1]) + 1;
    idx_lower = WorkingSet.nActiveConstr;
    for (int idx{idx_Aineq}; idx <= idx_lower; idx++) {
      WorkingSet
          .isActiveConstr[(WorkingSet.isActiveIdx[WorkingSet.Wid[idx - 1] - 1] +
                           WorkingSet.Wlocalidx[idx - 1]) -
                          2] = false;
    }
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
    for (int idx{0}; idx < nWIneq_old; idx++) {
      idx_Aineq = b_TrialState.workingset_old[idx];
      WorkingSet.nWConstr[2]++;
      WorkingSet.isActiveConstr[(WorkingSet.isActiveIdx[2] + idx_Aineq) - 2] =
          true;
      WorkingSet.nActiveConstr++;
      WorkingSet.Wid[WorkingSet.nActiveConstr - 1] = 3;
      WorkingSet.Wlocalidx[WorkingSet.nActiveConstr - 1] = idx_Aineq;
      idx_lower = 101 * (idx_Aineq - 1);
      idx_upper = 101 * (WorkingSet.nActiveConstr - 1);
      i = WorkingSet.nVar;
      for (int iac{0}; iac < i; iac++) {
        WorkingSet.ATwset[idx_upper + iac] = WorkingSet.Aineq[idx_lower + iac];
      }
      WorkingSet.bwset[WorkingSet.nActiveConstr - 1] =
          WorkingSet.bineq[idx_Aineq - 1];
    }
    for (int idx{0}; idx < nWLower_old; idx++) {
      qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
          WorkingSet, 4, b_TrialState.workingset_old[idx + 80]);
    }
    for (int idx{0}; idx < nWUpper_old; idx++) {
      qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
          WorkingSet, 5,
          b_TrialState.workingset_old[(idx + idx_Partition) + 80]);
    }
    std::copy(&b_TrialState.lambdaStopTest[0],
              &b_TrialState.lambdaStopTest[201], &b_TrialState.lambda[0]);
  } else {
    qpactiveset::parseoutput::sortLambdaQP(
        b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
        WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
        memspace.workspace_float);
  }
  return success;
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (soc.cpp)
