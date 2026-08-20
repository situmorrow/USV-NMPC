//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fmincon.cpp
//
// Code generation for function 'fmincon'
//

// Include files
#include "fmincon.h"
#include "anonymous_function.h"
#include "computeConstraints_.h"
#include "computeFiniteDifferences.h"
#include "driver1.h"
#include "loadProblem.h"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "nmpc_kapal_waypoint_internal_types1.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "stickyStruct.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
double fmincon(const b_anonymous_function &fun, const double x0[20],
               const double Aineq[800], const double bineq[40],
               const anonymous_function nonlcon, double x[20], double &exitflag)
{
  static b_struct_T memspace;
  static c_struct_T obj;
  static g_struct_T WorkingSet;
  static i_struct_T CholManager;
  static const signed char iv[101]{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0};
  internal::i_stickyStruct FcnEvaluator;
  f_struct_T TrialState;
  h_struct_T MeritFunction;
  l_struct_T FiniteDifferences;
  struct_T QPObjective;
  double c;
  double fval;
  int idx;
  TrialState.nVarMax = 101;
  TrialState.mNonlinIneq = 40;
  TrialState.mNonlinEq = 0;
  TrialState.mIneq = 80;
  TrialState.mEq = 0;
  TrialState.iNonIneq0 = 41;
  TrialState.iNonEq0 = 1;
  TrialState.sqpFval_old = 0.0;
  TrialState.sqpIterations = 0;
  TrialState.sqpExitFlag = 0;
  std::memset(&TrialState.lambdasqp[0], 0, 201U * sizeof(double));
  TrialState.steplength = 1.0;
  std::memset(&TrialState.delta_x[0], 0, 101U * sizeof(double));
  TrialState.fstar = 0.0;
  TrialState.firstorderopt = 0.0;
  std::memset(&TrialState.lambda[0], 0, 201U * sizeof(double));
  TrialState.state = 0;
  TrialState.maxConstr = 0.0;
  TrialState.iterations = 0;
  std::copy(&x0[0], &x0[20], &TrialState.xstarsqp[0]);
  FcnEvaluator.next.next.next.next.next.next.next.next.value = fun;
  FcnEvaluator.next.next.next.next.next.next.next.value = nonlcon;
  WorkingSet.mConstr = 0;
  WorkingSet.nVar = 20;
  std::memset(&WorkingSet.Aineq[0], 0, 8080U * sizeof(double));
  std::memset(&WorkingSet.bineq[0], 0, 80U * sizeof(double));
  std::memset(&WorkingSet.lb[0], 0, 101U * sizeof(double));
  std::memset(&WorkingSet.ub[0], 0, 101U * sizeof(double));
  std::memset(&WorkingSet.ATwset[0], 0, 20301U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  std::memset(&WorkingSet.bwset[0], 0, 201U * sizeof(double));
  std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 201U * sizeof(double));
  for (int i{0}; i < 5; i++) {
    WorkingSet.sizes[i] = 0;
    WorkingSet.sizesNormal[i] = 0;
    WorkingSet.sizesPhaseOne[i] = 0;
    WorkingSet.sizesRegularized[i] = 0;
    WorkingSet.sizesRegPhaseOne[i] = 0;
  }
  for (int i{0}; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = 0;
    WorkingSet.isActiveIdxNormal[i] = 0;
    WorkingSet.isActiveIdxPhaseOne[i] = 0;
    WorkingSet.isActiveIdxRegularized[i] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[i] = 0;
  }
  std::memset(&WorkingSet.isActiveConstr[0], 0, 201U * sizeof(bool));
  std::memset(&WorkingSet.Wid[0], 0, 201U * sizeof(int));
  std::memset(&WorkingSet.Wlocalidx[0], 0, 201U * sizeof(int));
  for (int i{0}; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  for (int i{0}; i < 101; i++) {
    signed char b_i;
    b_i = iv[i];
    WorkingSet.indexLB[i] = b_i;
    WorkingSet.indexUB[i] = b_i;
  }
  optim::coder::qpactiveset::WorkingSet::loadProblem(WorkingSet, Aineq);
  for (int i{0}; i < 20; i++) {
    idx = WorkingSet.indexLB[i];
    TrialState.xstarsqp[idx - 1] =
        std::fmax(TrialState.xstarsqp[idx - 1], -0.78539816339744828);
  }
  for (int i{0}; i < 20; i++) {
    idx = WorkingSet.indexUB[i];
    TrialState.xstarsqp[idx - 1] =
        std::fmin(TrialState.xstarsqp[idx - 1], 0.78539816339744828);
  }
  std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[20], &x[0]);
  fval = nmpc_kapal_waypoint_anonFcn1(
      fun.workspace.current_state_nd, fun.workspace.x_ref_seq,
      fun.workspace.y_ref_seq, fun.workspace.psi_ref_seq, x);
  if ((!std::isinf(fval)) && (!std::isnan(fval))) {
    optim::coder::utils::ObjNonlinEvaluator::computeConstraints_(
        FcnEvaluator, TrialState.xstarsqp, TrialState.cIneq);
  }
  TrialState.sqpFval = fval;
  FiniteDifferences.objfun = fun;
  FiniteDifferences.nonlin = nonlcon;
  FiniteDifferences.f_1 = 0.0;
  FiniteDifferences.numEvals = 0;
  FiniteDifferences.hasBounds = true;
  optim::coder::utils::FiniteDifferences::computeFiniteDifferences(
      FiniteDifferences, fval, TrialState.cIneq, TrialState.xstarsqp,
      TrialState.grad, WorkingSet.Aineq);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  for (int i{0}; i < 40; i++) {
    TrialState.cIneq[i] = -bineq[i];
  }
  for (int iac{0}; iac <= 3939; iac += 101) {
    c = 0.0;
    idx = iac + 20;
    for (int i{iac + 1}; i <= idx; i++) {
      c += WorkingSet.Aineq[i - 1] * TrialState.xstarsqp[(i - iac) - 1];
    }
    idx = div_nzp_s32_floor(iac, 101);
    TrialState.cIneq[idx] += c;
  }
  for (int i{0}; i < 80; i++) {
    WorkingSet.bineq[i] = -TrialState.cIneq[i];
  }
  for (int i{0}; i < 20; i++) {
    WorkingSet.lb[WorkingSet.indexLB[i] - 1] =
        x0[WorkingSet.indexLB[i] - 1] + 0.78539816339744828;
    WorkingSet.ub[WorkingSet.indexUB[i] - 1] =
        0.78539816339744828 - x0[WorkingSet.indexUB[i] - 1];
  }
  optim::coder::qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
  idx = WorkingSet.isActiveIdx[2];
  for (int i{idx}; i < 202; i++) {
    WorkingSet.isActiveConstr[i - 1] = false;
  }
  WorkingSet.nWConstr[0] = 0;
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = 0;
  MeritFunction.initFval = fval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  fval = 0.0;
  for (int i{0}; i < 80; i++) {
    c = TrialState.cIneq[i];
    if (c > 0.0) {
      fval += c;
    }
  }
  double Hessian[400];
  MeritFunction.initConstrViolationIneq = fval;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  std::memset(&obj.Q[0], 0, 40401U * sizeof(double));
  std::memset(&obj.jpvt[0], 0, 201U * sizeof(int));
  obj.mrows = 0;
  obj.ncols = 0;
  obj.minRowCol = 0;
  obj.usedPivoting = false;
  optim::coder::fminconsqp::driver(
      bineq, TrialState, MeritFunction, FcnEvaluator, FiniteDifferences,
      memspace, WorkingSet, obj, Hessian, CholManager, QPObjective);
  std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[20], &x[0]);
  fval = TrialState.sqpFval;
  exitflag = TrialState.sqpExitFlag;
  return fval;
}

} // namespace coder

// End of code generation (fmincon.cpp)
