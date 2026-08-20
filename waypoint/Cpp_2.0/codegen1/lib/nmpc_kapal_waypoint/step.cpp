//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// step.cpp
//
// Code generation for function 'step'
//

// Include files
#include "step.h"
#include "driver.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include "sortLambdaQP.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_step(j_struct_T &stepFlags, double Hessian[400],
            f_struct_T &b_TrialState, h_struct_T &b_MeritFunction,
            b_struct_T &memspace, g_struct_T &WorkingSet,
            c_struct_T &b_QRManager, i_struct_T &b_CholManager,
            struct_T &QPObjective, k_struct_T &qpoptions)
{
  double obj_maxConstrWorkspace[201];
  double b_dv[101];
  double violationResid;
  int idxStartIneq;
  int nVar;
  bool checkBoundViolation;
  stepFlags.stepAccepted = true;
  checkBoundViolation = true;
  nVar = WorkingSet.nVar - 1;
  if (stepFlags.stepType != 3) {
    idxStartIneq = static_cast<unsigned char>(WorkingSet.nVar);
    std::copy(&b_TrialState.xstarsqp[0], &b_TrialState.xstarsqp[idxStartIneq],
              &b_TrialState.xstar[0]);
  } else if (nVar >= 0) {
    std::copy(&b_TrialState.xstar[0], &b_TrialState.xstar[nVar + 1],
              &b_TrialState.searchDir[0]);
  }
  int exitg1;
  bool guard1;
  do {
    double nrmDirInf;
    int idxEndIneq;
    exitg1 = 0;
    guard1 = false;
    switch (stepFlags.stepType) {
    case 1: {
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, b_TrialState.grad, b_TrialState, memspace, WorkingSet,
          b_QRManager, b_CholManager, QPObjective, qpoptions, qpoptions);
      if (WorkingSet.probType == 2) {
        nrmDirInf = 0.0;
        std::copy(&WorkingSet.maxConstrWorkspace[0],
                  &WorkingSet.maxConstrWorkspace[201],
                  &obj_maxConstrWorkspace[0]);
        for (int idx_global{0}; idx_global < 80; idx_global++) {
          obj_maxConstrWorkspace[idx_global] = -WorkingSet.bineq[idx_global];
        }
        for (int idx_col{0}; idx_col <= 7979; idx_col += 101) {
          violationResid = 0.0;
          idxStartIneq = idx_col + 20;
          for (int idx_global{idx_col + 1}; idx_global <= idxStartIneq;
               idx_global++) {
            violationResid += WorkingSet.Aineq[idx_global - 1] *
                              b_TrialState.xstar[(idx_global - idx_col) - 1];
          }
          idxStartIneq = div_nzp_s32_floor(idx_col, 101);
          obj_maxConstrWorkspace[idxStartIneq] += violationResid;
        }
        for (int idx_global{0}; idx_global < 80; idx_global++) {
          violationResid = obj_maxConstrWorkspace[idx_global] -
                           b_TrialState.xstar[idx_global + 20];
          obj_maxConstrWorkspace[idx_global] = violationResid;
          nrmDirInf = std::fmax(nrmDirInf, violationResid);
        }
      } else {
        nrmDirInf = 0.0;
        std::copy(&WorkingSet.maxConstrWorkspace[0],
                  &WorkingSet.maxConstrWorkspace[201],
                  &obj_maxConstrWorkspace[0]);
        for (int idx_global{0}; idx_global < 80; idx_global++) {
          obj_maxConstrWorkspace[idx_global] = -WorkingSet.bineq[idx_global];
        }
        for (int idx_col{0}; idx_col <= 7979; idx_col += 101) {
          violationResid = 0.0;
          idxStartIneq = idx_col + WorkingSet.nVar;
          for (int idx_global{idx_col + 1}; idx_global <= idxStartIneq;
               idx_global++) {
            violationResid += WorkingSet.Aineq[idx_global - 1] *
                              b_TrialState.xstar[(idx_global - idx_col) - 1];
          }
          idxStartIneq = div_nzp_s32_floor(idx_col, 101);
          obj_maxConstrWorkspace[idxStartIneq] += violationResid;
        }
        for (int idx_global{0}; idx_global < 80; idx_global++) {
          nrmDirInf = std::fmax(nrmDirInf, obj_maxConstrWorkspace[idx_global]);
        }
      }
      idxStartIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
      for (int idx_global{0}; idx_global < idxStartIneq; idx_global++) {
        idxEndIneq = WorkingSet.indexLB[idx_global] - 1;
        nrmDirInf = std::fmax(nrmDirInf, -b_TrialState.xstar[idxEndIneq] -
                                             WorkingSet.lb[idxEndIneq]);
      }
      for (int idx_global{0}; idx_global < 20; idx_global++) {
        idxStartIneq = WorkingSet.indexUB[idx_global] - 1;
        nrmDirInf = std::fmax(nrmDirInf, b_TrialState.xstar[idxStartIneq] -
                                             WorkingSet.ub[idxStartIneq]);
      }
      if ((b_TrialState.state > 0) ||
          ((b_TrialState.state == 0) && (nrmDirInf <= 1.0E-6))) {
        double constrViolationIneq;
        double penaltyParamTrial;
        penaltyParamTrial = b_MeritFunction.penaltyParam;
        constrViolationIneq = 0.0;
        for (int idx_global{0}; idx_global < 80; idx_global++) {
          violationResid = b_TrialState.cIneq[idx_global];
          if (violationResid > 0.0) {
            constrViolationIneq += violationResid;
          }
        }
        violationResid = b_MeritFunction.linearizedConstrViol;
        b_MeritFunction.linearizedConstrViol = 0.0;
        nrmDirInf = constrViolationIneq + violationResid;
        if ((nrmDirInf > 2.2204460492503131E-16) &&
            (b_TrialState.fstar > 0.0)) {
          if (b_TrialState.sqpFval == 0.0) {
            violationResid = 1.0;
          } else {
            violationResid = 1.5;
          }
          penaltyParamTrial = violationResid * b_TrialState.fstar / nrmDirInf;
        }
        if (penaltyParamTrial < b_MeritFunction.penaltyParam) {
          b_MeritFunction.phi =
              b_TrialState.sqpFval + penaltyParamTrial * constrViolationIneq;
          if ((b_MeritFunction.initFval +
               penaltyParamTrial * b_MeritFunction.initConstrViolationIneq) -
                  b_MeritFunction.phi >
              static_cast<double>(b_MeritFunction.nPenaltyDecreases) *
                  b_MeritFunction.threshold) {
            b_MeritFunction.nPenaltyDecreases++;
            if ((b_MeritFunction.nPenaltyDecreases << 1) >
                b_TrialState.sqpIterations) {
              b_MeritFunction.threshold *= 10.0;
            }
            b_MeritFunction.penaltyParam =
                std::fmax(penaltyParamTrial, 1.0E-10);
          } else {
            b_MeritFunction.phi =
                b_TrialState.sqpFval +
                b_MeritFunction.penaltyParam * constrViolationIneq;
          }
        } else {
          b_MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          b_MeritFunction.phi =
              b_TrialState.sqpFval +
              b_MeritFunction.penaltyParam * constrViolationIneq;
        }
        b_MeritFunction.phiPrimePlus =
            std::fmin(b_TrialState.fstar -
                          b_MeritFunction.penaltyParam * constrViolationIneq,
                      0.0);
      } else if (b_TrialState.state != -6) {
        stepFlags.stepType = 2;
      }
      qpactiveset::parseoutput::sortLambdaQP(
          b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
          memspace.workspace_float);
      if (stepFlags.stepType != 2) {
        if (nVar >= 0) {
          std::copy(&b_TrialState.xstar[0], &b_TrialState.xstar[nVar + 1],
                    &b_TrialState.delta_x[0]);
        }
        guard1 = true;
      }
    } break;
    case 2:
      idxStartIneq = (WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1]) + 1;
      idxEndIneq = WorkingSet.nActiveConstr;
      for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq;
           idx_global++) {
        WorkingSet.isActiveConstr
            [(WorkingSet.isActiveIdx[WorkingSet.Wid[idx_global - 1] - 1] +
              WorkingSet.Wlocalidx[idx_global - 1]) -
             2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr =
          WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      std::copy(&b_TrialState.xstar[0], &b_TrialState.xstar[101], &b_dv[0]);
      idxStartIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
      for (int idx_global{0}; idx_global < idxStartIneq; idx_global++) {
        if (-b_dv[WorkingSet.indexLB[idx_global] - 1] >
            WorkingSet.lb[WorkingSet.indexLB[idx_global] - 1]) {
          b_dv[WorkingSet.indexLB[idx_global] - 1] =
              (WorkingSet.ub[WorkingSet.indexLB[idx_global] - 1] -
               WorkingSet.lb[WorkingSet.indexLB[idx_global] - 1]) /
              2.0;
        }
      }
      for (int idx_global{0}; idx_global < 20; idx_global++) {
        if (b_dv[WorkingSet.indexUB[idx_global] - 1] >
            WorkingSet.ub[WorkingSet.indexUB[idx_global] - 1]) {
          b_dv[WorkingSet.indexUB[idx_global] - 1] =
              (WorkingSet.ub[WorkingSet.indexUB[idx_global] - 1] -
               WorkingSet.lb[WorkingSet.indexUB[idx_global] - 1]) /
              2.0;
        }
      }
      std::copy(&b_dv[0], &b_dv[101], &b_TrialState.xstar[0]);
      step::b_relaxed(Hessian, b_TrialState.grad, b_TrialState, b_MeritFunction,
                      memspace, WorkingSet, b_QRManager, b_CholManager,
                      QPObjective, qpoptions);
      if (nVar >= 0) {
        std::copy(&b_TrialState.xstar[0], &b_TrialState.xstar[nVar + 1],
                  &b_TrialState.delta_x[0]);
      }
      guard1 = true;
      break;
    default:
      std::copy(&b_TrialState.grad[0], &b_TrialState.grad[101], &b_dv[0]);
      checkBoundViolation =
          step::b_soc(Hessian, b_dv, b_TrialState, memspace, WorkingSet,
                      b_QRManager, b_CholManager, QPObjective, qpoptions);
      stepFlags.stepAccepted = checkBoundViolation;
      if (stepFlags.stepAccepted && (b_TrialState.state != -6)) {
        idxStartIneq = static_cast<unsigned char>(nVar + 1);
        for (int idx_global{0}; idx_global < idxStartIneq; idx_global++) {
          b_TrialState.delta_x[idx_global] =
              b_TrialState.xstar[idx_global] +
              b_TrialState.socDirection[idx_global];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (b_TrialState.state != -6) {
        exitg1 = 1;
      } else {
        violationResid = 0.0;
        nrmDirInf = 1.0;
        for (int idx_global{0}; idx_global < 20; idx_global++) {
          violationResid = std::fmax(violationResid,
                                     std::abs(b_TrialState.grad[idx_global]));
          nrmDirInf =
              std::fmax(nrmDirInf, std::abs(b_TrialState.xstar[idx_global]));
        }
        violationResid =
            std::fmax(2.2204460492503131E-16, violationResid / nrmDirInf);
        for (int idx_col{0}; idx_col < 20; idx_col++) {
          idxStartIneq = 20 * idx_col;
          for (int idx_global{0}; idx_global < idx_col; idx_global++) {
            Hessian[idxStartIneq + idx_global] = 0.0;
          }
          idxStartIneq = idx_col + 20 * idx_col;
          Hessian[idxStartIneq] = violationResid;
          idxEndIneq = 18 - idx_col;
          if (idxEndIneq >= 0) {
            std::memset(&Hessian[idxStartIneq + 1], 0,
                        static_cast<unsigned int>(idxEndIneq + 1) *
                            sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    std::copy(&b_TrialState.delta_x[0], &b_TrialState.delta_x[101], &b_dv[0]);
    idxStartIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
    for (int idx_global{0}; idx_global < idxStartIneq; idx_global++) {
      violationResid =
          (b_TrialState.xstarsqp[WorkingSet.indexLB[idx_global] - 1] +
           b_dv[WorkingSet.indexLB[idx_global] - 1]) -
          -0.78539816339744828;
      if (violationResid < 0.0) {
        b_dv[WorkingSet.indexLB[idx_global] - 1] -= violationResid;
        b_TrialState.xstar[WorkingSet.indexLB[idx_global] - 1] -=
            violationResid;
      }
    }
    for (int idx_global{0}; idx_global < 20; idx_global++) {
      violationResid =
          (0.78539816339744828 -
           b_TrialState.xstarsqp[WorkingSet.indexUB[idx_global] - 1]) -
          b_dv[WorkingSet.indexUB[idx_global] - 1];
      if (violationResid < 0.0) {
        b_dv[WorkingSet.indexUB[idx_global] - 1] += violationResid;
        b_TrialState.xstar[WorkingSet.indexUB[idx_global] - 1] +=
            violationResid;
      }
    }
    std::copy(&b_dv[0], &b_dv[101], &b_TrialState.delta_x[0]);
  }
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (step.cpp)
