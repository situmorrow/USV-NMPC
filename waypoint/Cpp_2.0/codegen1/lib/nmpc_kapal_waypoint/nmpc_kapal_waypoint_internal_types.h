//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal_waypoint_internal_types.h
//
// Code generation for function 'nmpc_kapal_waypoint'
//

#ifndef NMPC_KAPAL_WAYPOINT_INTERNAL_TYPES_H
#define NMPC_KAPAL_WAYPOINT_INTERNAL_TYPES_H

// Include files
#include "anonymous_function.h"
#include "nmpc_kapal_waypoint_types.h"
#include "rtwtypes.h"

// Type Definitions
struct struct_T {
  double grad[101];
  double Hx[100];
  bool hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  bool prev_hasLinear;
  double gammaScalar;
};

struct b_struct_T {
  double workspace_float[20301];
  int workspace_int[201];
  int workspace_sort[201];
};

struct c_struct_T {
  int ldq;
  double QR[40401];
  double Q[40401];
  int jpvt[201];
  int mrows;
  int ncols;
  double tau[201];
  int minRowCol;
  bool usedPivoting;
};

struct f_struct_T {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[20];
  double xstarsqp_old[20];
  double cIneq[80];
  double cIneq_old[80];
  double grad[101];
  double grad_old[101];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[201];
  double lambdaStopTest[201];
  double lambdaStopTestPrev[201];
  double steplength;
  double delta_x[101];
  double socDirection[101];
  int workingset_old[201];
  double JacCineqTrans_old[4040];
  double gradLag[101];
  double delta_gradLag[101];
  double xstar[101];
  double fstar;
  double firstorderopt;
  double lambda[201];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[101];
};

struct g_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double Aineq[8080];
  double bineq[80];
  double lb[101];
  double ub[101];
  int indexLB[101];
  int indexUB[101];
  int indexFixed[101];
  int mEqRemoved;
  double ATwset[20301];
  double bwset[201];
  int nActiveConstr;
  double maxConstrWorkspace[201];
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  bool isActiveConstr[201];
  int Wid[201];
  int Wlocalidx[201];
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct h_struct_T {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpDualFeasError;
  double nlpComplError;
};

struct i_struct_T {
  double FMat[40401];
  int ndims;
  int info;
  bool ConvexCheck;
  double workspace_;
  double workspace2_;
};

struct j_struct_T {
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
};

struct k_struct_T {
  char SolverName[7];
  int MaxIterations;
};

struct l_struct_T {
  coder::b_anonymous_function objfun;
  coder::anonymous_function nonlin;
  double f_1;
  double cIneq_1[40];
  int numEvals;
  bool hasBounds;
};

#endif
// End of code generation (nmpc_kapal_waypoint_internal_types.h)
