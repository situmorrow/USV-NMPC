//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkStoppingAndUpdateFval.cpp
//
// Code generation for function 'checkStoppingAndUpdateFval'
//

// Include files
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "nmpc_kapal_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Declarations
static int div_nzp_s32(int numerator);

// Function Definitions
static int div_nzp_s32(int numerator)
{
  int quotient;
  unsigned int tempAbsQuotient;
  if (numerator < 0) {
    tempAbsQuotient = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    tempAbsQuotient = static_cast<unsigned int>(numerator);
  }
  tempAbsQuotient /= 50U;
  if (numerator < 0) {
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace stopping {
void b_checkStoppingAndUpdateFval(int &activeSetChangeID, f_struct_T &solution,
                                  b_struct_T &memspace,
                                  const struct_T &objective,
                                  g_struct_T &workingset, c_struct_T &qrmanager,
                                  int runTimeOptions_MaxIterations,
                                  bool &updateFval)
{
  int nVar;
  solution.iterations++;
  nVar = objective.nvar;
  if ((solution.iterations >= runTimeOptions_MaxIterations) &&
      ((solution.state != 1) || (objective.objtype == 5))) {
    solution.state = 0;
  }
  if (solution.iterations - div_nzp_s32(solution.iterations) * 50 == 0) {
    double tempMaxConstr;
    tempMaxConstr =
        WorkingSet::b_maxConstraintViolation(workingset, solution.xstar);
    solution.maxConstr = tempMaxConstr;
    if (objective.objtype == 5) {
      tempMaxConstr = solution.maxConstr - solution.xstar[objective.nvar - 1];
    }
    if (tempMaxConstr > 1.0E-6) {
      bool nonDegenerateWset;
      if (nVar - 1 >= 0) {
        std::copy(&solution.xstar[0], &solution.xstar[nVar],
                  &solution.searchDir[0]);
      }
      nonDegenerateWset = initialize::feasibleX0ForWorkingSet(
          memspace.workspace_float, solution.searchDir, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution.state != 0)) {
        solution.state = -2;
      }
      activeSetChangeID = 0;
      tempMaxConstr =
          WorkingSet::b_maxConstraintViolation(workingset, solution.searchDir);
      if (tempMaxConstr < solution.maxConstr) {
        if (nVar - 1 >= 0) {
          std::copy(&solution.searchDir[0], &solution.searchDir[nVar],
                    &solution.xstar[0]);
        }
        solution.maxConstr = tempMaxConstr;
      }
    }
  }
  if (updateFval) {
    updateFval = false;
  }
}

void checkStoppingAndUpdateFval(int &activeSetChangeID, const double f[151],
                                f_struct_T &solution, b_struct_T &memspace,
                                const struct_T &objective,
                                g_struct_T &workingset, c_struct_T &qrmanager,
                                int runTimeOptions_MaxIterations,
                                const bool &updateFval)
{
  int nVar;
  solution.iterations++;
  nVar = objective.nvar;
  if ((solution.iterations >= runTimeOptions_MaxIterations) &&
      ((solution.state != 1) || (objective.objtype == 5))) {
    solution.state = 0;
  }
  if (solution.iterations - div_nzp_s32(solution.iterations) * 50 == 0) {
    double tempMaxConstr;
    tempMaxConstr =
        WorkingSet::b_maxConstraintViolation(workingset, solution.xstar);
    solution.maxConstr = tempMaxConstr;
    if (objective.objtype == 5) {
      tempMaxConstr = solution.maxConstr - solution.xstar[objective.nvar - 1];
    }
    if (tempMaxConstr > 1.0E-6) {
      bool nonDegenerateWset;
      if (nVar - 1 >= 0) {
        std::copy(&solution.xstar[0], &solution.xstar[nVar],
                  &solution.searchDir[0]);
      }
      nonDegenerateWset = initialize::feasibleX0ForWorkingSet(
          memspace.workspace_float, solution.searchDir, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution.state != 0)) {
        solution.state = -2;
      }
      activeSetChangeID = 0;
      tempMaxConstr =
          WorkingSet::b_maxConstraintViolation(workingset, solution.searchDir);
      if (tempMaxConstr < solution.maxConstr) {
        if (nVar - 1 >= 0) {
          std::copy(&solution.searchDir[0], &solution.searchDir[nVar],
                    &solution.xstar[0]);
        }
        solution.maxConstr = tempMaxConstr;
      }
    }
  }
  if (updateFval) {
    solution.fstar = Objective::computeFval_ReuseHx(
        objective, memspace.workspace_float, f, solution.xstar);
    if ((solution.fstar < 1.0E-6) &&
        ((solution.state != 0) || (objective.objtype != 5))) {
      solution.state = 2;
    }
  }
}

} // namespace stopping
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (checkStoppingAndUpdateFval.cpp)
