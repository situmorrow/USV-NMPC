//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.cpp
//
// Code generation for function 'driver'
//

// Include files
#include "driver.h"
#include "PresolveWorkingSet.h"
#include "iterate.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "nmpc_kapal_waypoint_internal_types.h"
#include "phaseone.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[900], const double f[151], f_struct_T &solution,
            b_struct_T &memspace, g_struct_T &workingset, c_struct_T &qrmanager,
            i_struct_T &cholmanager, struct_T &objective,
            const k_struct_T &b_options, const k_struct_T &runTimeOptions)
{
  int ixlast;
  int nVar;
  bool guard1;
  solution.iterations = 0;
  nVar = workingset.nVar;
  guard1 = false;
  if (workingset.probType == 3) {
    ixlast = static_cast<unsigned char>(workingset.sizes[3]);
    for (int idx{0}; idx < ixlast; idx++) {
      if (workingset.isActiveConstr[(workingset.isActiveIdx[3] + idx) - 1]) {
        solution.xstar[workingset.indexLB[idx] - 1] =
            -workingset.lb[workingset.indexLB[idx] - 1];
      }
    }
    for (int idx{0}; idx < 30; idx++) {
      if (workingset.isActiveConstr[(workingset.isActiveIdx[4] + idx) - 1]) {
        solution.xstar[workingset.indexUB[idx] - 1] =
            workingset.ub[workingset.indexUB[idx] - 1];
      }
    }
    initialize::PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution.state >= 0) {
      guard1 = true;
    }
  } else {
    solution.state = 82;
    guard1 = true;
  }
  if (guard1) {
    solution.iterations = 0;
    solution.maxConstr =
        WorkingSet::b_maxConstraintViolation(workingset, solution.xstar);
    if (solution.maxConstr > 1.0E-6) {
      phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
               objective, b_options.SolverName, runTimeOptions);
      if (solution.state != 0) {
        solution.maxConstr =
            WorkingSet::b_maxConstraintViolation(workingset, solution.xstar);
        if (solution.maxConstr > 1.0E-6) {
          double maxConstr_new;
          std::memset(&solution.lambda[0], 0, 301U * sizeof(double));
          maxConstr_new = 0.0;
          switch (objective.objtype) {
          case 5:
            maxConstr_new =
                objective.gammaScalar * solution.xstar[objective.nvar - 1];
            break;
          case 3:
            Objective::linearForm_(objective.hasLinear, objective.nvar,
                                   memspace.workspace_float, H, f,
                                   solution.xstar);
            if (objective.nvar >= 1) {
              ixlast = objective.nvar;
              for (int idx{0}; idx < ixlast; idx++) {
                maxConstr_new +=
                    solution.xstar[idx] * memspace.workspace_float[idx];
              }
            }
            break;
          case 4:
            Objective::linearForm_(objective.hasLinear, objective.nvar,
                                   memspace.workspace_float, H, f,
                                   solution.xstar);
            ixlast = objective.nvar + 1;
            for (int idx{ixlast}; idx < 151; idx++) {
              memspace.workspace_float[idx - 1] =
                  0.5 * objective.beta * solution.xstar[idx - 1] +
                  objective.rho;
            }
            for (int idx{0}; idx < 150; idx++) {
              maxConstr_new +=
                  solution.xstar[idx] * memspace.workspace_float[idx];
            }
            break;
          }
          solution.fstar = maxConstr_new;
          solution.state = -2;
        } else {
          if (solution.maxConstr > 0.0) {
            double maxConstr_new;
            if (nVar - 1 >= 0) {
              std::copy(&solution.xstar[0], &solution.xstar[nVar],
                        &solution.searchDir[0]);
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager);
            maxConstr_new = WorkingSet::b_maxConstraintViolation(
                workingset, solution.xstar);
            if (maxConstr_new >= solution.maxConstr) {
              solution.maxConstr = maxConstr_new;
              if (nVar - 1 >= 0) {
                std::copy(&solution.searchDir[0], &solution.searchDir[nVar],
                          &solution.xstar[0]);
              }
            }
          }
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, b_options.SolverName,
                  runTimeOptions.MaxIterations);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, b_options.SolverName, runTimeOptions.MaxIterations);
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (driver.cpp)
