//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleratiotest.h
//
// Code generation for function 'feasibleratiotest'
//

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
double feasibleratiotest(
    const double solution_xstar[101], const double solution_searchDir[101],
    double workspace[20301], int workingset_nVar,
    const double workingset_Aineq[8080], const double workingset_bineq[80],
    const double workingset_lb[101], const double workingset_ub[101],
    const int workingset_indexLB[101], const int workingset_indexUB[101],
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const bool workingset_isActiveConstr[201], const int workingset_nWConstr[5],
    bool isPhaseOne, bool &newBlocking, int &constrType, int &constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (feasibleratiotest.h)
