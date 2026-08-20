//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGradLag.h
//
// Code generation for function 'computeGradLag'
//

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void b_computeGradLag(double workspace[20301], int nVar, const double grad[101],
                      const double AineqTrans[8080], const int finiteLB[101],
                      int mLB, const int finiteUB[101],
                      const double lambda[201]);

void computeGradLag(double workspace[101], int nVar, const double grad[101],
                    const double AineqTrans[8080], const int finiteLB[101],
                    int mLB, const int finiteUB[101], const double lambda[201]);

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeGradLag.h)
