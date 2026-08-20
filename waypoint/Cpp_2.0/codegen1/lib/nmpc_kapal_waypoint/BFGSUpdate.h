//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BFGSUpdate.h
//
// Code generation for function 'BFGSUpdate'
//

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool BFGSUpdate(int nvar, double Bk[400], const double sk[101], double yk[101],
                double workspace[20301]);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (BFGSUpdate.h)
