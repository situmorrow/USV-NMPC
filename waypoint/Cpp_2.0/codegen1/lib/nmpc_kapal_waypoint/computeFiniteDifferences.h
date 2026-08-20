//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFiniteDifferences.h
//
// Code generation for function 'computeFiniteDifferences'
//

#ifndef COMPUTEFINITEDIFFERENCES_H
#define COMPUTEFINITEDIFFERENCES_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct l_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
bool computeFiniteDifferences(l_struct_T &obj, double fCurrent,
                              const double cIneqCurrent[80], double xk[20],
                              double gradf[101], double JacCineqTrans[8080]);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeFiniteDifferences.h)
