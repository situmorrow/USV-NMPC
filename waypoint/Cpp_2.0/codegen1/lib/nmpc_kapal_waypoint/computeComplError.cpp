//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeComplError.cpp
//
// Code generation for function 'computeComplError'
//

// Include files
#include "computeComplError.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
double computeComplError(const double xCurrent[20], const double cIneq[80],
                         const int finiteLB[101], int mLB,
                         const int finiteUB[101], const double lambda[201])
{
  double nlpComplError;
  double nlpComplError_tmp;
  double ubLambda;
  int i;
  nlpComplError = 0.0;
  for (int idx{0}; idx < 40; idx++) {
    ubLambda = cIneq[idx];
    nlpComplError_tmp = lambda[idx];
    nlpComplError =
        std::fmax(nlpComplError,
                  std::fmin(std::abs(ubLambda * nlpComplError_tmp),
                            std::fmin(std::abs(ubLambda), nlpComplError_tmp)));
  }
  for (int idx{0}; idx < 40; idx++) {
    ubLambda = cIneq[idx + 40];
    nlpComplError_tmp = lambda[idx + 40];
    nlpComplError =
        std::fmax(nlpComplError,
                  std::fmin(std::abs(ubLambda * nlpComplError_tmp),
                            std::fmin(std::abs(ubLambda), nlpComplError_tmp)));
  }
  i = static_cast<unsigned char>(mLB);
  for (int idx{0}; idx < i; idx++) {
    ubLambda = xCurrent[finiteLB[idx] - 1] - -0.78539816339744828;
    nlpComplError_tmp = lambda[idx + 80];
    nlpComplError =
        std::fmax(nlpComplError,
                  std::fmin(std::abs(ubLambda * nlpComplError_tmp),
                            std::fmin(std::abs(ubLambda), nlpComplError_tmp)));
  }
  for (int idx{0}; idx < 20; idx++) {
    ubLambda = lambda[(mLB + idx) + 80];
    nlpComplError_tmp = 0.78539816339744828 - xCurrent[finiteUB[idx] - 1];
    nlpComplError =
        std::fmax(nlpComplError,
                  std::fmin(std::abs(nlpComplError_tmp * ubLambda),
                            std::fmin(std::abs(nlpComplError_tmp), ubLambda)));
  }
  return nlpComplError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeComplError.cpp)
