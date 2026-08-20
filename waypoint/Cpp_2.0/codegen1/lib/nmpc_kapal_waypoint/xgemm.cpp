//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemm.cpp
//
// Code generation for function 'xgemm'
//

// Include files
#include "xgemm.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemm(int m, int n, int k, const double A[400], int lda,
           const double B[40401], int ib0, double C[20301])
{
  if ((m != 0) && (n != 0)) {
    int ar;
    int br;
    int i;
    int lastColC;
    br = ib0;
    lastColC = 201 * (n - 1);
    for (int cr{0}; cr <= lastColC; cr += 201) {
      ar = cr + 1;
      i = cr + m;
      if (ar <= i) {
        std::memset(&C[ar + -1], 0,
                    static_cast<unsigned int>((i - ar) + 1) * sizeof(double));
      }
    }
    for (int cr{0}; cr <= lastColC; cr += 201) {
      ar = -1;
      i = br + k;
      for (int ib{br}; ib < i; ib++) {
        int i1;
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (int ic{i1}; ic <= i2; ic++) {
          C[ic - 1] += B[ib - 1] * A[(ar + ic) - cr];
        }
        ar += lda;
      }
      br += 201;
    }
  }
}

void xgemm(int m, int n, int k, const double A[40401], int ia0,
           const double B[20301], double C[40401])
{
  if ((m != 0) && (n != 0)) {
    int ar;
    int br;
    int lastColC;
    lastColC = 201 * (n - 1);
    for (int cr{0}; cr <= lastColC; cr += 201) {
      br = cr + 1;
      ar = cr + m;
      if (br <= ar) {
        std::memset(&C[br + -1], 0,
                    static_cast<unsigned int>((ar - br) + 1) * sizeof(double));
      }
    }
    br = -1;
    for (int cr{0}; cr <= lastColC; cr += 201) {
      int i;
      int i1;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (int ic{i}; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (int w{0}; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }
        C[ic - 1] += temp;
        ar += 201;
      }
      br += 201;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemm.cpp)
