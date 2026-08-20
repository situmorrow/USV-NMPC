//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// soc.h
//
// Code generation for function 'soc'
//

#ifndef SOC_H
#define SOC_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;

struct b_struct_T;

struct g_struct_T;

struct c_struct_T;

struct i_struct_T;

struct struct_T;

struct k_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
bool b_soc(const double Hessian[400], const double grad[101],
           f_struct_T &b_TrialState, b_struct_T &memspace,
           g_struct_T &WorkingSet, c_struct_T &b_QRManager,
           i_struct_T &b_CholManager, struct_T &QPObjective,
           const k_struct_T &qpoptions);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (soc.h)
