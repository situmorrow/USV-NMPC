//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal_internal_types1.h
//
// Code generation for function 'nmpc_kapal_internal_types1'
//

#ifndef NMPC_KAPAL_INTERNAL_TYPES1_H
#define NMPC_KAPAL_INTERNAL_TYPES1_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct d_struct_T {
  double current_state_nd[5];
  double x_ref_seq[30];
  double y_ref_seq[30];
  double psi_ref_seq[30];
};

struct e_struct_T {
  double current_state_nd[5];
};

#endif
// End of code generation (nmpc_kapal_internal_types1.h)
