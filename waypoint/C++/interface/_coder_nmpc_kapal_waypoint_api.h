//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_nmpc_kapal_waypoint_api.h
//
// Code generation for function 'nmpc_kapal_waypoint'
//

#ifndef _CODER_NMPC_KAPAL_WAYPOINT_API_H
#define _CODER_NMPC_KAPAL_WAYPOINT_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void nmpc_kapal_waypoint(real_T current_state_nd[5], real_T u_prev,
                         real_T x_ref_seq[30], real_T y_ref_seq[30],
                         real_T psi_ref_seq[30], real_T *u_opt,
                         real_T *exitflag);

void nmpc_kapal_waypoint_api(const mxArray *const prhs[5], int32_T nlhs,
                             const mxArray *plhs[2]);

void nmpc_kapal_waypoint_atexit();

void nmpc_kapal_waypoint_initialize();

void nmpc_kapal_waypoint_terminate();

void nmpc_kapal_waypoint_xil_shutdown();

void nmpc_kapal_waypoint_xil_terminate();

#endif
// End of code generation (_coder_nmpc_kapal_waypoint_api.h)
