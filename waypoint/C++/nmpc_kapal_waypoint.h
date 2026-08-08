//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal_waypoint.h
//
// Code generation for function 'nmpc_kapal_waypoint'
//

#ifndef NMPC_KAPAL_WAYPOINT_H
#define NMPC_KAPAL_WAYPOINT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void nmpc_kapal_waypoint(const double current_state_nd[5], double u_prev,
                                const double x_ref_seq[30],
                                const double y_ref_seq[30],
                                const double psi_ref_seq[30], double *u_opt,
                                double *exitflag);

double nmpc_kapal_waypoint_anonFcn1(const double current_state_nd[5],
                                    const double x_ref_seq[30],
                                    const double y_ref_seq[30],
                                    const double psi_ref_seq[30],
                                    const double U[30]);

void nmpc_kapal_waypoint_anonFcn2(const double current_state_nd[5],
                                  const double U[30], double varargout_1[60]);

#endif
// End of code generation (nmpc_kapal_waypoint.h)
