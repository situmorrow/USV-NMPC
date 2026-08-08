//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include files
#include "main.h"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Declarations
static void argInit_30x1_real_T(double result[30]);

static void argInit_5x1_real_T(double result[5]);

static double argInit_real_T();

// Function Definitions
static void argInit_30x1_real_T(double result[30])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 30; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_5x1_real_T(double result[5])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 5; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

int main(int, char **)
{
  // Initialize the application.
  // You do not need to do this more than one time.
  nmpc_kapal_waypoint_initialize();
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_nmpc_kapal_waypoint();
  // Terminate the application.
  // You do not need to do this more than one time.
  nmpc_kapal_waypoint_terminate();
  return 0;
}

void main_nmpc_kapal_waypoint()
{
  double x_ref_seq_tmp[30];
  double dv[5];
  double exitflag;
  double u_opt;
  // Initialize function 'nmpc_kapal_waypoint' input arguments.
  // Initialize function input argument 'current_state_nd'.
  // Initialize function input argument 'x_ref_seq'.
  argInit_30x1_real_T(x_ref_seq_tmp);
  // Initialize function input argument 'y_ref_seq'.
  // Initialize function input argument 'psi_ref_seq'.
  // Call the entry-point 'nmpc_kapal_waypoint'.
  argInit_5x1_real_T(dv);
  nmpc_kapal_waypoint(dv, argInit_real_T(), x_ref_seq_tmp, x_ref_seq_tmp,
                      x_ref_seq_tmp, &u_opt, &exitflag);
}

// End of code generation (main.cpp)
