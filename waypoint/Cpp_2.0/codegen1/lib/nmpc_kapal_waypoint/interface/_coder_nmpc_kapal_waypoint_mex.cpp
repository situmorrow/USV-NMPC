//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_nmpc_kapal_waypoint_mex.cpp
//
// Code generation for function 'nmpc_kapal_waypoint'
//

// Include files
#include "_coder_nmpc_kapal_waypoint_mex.h"
#include "_coder_nmpc_kapal_waypoint_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&nmpc_kapal_waypoint_atexit);
  nmpc_kapal_waypoint_initialize();
  unsafe_nmpc_kapal_waypoint_mexFunction(nlhs, plhs, nrhs, prhs);
  nmpc_kapal_waypoint_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_nmpc_kapal_waypoint_mexFunction(int32_T nlhs, mxArray *plhs[2],
                                            int32_T nrhs,
                                            const mxArray *prhs[6])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[6];
  const mxArray *outputs[2];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        19, "nmpc_kapal_waypoint");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 19,
                        "nmpc_kapal_waypoint");
  }
  // Call the function.
  for (int32_T i{0}; i < 6; i++) {
    b_prhs[i] = prhs[i];
  }
  nmpc_kapal_waypoint_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_nmpc_kapal_waypoint_mex.cpp)
