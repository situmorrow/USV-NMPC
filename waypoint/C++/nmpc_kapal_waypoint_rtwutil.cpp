//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nmpc_kapal_waypoint_rtwutil.cpp
//
// Code generation for function 'nmpc_kapal_waypoint_rtwutil'
//

// Include files
#include "nmpc_kapal_waypoint_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
int div_nzp_s32_floor(int numerator, int denominator)
{
  unsigned int absDenominator;
  unsigned int absNumerator;
  int quotient;
  unsigned int tempAbsQuotient;
  bool quotientNeedsNegation;
  if (numerator < 0) {
    absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    absNumerator = static_cast<unsigned int>(numerator);
  }
  if (denominator < 0) {
    absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
  } else {
    absDenominator = static_cast<unsigned int>(denominator);
  }
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

// End of code generation (nmpc_kapal_waypoint_rtwutil.cpp)
