//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// calculateA_initialize.cpp
//
// Code generation for function 'calculateA_initialize'
//

// Include files
#include "calculateA_initialize.h"
#include "_coder_calculateA_mex.h"
#include "calculateA_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void calculateA_once();

// Function Definitions
static void calculateA_once()
{
  mex_InitInfAndNan();
}

void calculateA_initialize()
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar{nullptr};
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    calculateA_once();
  }
}

// End of code generation (calculateA_initialize.cpp)
