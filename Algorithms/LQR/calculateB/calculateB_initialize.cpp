//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// calculateB_initialize.cpp
//
// Code generation for function 'calculateB_initialize'
//

// Include files
#include "calculateB_initialize.h"
#include "_coder_calculateB_mex.h"
#include "calculateB_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void calculateB_once();

// Function Definitions
static void calculateB_once()
{
  mex_InitInfAndNan();
}

void calculateB_initialize()
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
    calculateB_once();
  }
}

// End of code generation (calculateB_initialize.cpp)
