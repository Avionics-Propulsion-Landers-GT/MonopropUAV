//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// calculateA_terminate.cpp
//
// Code generation for function 'calculateA_terminate'
//

// Include files
#include "calculateA_terminate.h"
#include "_coder_calculateA_mex.h"
#include "calculateA_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void emlrtExitTimeCleanupDtorFcn(const void *r);

// Function Definitions
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void calculateA_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  try {
    emlrtPushHeapReferenceStackR2021a(&st, false, nullptr,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      nullptr, nullptr, nullptr);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

void calculateA_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (calculateA_terminate.cpp)
