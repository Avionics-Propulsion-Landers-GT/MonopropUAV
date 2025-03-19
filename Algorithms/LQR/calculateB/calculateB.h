//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// calculateB.h
//
// Code generation for function 'calculateB'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Function Declarations
void calculateB(const emlrtStack *sp, real_T m, real_T f, real_T cDrag,
                real_T areaVar, const real_T in5[12], const real_T in6[7],
                const real_T in7[3], const real_T in8[3], const real_T in9[9],
                const real_T in10[9], const real_T in11[9],
                const real_T in12[9], real_T B[84]);

// End of code generation (calculateB.h)
