#ifndef NEWTONKLEINMAN_H
#define NEWTONKLEINMAN_H

#include <stdexcept>
#include "lqr.h"

LQR lqrcontroller;

Matrix solveDiscreteLyapunov(const Matrix &A_cl, const Matrix &F, double tol, int maxIter);
Matrix newtonKleinmanDARE(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, double tol = 1e-8, int maxOuter = 100);


#endif