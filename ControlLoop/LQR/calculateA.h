#ifndef CALCULATE_A_H
#define CALCULATE_A_H

#include "../Matrix.h"
#include "../Vector.h"

// Computes the A matrix (12x12) from linearization
Matrix calculateA(
    double m,
    double f,
    double cDrag,
    double areaVar,
    const Vector& full_state,   // size 12
    const Vector& full_input,   // size 7
    const Vector& rc,           // size 3
    const Vector& rt,           // size 3
    const Matrix& inertia,      // size 3x3
    // const Matrix& inertia_s,
    const Matrix& inertia_a,
    const Matrix& inertia_b,
    const Vector& angular_states
);

#endif // CALCULATE_A_H
