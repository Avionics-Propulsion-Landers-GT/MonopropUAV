#ifndef MADGWICK_H
#define MADGWICK_H

#include "Vector.h"

/*
    Header file for updatedMadgwick.cpp filter — STL-Free
*/

class Madgwick {
private:
    double gain;
    double beta;
    float lastUpdateTime;

    Vector eulerAttitudeEstimation;  // size = 3
    Vector orientation;              // size = 4 (quaternion)

public:
    Madgwick(double setGain, double setBeta, double startTime, const Vector& initialOrientation);

    Vector update(const Vector& updateArr);

    Vector madgwickUpdate(const Vector& q,
                          const Vector& gyro,
                          const Vector& accel,
                          const Vector& mag,
                          double dt);

    void normalizeArray(Vector& array);
    double magnitude(const Vector& array);

    Vector computeGradient(const Vector& q,
                           const Vector& accel,
                           const Vector& mag);

    Vector quaternionToEuler(const Vector& quaternion);
    Vector eulerToQuaternion(const Vector& euler);
    Vector quaternionMultiply(const Vector& q1, const Vector& q2);
};

#endif // MADGWICK_H
