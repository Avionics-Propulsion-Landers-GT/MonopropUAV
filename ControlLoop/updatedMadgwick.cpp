#include "Madgwick.h"
#include <math.h>
#include "Vector.h"

// === Constructor ===
Madgwick::Madgwick(double setGain, double setBeta, double startTime, const Vector& initialOrientation)
    : gain(setGain), beta(setBeta), lastUpdateTime(startTime),
      orientation(initialOrientation), 
      eulerAttitudeEstimation(quaternionToEuler(initialOrientation)) {}

// === Public Update Function ===
Vector Madgwick::update(const Vector& updateArr) {
    double dt = updateArr(0, 0) - lastUpdateTime;
    lastUpdateTime = updateArr(0, 0);

    Vector accel(3, 0.0), gyro(3, 0.0), mag(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        accel(i, 0) = updateArr(1 + i, 0);
        gyro(i, 0)  = updateArr(4 + i, 0);
        mag(i, 0)   = updateArr(7 + i, 0);
    }

    orientation = madgwickUpdate(orientation, gyro, accel, mag, dt);
    eulerAttitudeEstimation = quaternionToEuler(orientation);
    return eulerAttitudeEstimation;
}

// === Madgwick Core Update ===
Vector Madgwick::madgwickUpdate(const Vector& q, const Vector& gyro, const Vector& accel, const Vector& mag, double dt) {
    Vector normAccel = accel.normalize();
    Vector normMag = mag.normalize();

    Vector step = computeGradient(q, normAccel, normMag).normalize();

    Vector qConj(4, 0.0);
    qConj(0,0) = q(0,0);
    qConj(1,0) = -q(1,0);
    qConj(2,0) = -q(2,0);
    qConj(3,0) = -q(3,0);

    Vector integration = quaternionMultiply(qConj, step).multiply(-2.0 * dt * gain);

    Vector gyroQ(4, 0.0);
    gyroQ(0,0) = integration(0,0);
    gyroQ(1,0) = gyro(0,0) + integration(1,0);
    gyroQ(2,0) = gyro(1,0) + integration(2,0);
    gyroQ(3,0) = gyro(2,0) + integration(3,0);

    Vector qGyroQ = quaternionMultiply(q, gyroQ).multiply(0.5);
    Vector stepScaled = step.multiply(-beta);

    Vector qdot = qGyroQ.add(stepScaled);
    Vector updatedQ(4, 0.0);
    for (int i = 0; i < 4; ++i) {
        updatedQ(i, 0) = q(i, 0) + qdot(i, 0) * dt;
    }

    return updatedQ.normalize();
}

// === Helper: Normalize ===
void Madgwick::normalizeArray(Vector& array) {
    array = array.normalize();  // replace in-place if needed
}

// === Helper: Magnitude ===
double Madgwick::magnitude(const Vector& array) {
    return array.magnitude();
}

// === Compute Gradient ===
Vector Madgwick::computeGradient(const Vector& q, const Vector& accel, const Vector& mag) {
    Vector conj(4, 0.0);
    conj(0,0) = q(0,0);
    conj(1,0) = -q(1,0);
    conj(2,0) = -q(2,0);
    conj(3,0) = -q(3,0);

    Vector qRef(4, 0.0);
    qRef(0,0) = 0;
    qRef(1,0) = mag(0,0);
    qRef(2,0) = mag(1,0);
    qRef(3,0) = mag(2,0);

    Vector conjRef = quaternionMultiply(conj, qRef);
    Vector h = quaternionMultiply(conjRef, q);

    Vector realH(3, 0.0);
    for (int i = 0; i < 3; ++i) realH(i, 0) = h(i+1, 0);

    Vector b(4, 0.0);
    b(0,0) = 0;
    b(1,0) = realH.magnitude();
    b(2,0) = 0;
    b(3,0) = h(3,0);

    Vector grad(4, 0.0);
    grad(0,0) = 2 * (q(1,0) * q(3,0) - q(0,0) * q(2,0)) - accel(0,0);
    grad(1,0) = 2 * (q(0,0) * q(1,0) + q(2,0) * q(3,0)) - accel(1,0);
    grad(2,0) = 2 * (0.5 - q(1,0) * q(1,0) - q(2,0) * q(2,0)) - accel(2,0);
    grad(3,0) = 2 * b(1,0) * (0.5 - q(2,0) * q(2,0) - q(3,0) * q(3,0)) + 
                2 * b(3,0) * (q(1,0) * q(3,0) - q(0,0) * q(2,0)) - mag(0,0);
    return grad;
}

// === Quaternion → Euler ===
Vector Madgwick::quaternionToEuler(const Vector& q) {
    Vector euler(3, 0.0);
    euler(0,0) = atan2(2 * (q(0,0) * q(1,0) + q(2,0) * q(3,0)),
                       1 - 2 * (q(1,0) * q(1,0) + q(2,0) * q(2,0)));
    euler(1,0) = asin(2 * (q(0,0) * q(2,0) - q(3,0) * q(1,0)));
    euler(2,0) = atan2(2 * (q(0,0) * q(3,0) + q(1,0) * q(2,0)),
                       1 - 2 * (q(2,0) * q(2,0) + q(3,0) * q(3,0)));
    return euler;
}

// === Euler → Quaternion ===
Vector Madgwick::eulerToQuaternion(const Vector& euler) {
    double roll  = euler(0,0);
    double pitch = euler(1,0);
    double yaw   = euler(2,0);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Vector q(4, 0.0);
    q(0,0) = cr * cp * cy + sr * sp * sy;
    q(1,0) = sr * cp * cy - cr * sp * sy;
    q(2,0) = cr * sp * cy + sr * cp * sy;
    q(3,0) = cr * cp * sy - sr * sp * cy;
    return q;
}

// === Quaternion Multiply ===
Vector Madgwick::quaternionMultiply(const Vector& q1, const Vector& q2) {
    Vector result(4, 0.0);
    result(0,0) = q1(0,0) * q2(0,0) - q1(1,0) * q2(1,0) - q1(2,0) * q2(2,0) - q1(3,0) * q2(3,0);
    result(1,0) = q1(0,0) * q2(1,0) + q1(1,0) * q2(0,0) + q1(2,0) * q2(3,0) - q1(3,0) * q2(2,0);
    result(2,0) = q1(0,0) * q2(2,0) - q1(1,0) * q2(3,0) + q1(2,0) * q2(0,0) + q1(3,0) * q2(1,0);
    result(3,0) = q1(0,0) * q2(3,0) + q1(1,0) * q2(2,0) - q1(2,0) * q2(1,0) + q1(3,0) * q2(0,0);
    return result;
}
