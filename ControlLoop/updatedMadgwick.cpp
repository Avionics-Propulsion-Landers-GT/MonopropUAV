#include "Madgwick.h"
#include <cmath>
#include <iostream>

Madgwick::Madgwick(double setGain, double setBeta, double startTime, std::vector<double>& initialOrientation) {
    gain = setGain;
    beta = setBeta;
    lastUpdateTime = startTime;
    orientation = initialOrientation;
    eulerAttitudeEstimation = quaternionToEuler(orientation); // ✅ Ensure this function is implemented
}

std::vector<double> Madgwick::update(std::vector<double>& updateArr) {
    double dt = updateArr[0] - lastUpdateTime;
    lastUpdateTime = updateArr[0];

    std::vector<double> accel = {updateArr[1], updateArr[2], updateArr[3]};
    std::vector<double> gyro = {updateArr[4], updateArr[5], updateArr[6]};
    std::vector<double> mag = {updateArr[7], updateArr[8], updateArr[9]};

    orientation = madgwickUpdate(orientation, gyro, accel, mag, dt);
    eulerAttitudeEstimation = quaternionToEuler(orientation);

    return eulerAttitudeEstimation;
}

std::vector<double> Madgwick::madgwickUpdate(std::vector<double>& q, 
                                             std::vector<double>& gyro, 
                                             std::vector<double>& accel, 
                                             std::vector<double>& mag, 
                                             double dt) {
    normalizeArray(accel);
    normalizeArray(mag);

    std::vector<double> step = computeGradient(q, accel, mag);
    normalizeArray(step);

    std::vector<double> qConj = {q[0], -q[1], -q[2], -q[3]};
    std::vector<double> integration = quaternionMultiply(qConj, step);
    for (double& val : integration) val *= 2 * dt * gain * -1;

    std::vector<double> gyroQ = {integration[0], gyro[0] + integration[1], 
                                 gyro[1] + integration[2], gyro[2] + integration[3]};
    std::vector<double> qGyroQ = quaternionMultiply(q, gyroQ);
    for (double& val : qGyroQ) val *= 0.5;
    for (double& val : step) val *= -beta;

    std::vector<double> qdot = {qGyroQ[0] + step[0], qGyroQ[1] + step[1], 
                                qGyroQ[2] + step[2], qGyroQ[3] + step[3]};

    std::vector<double> updatedQ = {q[0] + qdot[0] * dt, q[1] + qdot[1] * dt, 
                                    q[2] + qdot[2] * dt, q[3] + qdot[3] * dt};
    normalizeArray(updatedQ);

    return updatedQ;
}

void Madgwick::normalizeArray(std::vector<double>& v) {
    double sumSquares = 0.0;
    for (double val : v) {
        sumSquares += val * val;
    }

    double norm = std::sqrt(sumSquares);
    if (norm < 1e-9) return;

    for (double& val : v) val /= norm;
}

double Madgwick::magnitude(std::vector<double>& array) {
    double total = 0;
    for (double value : array) {
        total += value * value;
    }
    return std::sqrt(total);
}

std::vector<double> Madgwick::computeGradient(std::vector<double>& q, 
                                              std::vector<double>& accel, 
                                              std::vector<double>& mag) {
    std::vector<double> conj = {q[0], -q[1], -q[2], -q[3]};
    std::vector<double> qRef = {0, mag[0], mag[1], mag[2]};
    std::vector<double> conjRef = quaternionMultiply(conj, qRef);
    std::vector<double> h = quaternionMultiply(conjRef, q);

    std::vector<double> realH = {h[1], h[2], h[3]};
    std::vector<double> b = {0, magnitude(realH), 0, h[3]};

    std::vector<double> gradient(4, 0);
    gradient[0] = 2 * (q[1] * q[3] - q[0] * q[2]) - accel[0];
    gradient[1] = 2 * (q[0] * q[1] + q[2] * q[3]) - accel[1];
    gradient[2] = 2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel[2];
    gradient[3] = 2 * b[1] * (0.5 - q[2] * q[2] - q[3] * q[3]) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]) - mag[0];

    return gradient;
}

std::vector<double> Madgwick::quaternionToEuler(std::vector<double>& quaternion) {

    if (quaternion.size() != 4) {
        std::cerr << "[Madgwick::quaternionToEuler] ERROR: Quaternion must have 4 elements. Got " 
                  << quaternion.size() << "\n";
        std::exit(EXIT_FAILURE);
    }

    return {
        std::atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 
                   1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])),
        std::asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1])),
        std::atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 
                   1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]))
    };
}


std::vector<double> Madgwick::eulerToQuaternion(std::vector<double>& euler) {
    double roll = euler[0]; 
    double pitch = euler[1]; 
    double yaw = euler[2];  

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    return {
        cr * cp * cy + sr * sp * sy, 
        sr * cp * cy - cr * sp * sy, 
        cr * sp * cy + sr * cp * sy, 
        cr * cp * sy - sr * sp * cy  
    };
}

std::vector<double> Madgwick::quaternionMultiply(std::vector<double>& q1, std::vector<double>& q2) {
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    };
}
