#define _USE_MATH_DEFINES
#include <vector>
#include <cmath>
#include <math.h>

class Madgwick {
    protected:
    double gain;
    double beta;
    float lastUpdateTime;
    std::vector<double> eulerAttitudeEstimation;
    std::vector<double> orientation;

    public:
        // Constructor for Madgwick filter, given gain and beta parameters.
        // Both gain and beta should be within the range [0,1]
        Madgwick(double setGain, double setBeta, double startTime, std::vector<double>& initialOrientation) {
            gain = setGain;
            beta = setBeta;
            lastUpdateTime = startTime;
            orientation = initialOrientation;
            eulerAttitudeEstimation = quaternionToEuler(orientation);
        }

        std::vector<double> update(std::vector<double>& updateArr) {
            // Read time data, calculate delta time, and update previous update time
            double dt = updateArr[0] - lastUpdateTime;
            delete &lastUpdateTime;
            lastUpdateTime = updateArr[0];

            // Read accelerometer, gyroscope, and magnetometer data
            std::vector<double> accel = {updateArr[1], updateArr[2], updateArr[3]};
            std::vector<double> gyro = {updateArr[4], updateArr[5], updateArr[6]};
            std::vector<double> mag = {updateArr[7], updateArr[8], updateArr[9]};

            // Apply Madgwick Filter
            std::vector updatedOrientation = madgwickUpdate(orientation, gyro, accel, mag, dt);
            delete &orientation;
            orientation = updatedOrientation;
            delete &updatedOrientation;
            delete &gyro;
            delete &accel;
            delete &mag;
            delete &dt;

            // Calculates Attitude in euler angles using quaternion conversion formulas
            // Source: https://madecalculators.com/quaternion-to-euler-calculator/
            delete &eulerAttitudeEstimation;
            eulerAttitudeEstimation = quaternionToEuler(orientation);

            // Returns the attitude estimation
            return eulerAttitudeEstimation;
        }

        std::vector<double> madgwickUpdate(std::vector<double>& q, std::vector<double>& gyro, std::vector<double>& accel, std::vector<double>& mag, double dt) {
            // Normalize accelerometer and magnetometer vectors
            normalizeArray(accel);
            normalizeArray(mag);

            // Gradient descent step to minimize the error
            std::vector<double> step = computeGradient(q, accel, mag);
            normalizeArray(step);

            // Integrate Gyroscope
            // Gyroscope compensation drift
            std::vector<double> qConj = {q[0], -q[1], -q[2], -q[3]};
            std::vector<double> integration = quaternionMultiply(qConj, step);
            for (int i = 0; i < integration.size(); i++) {
                integration[i] *= 2 * dt * gain * -1;
            }
            std::vector<double> gyroQ = {integration[0], gyro[0] + integration[1], gyro[1] + integration[2], gyro[2] + integration[3]};

            // Compute rate of change of quaternion
            std::vector<double> qGyroQ = quaternionMultiply(q, gyroQ);
            for (int i = 0; i < qGyroQ.size(); i++) {
                qGyroQ[i] *= 0.5;
            }
            for (int i = 0; i < step.size(); i++) {
                step[i] *= -beta;
            }
            std::vector<double> qdot = {qGyroQ[0] + step[0], qGyroQ[1] + step[1], qGyroQ[2] + step[2], qGyroQ[3] + step[3]};

            // Integrate and yield quaternion
            std::vector<double> updatedQ = {q[0] + qdot[0] * dt, q[1] + qdot[1] * dt, q[2] + qdot[2] * dt, q[3] + qdot[3] * dt};
            normalizeArray(updatedQ);

            delete &step;
            delete &qConj;
            delete &integration;
            delete &gyroQ;
            delete &qGyroQ;
            delete &qdot;

            return updatedQ;
        }

        std::vector<double> computeGradient(std::vector<double>& q, std::vector<double>& accel, std::vector<double>& mag) {
            // References direction of Earth's magnetic field
            std::vector<double> conj = {q[0], -q[1], -q[2], -q[3]};
            std::vector<double> qRef = {0, mag[0], mag[1], mag[2]};
            std::vector<double> conjRef = quaternionMultiply(conj, qRef);
            std::vector<double> h = quaternionMultiply(conjRef, q);
            delete &conj;
            delete &qRef;
            delete &conjRef;

            std::vector<double> realH = {h[1], h[2], h[3]};
            std::vector<double> b = {0, magnitude(realH), 0, h[3]};
            delete &realH;

            // Calculates objective function and the jacobian of the objective function respectively
            // Formulas sourced from: https://medium.com/@k66115704/imu-madgwick-filter-explanation-556fbe7f02e3
            std::vector<double> f = {2 * (q[1] * q[3] - q[0] * q[2]) - accel[0],
                                    2 * (q[0] * q[1] + q[2] * q[3]) - accel[1],
                                    2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel[2],
                                    2 * b[1] * (0.5 - q[2] * q[2] - q[3] * q[3]) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]) - mag[0],
                                    2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]) - mag[1],
                                    2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1] * q[1] - q[2] * q[2]) - mag[2]};
            std::vector<std::vector<double>> J = {{-2 * q[2], 2 * q[1], 0, -2 * b[3] * q[2], -2 * b[1] * q[3] + 2 * b[3] * q[1], 2 * b[1] * q[2]},
                                                {2 * q[3], 2 * q[0], -4 * q[1], 2 * b[3]* q[3], 2 * b[1] * q[2] + 2 * b[3] * q[0], 2 * b[1] * q[3] - 4 * b[3] * q[1]},
                                                {-2 * q[0], 2 * q[3], -4 * q[2], -4 * b[1] * q[2] - 2 * b[3] * q[0], 2 * b[1] * q[1] + 2 * b[3] * q[3], 2 * b[1] * q[0] - 4 * b[3] * q[2]},
                                                {2 * q[1], 2 * q[2], 0, -4 * b[1] * q[3] + 2 * b[3] * q[1], -2 * b[1] * q[0] + 2 * b[3] * q[2], 2 * b[1] * q[1]}};

            std::vector<double> gradient(6, 0);
            for (int i = 0; i < J.size(); i++) {
                for (int j = 0; j < J[0].size(); j++) {
                    gradient[i] += J[i][j] * f[j];
                }
            }

            delete &f;
            delete &J;

            return gradient;
        }

        std::vector<double> quaternionToEuler(std::vector<double>& quaternion) {
            return {std::atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])),
                    2 * std::atan2(std::sqrt(1 + 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3])), std::sqrt(1 - 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]))) - M_PI / 2,
                    std::atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]))};
        }

        double magnitude(std::vector<double>& array){
            double total = 0;
            for (double value : array){
                total += value * value;
            }
            return std::sqrt(total);
        }

        void normalizeArray(std::vector<double>& array) {
            if (array.size() == 0) {
                return;
            }
            double arrayMagnitude = magnitude(array);
            for (int i = 0; i < array.size(); i++) {
                array[i] /= arrayMagnitude;
            }
            delete &arrayMagnitude;
        }

        std::vector<double> quaternionMultiply(std::vector<double>& q1, std::vector<double>& q2) {
            return {q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
                    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
                    q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
                    q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]};
        }
};