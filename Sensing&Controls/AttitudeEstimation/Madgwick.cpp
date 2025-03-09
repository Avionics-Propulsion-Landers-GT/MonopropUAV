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
            std::vector accel = {updateArr[1], updateArr[2], updateArr[3]};
            std::vector gyro = {updateArr[4], updateArr[5], updateArr[6]};
            std::vector mag = {updateArr[7], updateArr[8], updateArr[9]};

            // Apply Madgwick Filter
            std::vector updatedOrientation = madgwickUpdate(orientation, gyro, accel, mag, dt);
            delete &orientation;
            orientation = updatedOrientation;
            delete &updatedOrientation;
            delete &gyro;
            delete &accel;
            delete &mag;
            delete &dt;

            // TODO: turn orientation into euler angles and return it

            return eulerAttitudeEstimation;
        }

        std::vector<double> madgwickUpdate(std::vector<double>& q, std::vector<double>& gyro, std::vector<double>& accel, std::vector<double>& mag, double dt) {
            // Normalize accelerometer and magnetometer vectors
            normalizeArray(accel);
            normalizeArray(mag);

            // Gradient descent step to minimize the error
            std::vector<double> step = computeGradient(q, accel, mag);
            normalizeArray(step);

            // TODO: integrate the gyroscope, compute rate of change of quaternion, and integrate that
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
            
            // TODO: implement gradient algorithm matrix multiplication method
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
}