#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Structure to hold sensor data 
struct SensorData {
    double timestamp;
    double accel[3];
    double gyro[3];
    double mag[3];
};

// Quaternion multiplication
vector<double> quaternionMultiply(vector<double> q1, vector<double> q2) {
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    };
}

// Quaternion normalization
vector<double> normalizeQuaternion(vector<double> q) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
}

// Filters out high frequency noise 
vector<double> lowPassFilter(vector<double> data, vector<double> prevData, double alpha = 0.1) {
    vector<double> result(3);
    for (int i = 0; i < 3; i++) {
        result[i] = alpha * data[i] + (1 - alpha) * prevData[i];
    }
    return result;
}

// Filters out low frequency noise
vector<double> highPassFilter(vector<double> data, vector<double> prevData, double alpha = 0.1) {
    vector<double> lowPass = lowPassFilter(data, prevData, alpha);
    vector<double> result(3);
    for (int i = 0; i < 3; i++) {
        result[i] = data[i] - lowPass[i];
    }
    return result;
}

// Rotating a vector with a quaternion
vector<double> rotateVectorWithQuaternion(vector<double> vec, vector<double> q) {
    vector<double> qVec = { 0, vec[0], vec[1], vec[2] };
    vector<double> qConjugate = { q[0], -q[1], -q[2], -q[3] };
    vector<double> rotatedVec = quaternionMultiply(quaternionMultiply(q, qVec), qConjugate);
    return { rotatedVec[1], rotatedVec[2], rotatedVec[3] };
}

// Compute error between expected and measured sensor values
vector<double> computeError(vector<double> expected, vector<double> measured) {
    vector<double> error(3);
    for (int i = 0; i < 3; i++) {
        error[i] = expected[i] - measured[i];
    }
    return error;
}

// Integrates gyroscope data and updates attitude quaternion
vector<double> integrateGyroscope(vector<double> q, vector<double> gyro, double dt) {
    // Quaternion for the gyroscope data 
    vector<double> gyroQuat = { 0, gyro[0], gyro[1], gyro[2] };
    
    // Scales gyroscope data by 0.5 for time elapsed sine (rate of change of a quaternion is half the angular displacement over a small time interval)
    for (int i = 0; i < 3; i++) {
        gyroQuat[i + 1] *= 0.5 * dt; 
    }

    // Compute the quaternion derivative
    vector<double> qDot = quaternionMultiply(q, gyroQuat);

    // Update quaternion
    for (int i = 0; i < 4; i++) {
        q[i] += qDot[i];
    }

    return normalizeQuaternion(q);
}


// Correct orientation using DBA and magnetometer data
vector<double> correctOrientation(vector<double> q, vector<double> dba, vector<double> mag) {
    // Expected gravity
    vector<double> gravity = { 0, 0, 9.81 };

    vector<double> referenceMag = { 1.0, 0.0, 0.0 }; //can be tuned later

    // Rotate gravity vector to body frame 
    vector<double> gravityBody = rotateVectorWithQuaternion(gravity, q);

    // Compute error between measured and expected gravity
    vector<double> accelError = computeError(gravityBody, dba);

    // compute error for the magnetic field
    vector<double> magError = computeError(mag, referenceMag);

    // Combine the error and apply correction 
    vector<double> qCorrection = { 1, accelError[0] + magError[0], accelError[1] + magError[1], accelError[2] + magError[2] };
    normalizeQuaternion(qCorrection);

    return quaternionMultiply(q, qCorrection);
}

// Compute DBA by subtracting gravity (For use to filter out sudden motions)
vector<double> computeDynamicBodyAcceleration(vector<double> accel, vector<double> q) {
    vector<double> gravity = { 0, 0, 9.81 };

    // Rotate gravity vector to body frame
    vector<double> gravityBody = rotateVectorWithQuaternion(gravity, q);

    // Subtract gravity from measured acceleration to get DBA
    vector<double> dba(3);
    for (int i = 0; i < 3; i++) {
        dba[i] = accel[i] - gravityBody[i];
    }

    return dba;
}

// Main Fourati Filter
vector<vector<double>> fouratiFilter(vector<SensorData> data, double alpha = 0.1) {
    int n = data.size();
    vector<double> q = { 1, 0, 0, 0 }; // Initial quaternion

    vector<double> prevAccel = { 0, 0, 0 };
    vector<double> prevMag = { 0, 0, 0 };
    vector<double> prevGyro = { 0, 0, 0 };

    vector<vector<double>> quaternions;  // Stores the quaternions

    for (int i = 0; i < n; i++) {
        vector<double> accel = { data[i].accel[0], data[i].accel[1], data[i].accel[2] };
        vector<double> gyro = { data[i].gyro[0], data[i].gyro[1], data[i].gyro[2] };
        vector<double> mag = { data[i].mag[0], data[i].mag[1], data[i].mag[2] };

        // Time calculation
        double dt = (i == 0) ? 0 : (data[i].timestamp - data[i - 1].timestamp);

        // Apply filters
        vector<double> accelFiltered = lowPassFilter(accel, prevAccel, alpha);
        vector<double> magFiltered = lowPassFilter(mag, prevMag, alpha);
        vector<double> gyroFiltered = highPassFilter(gyro, prevGyro, alpha);

        // Integrate gyroscope data
        q = integrateGyroscope(q, gyroFiltered, dt);

        // Compute DBA and correct orientation
        vector<double> dba = computeDynamicBodyAcceleration(accelFiltered, q);
        q = correctOrientation(q, dba, magFiltered); 

        // Store the quaternion
        quaternions.push_back(q);

        // Update previous sensor data for next loop
        prevAccel = accelFiltered;
        prevMag = magFiltered;
        prevGyro = gyroFiltered;
    }

    return quaternions;
}


int main() {
 
}