#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

// Structure to hold sensor data
struct SensorData {
    double timestamp;   
    double gyro[3];
    double accel[3];
    double mag[3];  
};

// Function to read CSV file and parse sensor data in gyro, accel, mag order
vector<SensorData> readCSV(const string& filename) {
    vector<SensorData> sensorData;
    ifstream file(filename);
    string line;
    if (file.is_open()) {
        getline(file, line); // Skip header row
        while (getline(file, line)) {
            stringstream ss(line);
            SensorData data;
            ss >> data.timestamp;
            ss.ignore();
            ss >> data.gyro[0];
            ss.ignore();
            ss >> data.gyro[1];
            ss.ignore();
            ss >> data.gyro[2];
            ss.ignore();
            ss >> data.accel[0];
            ss.ignore();
            ss >> data.accel[1];
            ss.ignore();
            ss >> data.accel[2];
            ss.ignore();
            ss >> data.mag[0];
            ss.ignore();
            ss >> data.mag[1];
            ss.ignore();
            ss >> data.mag[2];
            sensorData.push_back(data);
        }
        file.close();
    }
    return sensorData;
}

// Quaternion multiplication
vector<double> quaternionMultiply(const vector<double>& q1, const vector<double>& q2) {
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    };
}

// Quaternion normalization
vector<double> normalizeQuaternion(const vector<double>& q) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0) {
        return { q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm };
    }
    return { 1.0, 0.0, 0.0, 0.0 }; // Return identity quaternion if dividing by 0
}

// Filters out high-frequency noise 
vector<double> lowPassFilter(const vector<double>& data, const vector<double>& prevData, double alpha) {
    vector<double> result(3);
    for (int i = 0; i < 3; i++) {
        result[i] = alpha * data[i] + (1 - alpha) * prevData[i];
    }
    return result;
}

// Filters out low-frequency noise
vector<double> highPassFilter(const vector<double>& data, const vector<double>& prevData, double alpha) {
    vector<double> lowPass = lowPassFilter(data, prevData, alpha);
    vector<double> result(3);
    for (int i = 0; i < 3; i++) {
        result[i] = data[i] - lowPass[i];
    }
    return result;
}

// Rotate vector with a quaternion
vector<double> rotateVectorWithQuaternion(const vector<double>& vec, const vector<double>& q) {
    vector<double> qConjugate = { q[0], -q[1], -q[2], -q[3] };
    vector<double> vecQuat = { 0, vec[0], vec[1], vec[2] };
    vector<double> rotatedVec = quaternionMultiply(quaternionMultiply(q, vecQuat), qConjugate);
    return { rotatedVec[1], rotatedVec[2], rotatedVec[3] }; 
}

// Integrates gyroscope data and updates attitude quaternion
vector<double> integrateGyroscope(vector<double> q, const vector<double>& gyro, double dt) {
    // Quaternion for the gyroscope data 
    vector<double> gyroQuat = { 0, gyro[0], gyro[1], gyro[2] };

    // Scale gyroscope data by 0.5 for time elapsed (rate of change of a quaternion is half the angular displacement over a small time interval)
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

// Correct orientation using magnetometer and accelerometer data
vector<double> correctOrientation(const vector<double>& q, const vector<double>& accel, const vector<double>& mag) {
    // Expected gravity and magnetic reference vectors
    vector<double> gravityRef = { 0, 0, 9.81 }; 
    vector<double> magRef = { 5.00E-05, 0.0, 0.0 }; //can be tuned later

    // Rotate reference vectors to body frame
    vector<double> gravityBody = rotateVectorWithQuaternion(gravityRef, q);
    vector<double> accelError = { gravityBody[0] - accel[0], gravityBody[1] - accel[1], gravityBody[2] - accel[2] };

    // Rotate magnetometer reading to body frame
    vector<double> magBody = rotateVectorWithQuaternion(magRef, q);
    vector<double> magError = { magBody[0] - mag[0], magBody[1] - mag[1], magBody[2] - mag[2] };

    // Combine errors with weights for correction
    double alphaAccel = 0.5; // Weight for accelerometer correction (TUNE)
    double alphaMag = 0.5;   // Weight for magnetometer correction (TUNE)

    vector<double> correctionQuat = {
        1.0,
        alphaAccel * accelError[0] + alphaMag * magError[0],
        alphaAccel * accelError[1] + alphaMag * magError[1],
        alphaAccel * accelError[2] + alphaMag * magError[2]
    };

    return normalizeQuaternion(quaternionMultiply(q, correctionQuat));
}

// Main Fourati Filter
vector<vector<double>> fouratiFilter(const vector<SensorData>& data, double alpha = 0.1) {
    int n = data.size();
    vector<double> q = { 1, 0, 0, 0 }; // Initial quaternion

    vector<double> prevAccel = { data[0].accel[0], data[0].accel[1], data[0].accel[2] };
    vector<double> prevMag = { data[0].mag[0], data[0].mag[1], data[0].mag[2] };
    vector<double> prevGyro = { data[0].gyro[0], data[0].gyro[1], data[0].gyro[2] };

    vector<vector<double>> quaternions; // Stores the quaternions

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

        // Correct orientation
        q = correctOrientation(q, accelFiltered, magFiltered);

        // Store the quaternion
        quaternions.push_back(normalizeQuaternion(q));

        // Update previous sensor data for next loop
        prevAccel = accelFiltered;
        prevMag = magFiltered;
        prevGyro = gyroFiltered;

    }

    return quaternions;
}

int main() {
    vector<SensorData> sensorData = readCSV("noisy_monocopter_data.csv");
    vector<vector<double>> quaternions = fouratiFilter(sensorData);
    std::ofstream myfile;
    myfile.open ("fourati_output.csv");
    for (const auto& q : quaternions) {
        cout << "Quaternion: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << endl;
        myfile << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << endl;
    }
    myfile.close();
    return 0;
}

