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

// Quaternion inverse
vector<double> inverseQuaternion(const vector<double>& q) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0) {
        return { q[0] / norm, -q[1] / norm, -q[2] / norm, -q[3] / norm };
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
    vector<double> qConjugate = inverseQuaternion(q);
    vector<double> vecQuat = { 0, vec[0], vec[1], vec[2] };
    vector<double> rotatedVec = quaternionMultiply(quaternionMultiply(q, vecQuat), qConjugate);
    return { rotatedVec[1], rotatedVec[2], rotatedVec[3] };
}

// Gets determinant of a 3x3 matrix
double determinant3x3Matrix(const vector<vector<double>>& matrix) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) - matrix[0][1] * (matrix[1][0] * matrix[2][1] - matrix[1][2] * matrix[2][0]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

// Returns the cofactor of a matrix
vector<vector<double>> cofactorMatrix(const vector<vector<double>>& matrix) {
    vector<vector<double>> cofactor(matrix.size(), vector<double> (matrix.size()));

    for(int i = 0; i < matrix.size(); i++) {
        for(int j = 0; j < matrix.size(); j++) {
            cofactor[i][j] = matrix[i][j]; 
        }
    }

    for (int i = 0; i < cofactor.size(); i++) {
        for (int j = 0; j < cofactor[0].size(); j++) {
            cofactor[i][j] *= (i + j) % 2;
        }
    }

    return cofactor;
}

// Returns the transpose of a matrix
vector<vector<double>> transposeMatrix(const vector<vector<double>>& matrix) {
    vector<vector<double>> transpose(matrix[0].size(), vector<double> (matrix.size()));

    for(int i = 0; i < transpose.size(); i++) {
        for(int j = 0; j < transpose[0].size(); j++) {
            transpose[i][j] = matrix[j][i]; 
        }
    }

    return transpose;
}

// Inverts a 3x3 matrix
vector<vector<double>> inverse3x3Matrix(const vector<vector<double>>& matrix) {
    double d = 1.0 / determinant3x3Matrix(matrix);
    vector<vector<double>> solution(matrix.size(), vector<double> (matrix.size()));

    for(int i = 0; i < matrix.size(); i++) {
        for(int j = 0; j < matrix.size(); j++) {
            solution[i][j] = matrix[i][j]; 
        }
    }

    solution = transposeMatrix(cofactorMatrix(solution));

    for(int i = 0; i < solution.size(); i++) {
        for(int j = 0; j < solution[0].size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}

// Multiplies a matrix by a scalar
vector<vector<double>> scalarMultiplyMatrix(const vector<vector<double>>& matrix, double scalar) {
    vector<vector<double>> solution(matrix.size(), vector<double> (matrix[0].size()));

    for(int i = 0; i < matrix.size(); i++) {
        for(int j = 0; j < matrix[0].size(); j++) {
            solution[i][j] = matrix[i][j] * scalar; 
        }
    }

    return solution;
}

// Multiplies two matrices together
vector<vector<double>> matrixMultiply(const vector<vector<double>>& matrixOne, const vector<vector<double>>& matrixTwo) {
    vector<vector<double>> solution(matrixOne.size(), vector<double> (matrixTwo[0].size()));

    for(int i = 0; i < solution.size(); i++) {
        for(int j = 0; j < solution[0].size(); j++) {
            solution[i][j] = 0;
        }
    }

    for(int i = 0; i < solution.size(); i++) {
        for(int j = 0; j < solution[0].size(); j++) {
            for (int k = 0; k < matrixOne[0].size(); k++) {
                solution[i][j] += matrixOne[i][k] * matrixTwo[k][j];
            }
        }
    }

    return solution;
}

// Adds two matrices together
vector<vector<double>> matrixAddition(const vector<vector<double>>& matrixOne, const vector<vector<double>>& matrixTwo) {
    vector<vector<double>> solution(matrixOne.size(), vector<double> (matrixOne[0].size()));

    for(int i = 0; i < solution.size(); i++) {
        for(int j = 0; j < solution[0].size(); j++) {
            solution[i][j] = matrixOne[i][j] + matrixTwo[i][j];
        }
    }

    return solution;
}

// Copies a matrix
vector<vector<double>> matrixCopy(const vector<vector<double>>& matrix) {
    vector<vector<double>> copy(matrix.size(), vector<double> (matrix[0].size()));

    for(int i = 0; i < copy.size(); i++) {
        for(int j = 0; j < copy[0].size(); j++) {
            copy[i][j] = matrix[i][j];
        }
    }

    return copy;
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

    vector<vector<double>> jacobian = scalarMultiplyMatrix(transposeMatrix({ {0, -gravityBody[2], gravityBody[1], 0, -magBody[2], magBody[1]},
                                {gravityBody[2], 0, -gravityBody[0], magBody[2], 0, -magBody[0]},
                                {-gravityBody[1], gravityBody[0], 0 -magBody[1], magBody[0], 0}, }), -2);

    double k = 0.2;
    vector<vector<double>> gain = matrixMultiply(scalarMultiplyMatrix(inverse3x3Matrix(matrixAddition(matrixMultiply(transposeMatrix(jacobian), jacobian), scalarMultiplyMatrix({{1,0,0},{0,1,0},{0,0,1}}, 0.0000000000001))), k), transposeMatrix(jacobian));
    vector<vector<double>> correctionTerm = { {1, 0, 0, 0, 0, 0, 0},
                                            gain[0],
                                            gain[1],
                                            gain[2] };
    for (int i = 1; i < 3; i++) {
        correctionTerm[i].insert(correctionTerm[i].begin(), 0);
    }
    correctionTerm = matrixMultiply(correctionTerm, {{1}, {accelError[0]}, {accelError[1]}, {accelError[2]}, {magError[0]}, {magError[1]}, {magError[2]}});

    return normalizeQuaternion(quaternionMultiply(q, { correctionTerm[0][0], correctionTerm[1][0], correctionTerm[2][0], correctionTerm[3][0] }));
}

// Integrates gyroscope data and updates attitude quaternion
vector<double> integrateGyroscope(vector<double> q, const vector<double>& gyro, double dt, const vector<double>& accel, const vector<double>& mag) {
    // Quaternion for the gyroscope data 
    vector<double> gyroQuat = { 0, gyro[0], gyro[1], gyro[2] };

    // Scale gyroscope data by 0.5 for time elapsed (rate of change of a quaternion is half the angular displacement over a small time interval)
    for (int i = 0; i < 3; i++) {
        gyroQuat[i + 1] *= 0.5 * dt;
    }

    // Compute the quaternion derivative
    vector<double> qDot = quaternionMultiply(q, gyroQuat);

    qDot = correctOrientation(qDot, accel, mag);

    // Update quaternion
    for (int i = 0; i < 4; i++) {
        q[i] += qDot[i] * dt;
    }

    return normalizeQuaternion(q);
}

// Main Fourati Filter
vector<vector<double>> fouratiFilter(const vector<SensorData>& data) {
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
        vector<double> accelFiltered = lowPassFilter(accel, prevAccel, 0.5);
        vector<double> magFiltered = lowPassFilter(mag, prevMag, 0.5);
        vector<double> gyroFiltered = highPassFilter(gyro, prevGyro, 0.5);

        // Integrate gyroscope data
        q = integrateGyroscope(q, gyroFiltered, dt, accelFiltered, magFiltered);

        // Correct orientation
        //q = correctOrientation(q, accelFiltered, magFiltered);

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
        //cout << "Quaternion: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << endl;
        myfile << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << endl;
    }
    myfile.close();



    std::ifstream file("quaternion_data.csv");
    vector<vector<double>> quaternionData;
    string line;
    if (file.is_open()) {
        while (getline(file, line)) {
            stringstream ss(line);
            string value;
            vector<string> row;
            vector<double> quaternion;
            
            while (getline(ss, value, ',')) {
                row.push_back(value);
            }
            for (const auto& val : row) {
                quaternion.push_back(stod(val));
            }
            quaternionData.push_back(quaternion);
        }
        file.close();
    }
    
    ifstream fileTwo("fourati_output.csv");
    vector<vector<double>> fouratiOutput;
    if (fileTwo.is_open()) {
        while (getline(fileTwo, line)) {
            stringstream ss(line);
            string value;
            vector<string> row;
            vector<double> quaternion;
            
            while (getline(ss, value, ',')) {
                row.push_back(value);
            }
            for (const auto& val : row) {
                quaternion.push_back(stod(val));
            }
            fouratiOutput.push_back(quaternion);
        }
        fileTwo.close();
    }

    vector<vector<double>> fouratiError;
    for (int i = 0; i < quaternionData.size(); i++){
        fouratiError.push_back({quaternionData[i][0] - fouratiOutput[i][0], quaternionData[i][1] - fouratiOutput[i][1], quaternionData[i][2] - fouratiOutput[i][2], quaternionData[i][4] - fouratiOutput[i][4]});
    }
    std::ofstream mefile;
    mefile.open ("fourati_error.csv");
    for (const auto& q : fouratiError) {
        mefile << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << endl;
    }
    mefile.close();

    ifstream fileThree("fourati_error.csv");
    vector<double> errorMags;
    if (fileThree.is_open()) {
        while (getline(fileThree, line)) {
            stringstream ss(line);
            string value;
            vector<string> row;
            vector<double> quaternion;
            double errorMag;
            
            while (getline(ss, value, ',')) {
                row.push_back(value);
            }
            for (const auto& val : row) {
                quaternion.push_back(stod(val));
            }

            for (const auto& val : quaternion) {
                errorMag += val * val;
            }
            errorMag = sqrt(errorMag);
            errorMags.push_back(errorMag);
        }
        fileThree.close();
    }
    
    std::ofstream errorMagFile;
    errorMagFile.open ("fourati_error_magnitude.csv");
    for (const auto& val : errorMags) {
        errorMagFile << val << endl;
    }
    errorMagFile.close();
    return 0;
    return 0;
}

