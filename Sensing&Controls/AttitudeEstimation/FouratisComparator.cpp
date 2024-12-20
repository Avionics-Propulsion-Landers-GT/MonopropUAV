#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

int main() {
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
    std::ofstream myfile;
    myfile.open ("fourati_error.csv");
    for (const auto& q : fouratiError) {
        myfile << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << endl;
    }
    myfile.close();

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
}