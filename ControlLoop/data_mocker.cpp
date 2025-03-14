#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip> 
#include <chrono>
#include <numeric>
#include "loop.h"
#include "init.h"

std::vector<double> readGPSInit(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<double> gpsInit;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open GPS CSV file: " << filename << std::endl;
        return gpsInit;
    }

    std::string line;
    
    // Skip the header line
    if (!std::getline(file, line)) {
        std::cerr << "Error: GPS CSV file is empty or header missing!" << std::endl;
        return gpsInit;
    }

    // Read the first data line
    if (std::getline(file, line)) {  // Read second line (first data row)
        std::stringstream ss(line);
        std::string value;
        
        while (std::getline(ss, value, ',')) { // Parse comma-separated values
            try {
                gpsInit.push_back(std::stod(value));  // Convert to double
            } catch (const std::exception& e) {
                std::cerr << "Error parsing GPS CSV: " << e.what() << std::endl;
                return {};
            }
        }
    } else {
        std::cerr << "Error: No data found in GPS CSV!" << std::endl;
    }

    file.close();
    return gpsInit;  // Returns {longitude, latitude, altitude}
}

bool isNumber(const std::string& str) {
    bool hasDecimal = false;

    for (size_t i = 0; i < str.length(); ++i) {
        if (std::isdigit(str[i]) || (str[i] == '-' && i == 0)) {
            continue;  // Allow digits and a negative sign at the start
        }
        if (str[i] == '.' && !hasDecimal) {
            hasDecimal = true;  // Allow one decimal point
            continue;
        }
        return false;  // If we hit a non-numeric character, return false
    }
    return !str.empty();  // Empty strings are not numbers
}

bool isHeader(const std::string& line) {
    std::stringstream ss(line);
    std::string cell;
    
    while (std::getline(ss, cell, ',')) {
        if (!isNumber(cell)) {
            return true;  // If any cell is not numeric, it's a header
        }
    }
    return false;  // If all cells are numbers, it's a data row
}

int main() {
    std::vector<std::string> files = {
        "../Sensing&Controls/AttitudeEstimation/monocopter_data.csv",
        "../Sensing&Controls/AttitudeEstimation/noisy_monocopter_data.csv",
        "../Sensing&Controls/AttitudeEstimation/gps_data.csv",
        "../Sensing&Controls/AttitudeEstimation/noisy_lidar_data.csv",
        "../Sensing&Controls/AttitudeEstimation/uwb_data.csv"
    };

    std::vector<std::ifstream> fileStreams;
    
    // Open files
    for (const std::string& file : files) {
        std::ifstream fs(file);
        if (!fs.is_open()) {
            std::cerr << "Error: Could not open " << file << std::endl;
            return 1;  // Exit the program if a file cannot be opened
        }
        fileStreams.emplace_back(std::move(fs)); 
    }

    std::string line;
    std::vector<std::vector<double>> values;

    

    // Skip header text
    for (auto& fs : fileStreams) {
        if (std::getline(fs, line)) {
            if (isHeader(line)) {
                std::cout << "Skipping header: " << line << std::endl;
                std::getline(fs, line); 
            }
        }
    }

    std::ofstream latencyFile("latency.csv");
    latencyFile << "Iteration,Latency (ms)\n";

    std::ofstream dataFile("state.csv");
    dataFile << "Iteration,x,y,z\n";

    bool allFilesHaveData = true;
    int iteration = 0;
    double latency_ms;
    std::vector<double> latencies; 
    // status consists of: < gpsSanityCheck, lidarStatus, >
    std::vector<bool> status = {false, true};

    auto startProcessing = std::chrono::high_resolution_clock::now();


    LoopOutput out; // Initialize state vector and command output
    out.state = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
    out.command = {0,0,0};

    std::vector<double> gps_init = readGPSInit(files[2]);
    std::cout << "gps init: " << gps_init[0] << ", " << gps_init[1] << ", " << gps_init[2] << std::endl; 

    SystemComponents system = init(gps_init, out.state); // Initialize all filters

    while (allFilesHaveData) {
        values.clear();
        allFilesHaveData = false;

        size_t maxColumns = 0; 

        for (size_t i = 0; i < fileStreams.size(); ++i) {
            std::vector<double> rowValues;

            if (fileStreams[i].good() && std::getline(fileStreams[i], line)) {  // ✅ Only read valid lines
                std::stringstream ss(line);
                std::string cell;

                while (std::getline(ss, cell, ',')) {
                    try {
                        rowValues.push_back(std::stod(cell));
                    } catch (const std::invalid_argument&) {
                        rowValues.push_back(0.0);  // Default to zero if conversion fails
                    }
                }

                allFilesHaveData = true;  
                if (rowValues.size() > maxColumns) maxColumns = rowValues.size();
            }

            if (!rowValues.empty()) {
                values.push_back(rowValues);
            }
        }

        if (!allFilesHaveData) break;

        for (auto& row : values) {
            while (row.size() < maxColumns) {
                row.push_back(0.0);  // Fill missing values with 0.0
            }
        }

        if (values.size() > 1 && !values[1].empty()) {

            auto start = std::chrono::high_resolution_clock::now();
            double dt = 0.001; // s
            out = loop(values, out.state, system, status, dt);
            auto end = std::chrono::high_resolution_clock::now();

            latency_ms = std::chrono::duration<double, std::milli>(end - start).count();
            latencies.push_back(latency_ms);

            double frequency_hz = latency_ms > 0 ? 1000.0 / latency_ms : 0.0;
           
            // Log to a latency file to make sure there are no spikes in compute time. A safe value is >5000 Hz. Anything below that is dangerous!
            latencyFile << iteration << "," << std::fixed << std::setprecision(6) << latency_ms << "," << frequency_hz << "\n";

            // Log the state to a data file for review & filter testing vs. original data
            dataFile << iteration << "," << std::fixed << std::setprecision(6) << out.state[1][0] << "," << out.state[1][1] << "," << out.state[1][2] << "\n";
            iteration++;

        } else {
            std::cerr << "EOF" << std::endl;
        }
    }

    auto endProcessing = std::chrono::high_resolution_clock::now();


    // Print a convenient performance summary
    if (!latencies.empty()) {
        double avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
        double avg_frequency = avg_latency > 0 ? 1000.0 / avg_latency : 0.0;
        double processingTime = std::chrono::duration<double, std::milli>(endProcessing - startProcessing).count();

        latencyFile << "\nAverage Latency (ms)," << std::fixed << std::setprecision(6) << avg_latency << "\n";
        latencyFile << "Average Frequency (Hz)," << avg_frequency << "\n";

        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << "Average Latency: " << avg_latency << " ms" << std::endl;
        std::cout << "Average Frequency: " << avg_frequency << " Hz" << std::endl;
        std::cout << "Total Processing Time: " << processingTime << " ms" << std::endl;
    }

    latencyFile.close(); 

    // Close files safely
    for (auto& fs : fileStreams) {
        if (fs.is_open()) {
            fs.close();
        }
    }

    return 0;
}