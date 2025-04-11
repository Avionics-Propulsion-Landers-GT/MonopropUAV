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
#include "LQR/calculateA.h"
#include "LQR/calculateB.h"
#include "Matrix.h"
#include "Vector.h"
#include <thread>
#include <algorithm> 
#include <cmath>

/*

data_mocker.cpp 

by spencer boebel

PURPOSE: Read in data from multiple CSVs and output
relevant state and command data to a CSV file ["state.csv"].

USAGE: This program has associated libraries. Compile this program
and its associated libraries (in Linux/Unix based systems) with:

g++ -I /usr/include/eigen3 -o data_mocker data_mocker.cpp \
loop.cpp init.cpp "../Sensing&Controls/AttitudeEstimation/updatedMadgwick.cpp" \
ExtendedKalmanFilterGeneral.cpp EKF_xy.cpp EKF_z.cpp

FOR ELECTRONICS: Replace methods where appropriate in order to integrate with the hardware.
However, before you start making changes I would highly suggest loading the ControlLoop
directory and the associated CSV files (make sure you change the paths!) to the Teensy 
and checking the execution time. On my computer (Windows X86 w/ Intel Core i5) this program
runs at about 8000 Hz minimum. The program will output a performance summary at the end, which
will tell you how fast it is running. If it is running slower than 2000 Hz, then there is
a serious danger of program lag and the timestep will have to be increased to 0.01s or something.

*/

const double SET_DT = 0.001;

std::vector<double> trajectory(double t) { // Replace with waypoint code
    const double pi = 3.141592653589793;

    if (t >= 0.0 && t < 5.0) {
        return {0,0,-0.5 * cos(pi * t / 5.0) + 0.5, 0,0,(pi / 10.0) * sin(pi * t / 5.0), 0,0,0, 0,0,0};
    } else if (t >= 5.0 && t < 10.0) {
        return {0,0,1, 0,0,0, 0,0,0, 0,0,0};
    } else if (t >= 10.0 && t < 15.0) {
        return {0,0,0.5 * cos(pi * t / 5.0) + 0.5, 0,0,-(pi / 10.0) * sin(pi * t / 5.0), 0,0,0, 0,0,0};
    } else if (t >= 15.0 && t <= 20.0) {
        return {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    } else {
        // Outside of defined range; handle as needed
        return {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    }
}

std::vector<double> delta_trajectory(double t) { // Replace with waypoint code
    const double pi = 3.141592653589793;

    if (t >= 0.0 && t < 5.0) {
        return {0,0,(pi / 10.0) * sin(pi * t / 5.0), 0,0,(pi * pi / 50.0) * cos(pi * t / 5.0), 0,0,0, 0,0,0};
    } else if (t >= 5.0 && t < 10.0) {
        return {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    } else if (t >= 10.0 && t < 15.0) {
        return {0,0,-(pi / 10.0) * sin(pi * t / 5.0), 0,0,-(pi * pi / 50.0) * cos(pi * t / 5.0), 0,0,0, 0,0,0};
    } else if (t >= 15.0 && t <= 20.0) {
        return {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    } else {
        // Outside of defined range; handle as needed
        return {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    }
}



std::vector<double> readGPSInit(const std::string& filename) {
    /*
        Read the first line of the GPS CSV file to get 
    */
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
    if (std::getline(file, line)) {  // Read second line (first data row) to get init values
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

std::vector<double> subtract(const std::vector<double>& a, const std::vector<double>& b) {
    std::vector<double> result;
    if (a.size() != b.size()) throw std::runtime_error("Vector size mismatch");

    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result.push_back(a[i] - b[i]);

    return result;
}

std::vector<double> multiply(const std::vector<double>& v, double scalar) {
    std::vector<double> result(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * scalar;
    }
    return result;
}

void printVector(const std::vector<double>& v, const std::string& label = "Vector") {
    std::cout << label << " [";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i < v.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

int main() {

    double dt = SET_DT;

    // Test A and B matrix generation

    // // Test data
    // double m = 0.7;
    // double f = 1.2;
    // double cDrag = 0.8;
    // double areaVar = 0.004;

    // Vector full_state(12, 0.0); // State vector (12x1)
    // full_state[0] = 1.0; full_state[1] = 2.0; full_state[2] = 3.0;  // Position
    // full_state[3] = 0.1; full_state[4] = 0.2; full_state[5] = 5;  // Velocity
    // full_state[6] = 0.05; full_state[7] = 0.1; full_state[8] = 0.15;  // Euler Angles
    // full_state[9] = 0.01; full_state[10] = 0.02; full_state[11] = 0.03;  // Angular Velocities

    // Vector full_input(7, 0.0);  // Input vector (7x1)
    // full_input[0] = 5.0; full_input[1] = 0.1; full_input[2] = 0.2;  // Inputs
    // full_input[3] = 0.01; full_input[4] = 0.02; full_input[5] = 0.001;
    // full_input[6] = 0.002;

    // Vector rc(3, 0.0);  // Position vector
    // rc[0] = 0.001; rc[1] = 0.0; rc[2] = -0.001;

    // Vector rt(3, 0.0);  // Rotation vector
    // rt[0] = 0.0; rt[1] = 0.0; rt[2] = -0.2;

    // // Inertia matrices
    // Matrix inertia(3, 3, 0.0);
    // inertia(0, 0) = 1.0;
    // inertia(1, 1) = 1.0;
    // inertia(2, 2) = 1.0;

    // // Matrix inertia_s(3, 3, 0.0);
    // // inertia_s(0, 0) = 1.1; inertia_s(1, 1) = 2.1; inertia_s(2, 2) = 3.1;

    // Matrix inertia_a(3, 3, 0.0);
    // inertia_a(0, 0) = 0.05; inertia_a(1, 1) = 0.05; inertia_a(2, 2) = 0.05;

    // Matrix inertia_b(3, 3, 0.0);
    // inertia_b(0, 0) = 0.05; inertia_b(1, 1) = 0.05; inertia_b(2, 2) = 0.05;

    // // Call calculateA function
    // Matrix B_out = calculateB(m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_a, inertia_b);
    // Matrix A_out = calculateA(m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_a, inertia_b);

    // // Print the resulting A matrix
    // std::cout << "Calculated A matrix:" << std::endl;
    // for (unsigned int i = 0; i < A_out.getRows(); ++i) {  // Iterate over rows (12 rows)
    //     for (unsigned int j = 0; j < A_out.getCols(); ++j) {  // Iterate over columns (12 columns)
    //         std::cout << A_out(i, j) << " ";  // Print each element of A
    //     }
    //     std::cout << std::endl;  // New line after each row
    // }

    // // Print the resulting B matrix
    // std::cout << "Calculated B matrix:" << std::endl;
    // for (unsigned int i = 0; i < B_out.getRows(); ++i) {  // Iterate over rows (12 rows)
    //     for (unsigned int j = 0; j < B_out.getCols(); ++j) {  // Iterate over columns (12 columns)
    //         std::cout << B_out(i, j) << " ";  // Print each element of A
    //     }
    //     std::cout << std::endl;  // New line after each row
    // }

    // Note to anyone using this: CHECK THESE PATHS!!
    std::vector<std::string> files = {
        "imu_data.csv",
        "ax6_imu_data.csv",
        "gps_data.csv",
        "lidar_data.csv",
        "uwb_combined_distances.csv"
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

    // Initialize the values array; this takes all the values
    // at any given time and feeds them to the loop.
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

    // Create a file called latency.csv where we will store
    // performance data. It and state.csv initialize
    // in the cwd (/ControlLoop/)
    std::ofstream latencyFile("latency.csv");
    latencyFile << "Iteration,Latency (ms)\n";

    // Create a file called state.csv to store the state.
    std::ofstream dataFile("state.csv");
    dataFile << "Iteration,x,y,z,vx,vy,vz,theta_x,theta_y,theta_z,omega_x,omega_y,omega_z,thrust,a,b\n";

    // This is the loop boolean. If it goes false, theloop stops.
    // The program sets this to false only once the data runs out,
    // so unless you are using a test dataset the program
    // will run forever.
    bool allFilesHaveData = true;

    // This is the iterator variable. It's useful for printing data
    // to the CSVs.
    int iteration = 0;

    // Track the latency.
    double latency_ms;
    std::vector<double> latencies; 

    // status consists of: < gpsSanityCheck, lidarStatus, >. It controls some
    // logic in the loop.
    std::vector<bool> status = {false, true};

    // Start the clock for processing time
    auto startProcessing = std::chrono::high_resolution_clock::now();

    // Initialize state vector and command output. LoopOutput
    // is a struct with {state, status, command}.
    LoopOutput out;

    // The state vector is {eulerAngles, position, angularVelocity, velocity}
    out.state = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
    std::vector<std::vector<double>> prevState = out.state;
    std::vector<std::vector<double>> state2;

     // The command output is thrust, gimbal angle a, gimbal angle b, adot, bdot, ddot{a}, ddot{b}
    // We only use the first 3 outside the LQR
    out.command = {0,0,0};
    std::vector<double> prevCommand = out.command;
    std::vector<double> command2 = prevCommand;
    std::vector<double> command3;
    

   

    // Read the initial GPS readings via the function at the beginning of this program & print it
    std::vector<double> gps_init = readGPSInit(files[2]);
    std::cout << "gps init: " << gps_init[0] << ", " << gps_init[1] << ", " << gps_init[2] << std::endl; 

    /*  
        Initialize the system. This is a wrapper for what's happening
        in the init.cpp file, so go look there for more details. But,
        the SystemComponents is a struct for the filters used. It contains
        {Madgwick, ekf_xy, ekf_z}. 
    */
    SystemComponents system = init(gps_init, out.state, dt); // Initialize all filters

    // Begin the loop!
    std::cout << "Beginning loop!" << std::endl;
    double total_time = 0;
    double time_spent;
    while (allFilesHaveData) {
        
        // std::cout << "Completion... " << iteration << std::endl;

        // Start clock
        auto start = std::chrono::high_resolution_clock::now();

        // Clear the data from the previous loop.
        values.clear();

        // Ensure the loop stops if no more data is found.
        allFilesHaveData = false;

        // Get the data from the CSVs by looping through them
        size_t maxColumns = 0; 
        for (size_t i = 0; i < fileStreams.size(); ++i) {
            std::vector<double> rowValues;
            if (fileStreams[i].good() && std::getline(fileStreams[i], line)) {  // Only read valid lines
                std::stringstream ss(line);
                std::string cell;
                while (std::getline(ss, cell, ',')) {
                    try {
                        rowValues.push_back(std::stod(cell));
                    } catch (const std::invalid_argument&) {
                        rowValues.push_back(0.0);  // Default to zero if conversion fails
                    }
                }
                // If the files have read data, continue the loop.
                allFilesHaveData = true;  
                if (rowValues.size() > maxColumns) maxColumns = rowValues.size();
            }
            if (!rowValues.empty()) {
                values.push_back(rowValues);
            }
        }

        // If allFilesHaveData is false then break out of the loop.
        if (!allFilesHaveData) break;

        // Ensure missing values are filled in with zero.
        // It might be possible to get rid of this.
        for (auto& row : values) {
            while (row.size() < maxColumns) {
                row.push_back(0.0);  // Fill missing values with 0.0
            }
        }

        std::vector<double> desired_state = trajectory(total_time);
        std::vector<double> delta_desired_state = delta_trajectory(total_time);

        // Basic check on the values to make sure they are sane.
        if (values.size() > 1 && !values[1].empty()) {

            /*
                This is the loop. The output is a struct with {state, status, command.}
                We put in the data values, the last known state, the system (aka filter
                objects), the status (which is a vector bool) controls the system sanity
                state. dt is obviously just a constant (set at 0.001s nominally) <-- set chrono
                
            */
            // std::cout << "[DEBUG] Entered loop() " << iteration << std::endl;
                    
            state2 = prevState;
            prevState = out.state; 
            command3 = command2;
            command2 = prevCommand;
            prevCommand = out.command;
           
            out = loop(values, out.state, state2, system, status, dt, desired_state, delta_desired_state, out.command, command2, command3);
        

            

            auto end = std::chrono::high_resolution_clock::now();

            // Compute the latency from the loop clock.
            latency_ms = std::chrono::duration<double, std::milli>(end - start).count();
            latencies.push_back(latency_ms);

            // Compute the frequency in Hz.
            double frequency_hz = latency_ms > 0 ? 1000.0 / latency_ms : 0.0;
           
            // Log to a latency file to make sure there are no spikes in compute time. A safe value is >2000 Hz. Anything below that is dangerous!
            latencyFile << iteration << "," << std::fixed << std::setprecision(6) << latency_ms << "," << frequency_hz << "\n";

            // Log the state to a data file for review & filter testing vs. original data
            dataFile << iteration << "," << std::fixed << std::setprecision(6) << out.state[0][0] << "," << out.state[0][1] << "," << out.state[0][2] << "," << out.state[1][0] << "," << out.state[1][1] << "," << out.state[1][2] << "," << out.state[2][0] << "," << out.state[2][1] << "," << out.state[2][2] << "," << out.state[3][0] << "," << out.state[3][1] << "," << out.state[3][2] << "," << out.command[0] << "," << out.command[1] << "," << out.command[2] <<"\n";
            iteration = iteration+1;

            // End clock & 
            double remaining_time_ms = std::max(0.0, 1.0 - latency_ms);  
            //std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(remaining_time_ms));
            if (latency_ms > SET_DT*1000) {
                total_time += (latency_ms/1000);
            } else {
                total_time += (SET_DT);
            }
            time_spent = std::max(latency_ms, SET_DT)/1000;


        } else {
            // If that basic check fails start a new loop and
            // hope for the best.
            std::cerr << "EOF" << std::endl;
        }

    } // End loop

    // End processing clock
    auto endProcessing = std::chrono::high_resolution_clock::now();


    // Print performance summary
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
        std::cout << "Total Calculated Processing Time: " << total_time << " s" << std::endl;
    }

    // Close files
    latencyFile.close(); 
    for (auto& fs : fileStreams) {
        if (fs.is_open()) {
            fs.close();
        }
    }

    return 0;
}