// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <iomanip> 
// #include <chrono>
// #include <numeric>
#include "loop.h"
#include "init.h"
#include "LQR/calculateA.h"
#include "LQR/calculateB.h"
#include "Matrix.h"
#include "Vector.h"
#include <string.h>
#include <ctype.h> 
#include <stdio.h>
#include <time.h>
#include "print.h"

#define MAX_LINE_LENGTH 1024
#define MAX_GPS_COLUMNS 10

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

Vector readGPSInit(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        fputs("Error: Could not open GPS CSV file\n", stderr);
        return Vector(0, 0.0);
    }

    char line[MAX_LINE_LENGTH];

    // Skip the header line
    if (!fgets(line, sizeof(line), file)) {
        fputs("Error: GPS CSV file is empty or header missing!\n", stderr);
        fclose(file);
        return Vector(0, 0.0);
    }

    // Read the first data line
    if (!fgets(line, sizeof(line), file)) {
        fputs("Error: No data found in GPS CSV!\n", stderr);
        fclose(file);
        return Vector(0, 0.0);
    }

    // Trim newline
    line[strcspn(line, "\r\n")] = '\0';

    char* token = strtok(line, ",");
    double values[MAX_GPS_COLUMNS];
    int count = 0;

    while (token && count < MAX_GPS_COLUMNS) {
        values[count++] = strtod(token, NULL);
        token = strtok(NULL, ",");
    }

    fclose(file);

    // Create and populate the custom Vector
    Vector gpsInit(count, 0.0);
    for (int i = 0; i < count; ++i) {
        gpsInit(i, 0) = values[i];
    }

    return gpsInit;
}

bool isNumber(const char* s) {
    if (!s || *s == '\0') return false;

    // Skip leading +/- sign
    if (*s == '+' || *s == '-') ++s;

    bool hasDigits = false;
    bool hasDecimal = false;

    while (*s) {
        if (*s == '.') {
            if (hasDecimal) return false;  // multiple dots = not a number
            hasDecimal = true;
        } else if (!isdigit(*s)) {
            return false;
        } else {
            hasDigits = true;
        }
        ++s;
    }

    return hasDigits;
}

bool isHeader(const char* line) {
    // Make a copy because strtok modifies the input
    char lineCopy[1024];
    strncpy(lineCopy, line, sizeof(lineCopy));
    lineCopy[sizeof(lineCopy) - 1] = '\0';  // Ensure null-termination

    char* token = strtok(lineCopy, ",");

    while (token != NULL) {
        if (!isNumber(token)) {
            return true;  // found a non-numeric field
        }
        token = strtok(NULL, ",");
    }

    return false;  // all tokens are numeric
}

inline void clearSensorValues(double values[SENSOR_ROWS][SENSOR_COLS], const int* sizes, int sensorCount) {
    for (int row = 0; row < sensorCount; ++row) {
        for (int col = 0; col < sizes[row]; ++col) {
            values[row][col] = 0.0;
        }
    }
}


int main() {

    double dt = 0.001;

    // Test A and B matrix generation

    // // Test data
    // double m = 10.0;
    // double f = 5.0;
    // double cDrag = 0.2;
    // double areaVar = 1.0;

    // Vector full_state(12, 0.0); // State vector (12x1)
    // full_state[0] = 1.0; full_state[1] = 2.0; full_state[2] = 3.0;  // Position
    // full_state[3] = 4.0; full_state[4] = 5.0; full_state[5] = 6.0;  // Velocity
    // full_state[6] = 0.1; full_state[7] = 0.2; full_state[8] = 0.3;  // Euler Angles
    // full_state[9] = 0.01; full_state[10] = 0.02; full_state[11] = 0.03;  // Angular Velocities

    // Vector full_input(7, 0.0);  // Input vector (7x1)
    // full_input[0] = 0.5; full_input[1] = 0.4; full_input[2] = 0.3;  // Inputs
    // full_input[3] = 0.1; full_input[4] = 0.2; full_input[5] = 0.05;
    // full_input[6] = 0.06;

    // Vector rc(3, 0.0);  // Position vector
    // rc[0] = 0.1; rc[1] = 0.2; rc[2] = 0.3;

    // Vector rt(3, 0.0);  // Rotation vector
    // rt[0] = 1.0; rt[1] = 2.0; rt[2] = 3.0;

    // // Inertia matrices
    // Matrix inertia(3, 3, 0.0);
    // inertia(0, 0) = 1.0;
    // inertia(1, 1) = 2.0;
    // inertia(2, 2) = 3.0;

    // Matrix inertia_s(3, 3, 0.0);
    // inertia_s(0, 0) = 1.1; inertia_s(1, 1) = 2.1; inertia_s(2, 2) = 3.1;

    // Matrix inertia_a(3, 3, 0.0);
    // inertia_a(0, 0) = 1.2; inertia_a(1, 1) = 2.2; inertia_a(2, 2) = 3.2;

    // Matrix inertia_b(3, 3, 0.0);
    // inertia_b(0, 0) = 1.3; inertia_b(1, 1) = 2.3; inertia_b(2, 2) = 3.3;

    // // Call calculateA function
    // Matrix B_out = calculateB(m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_s, inertia_a, inertia_b);
    // Matrix A_out = calculateA(m, f, cDrag, areaVar, full_state, full_input, rc, rt, inertia, inertia_s, inertia_a, inertia_b);

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
    const char* files[] = {
        "../Sensing&Controls/AttitudeEstimation/monocopter_data.csv",
        "../Sensing&Controls/AttitudeEstimation/noisy_monocopter_data.csv",
        "../Sensing&Controls/AttitudeEstimation/gps_data.csv",
        "../Sensing&Controls/AttitudeEstimation/noisy_lidar_data.csv",
        "uwb_combined_distances.csv"
    };
    
    const int NUM_FILES = sizeof(files) / sizeof(files[0]);

    FILE* fileStreams[NUM_FILES] = {nullptr};
    
    // Open files
    for (int i = 0; i < NUM_FILES; ++i) {
        fileStreams[i] = fopen(files[i], "r");
        if (!fileStreams[i]) {
            printerr_open_fail(files[i]);
            return 1;  // Exit if any file can't be opened
        }
    }

    char line[1024];

    // Initialize the values array; this takes all the values
    // at any given time and feeds them to the loop.
    int sizes[] = { 10,    6,     3,     1,     4 };   
    const int sensorCount = 5;

    double imu9[10]   = { /* Time, Gyro x/y/z, Accel x/y/z, Mag x/y/z */ };
    double imu6[6]    = { /* Time, Gyro x/y/z, Accel x/y/z */ };
    double gps[3]     = { /* Lat, Lon, Alt */ };
    double lidar[1]   = { /* Altitude */ };
    double uwb[4]     = { /* Time, D1, D2, D3 */ };
    
    double values[SENSOR_ROWS][SENSOR_COLS] = {0};  // Initialize all to 0.0

    // Skip header text
    for (int i = 0; i < NUM_FILES; ++i) {
        FILE* fs = fileStreams[i];
    
        if (fs && fgets(line, sizeof(line), fs)) {
            // Remove newline characters (optional)
            line[strcspn(line, "\r\n")] = '\0';
    
            if (isHeader(line)) {
                print_str("Skipping header: ");
                print_str(line);
                fputc('\n', stdout);
    
                // Read and discard next line (actual data starts after header)
                fgets(line, sizeof(line), fs);
            }
        }
    }

    // Create a file called latency.csv where we will store
    // performance data. It and state.csv initialize
    // in the cwd (/ControlLoop/)

    FILE* latencyFile = fopen("latency.csv", "w");
    if (!latencyFile) {
        fputs("Error: Could not create latency.csv\n", stderr);
        // handle error or return
    } else {
        fputs("Iteration,Latency (ms)\n", latencyFile);
    }

    FILE* dataFile = fopen("state.csv", "w");
    if (!dataFile) {
        fputs("Error: Could not create state.csv\n", stderr);
        // handle error or return
    } else {
        fputs("Iteration,x,y,z\n", dataFile);
    }


    // This is the loop boolean. If it goes false, theloop stops.
    // The program sets this to false only once the data runs out,
    // so unless you are using a test dataset the program
    // will run forever.
    bool allFilesHaveData = true;

    // This is the iterator variable. It's useful for printing data
    // to the CSVs.
    int iteration = 0;

    // Track the latency.
    #define MAX_LATENCY_SAMPLES 12000
    double latency_ms;
    double latencies[MAX_LATENCY_SAMPLES];
    int latencyCount = 0;
    double latencySum = 0.0;


    // status consists of: < gpsSanityCheck, lidarStatus, >. It controls some
    // logic in the loop.
    bool status[2] = { false, true };

    // Start the clock for processing time
    struct timespec startProcessing;
    clock_gettime(CLOCK_MONOTONIC, &startProcessing);

    // Initialize state vector and command output. LoopOutput
    // is a struct with {state, status, command}.
    LoopOutput out;

    // Initialize setpoint
    double setPoint[12] = {0}; 

    out = {
        // state
        {
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
        },
        // status
        { false, true },
        // command
        { 0, 0, 0, 0, 0, 0, 0 },
        // error
        {
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
        },
    };
    

    // Read the initial GPS readings via the function at the beginning of this program & print it
    Vector gps_init = readGPSInit(files[2]);

    print_str("gps init: ");
    print_dbl(gps_init(0, 0));
    print_str(", ");
    print_dbl(gps_init(1, 0));
    print_str(", ");
    print_dbl(gps_init(2, 0));
    fputc('\n', stdout);  // newline


    /*  
        Initialize the system. This is a wrapper for what's happening
        in the init.cpp file, so go look there for more details. But,
        the SystemComponents is a struct for the filters used. It contains
        {Madgwick, ekf_xy, ekf_z}. 
    */
    SystemComponents system = init(gps_init, out.state, dt); // Initialize all filters


    // Begin the loop!
    // std::cout << "Beginning loop!" << std::endl;
    int currentRow = 0;
    while (allFilesHaveData) {
        // Clear the data from the previous loop.
        clearSensorValues(values, sizes, sensorCount);


        // Ensure the loop stops if no more data is found.
        allFilesHaveData = false;

        // Get the data from the CSVs by looping through them
        size_t maxColumns = 0; 
        for (int i = 0; i < NUM_FILES; ++i) {
            if (fileStreams[i] != NULL && fgets(line, sizeof(line), fileStreams[i]) != NULL) {
                // Tokenize line
                char* token = strtok(line, ",");
                int col = 0;
        
                while (token != NULL && col < SENSOR_COLS) {
                    char* endptr;
                    double val = strtod(token, &endptr);
        
                    if (endptr == token) {
                        val = 0.0;  // conversion failed
                    }
        
                    values[currentRow][col] = val;
                    token = strtok(NULL, ",");
                    col++;
                }
        
                // Fill missing values with zeros
                for (; col < SENSOR_COLS; ++col) {
                    values[currentRow][col] = 0.0;
                }
        
                if (col > maxColumns) maxColumns = col;
                allFilesHaveData = true;
                currentRow++;
            }
        }
        

        // If allFilesHaveData is false then break out of the loop.
        if (!allFilesHaveData) break;

        // Ensure missing values are filled in with zero.
        // It might be possible to get rid of this.
        // for (int r = 0; r < SENSOR_ROWS; ++r) {
        //     for (int c = maxColumns; c < SENSOR_COLS; ++c) {
        //         values[r][c] = 0.0;
        //     }
        // }

        // Basic check on the values to make sure they are sane.

        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);

        /*
            This is the loop. The output is a struct with {state, status, command.}
            We put in the data values, the last known state, the system (aka filter
            objects), the status (which is a vector bool) controls the system sanity
            state. dt is obviously just a constant (set at 0.001s nominally)
            
        */
        // std::cout << "[DEBUG] Entered loop() " << iteration << std::endl;
        out = loop(values, out.state, &system, status, dt, setPoint, out.command);
        

        // End clock
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &end);

        // Compute the latency from the loop clock.
        latency_ms = (end.tv_sec - start.tv_sec) * 1000.0 +
             (end.tv_nsec - start.tv_nsec) / 1e6;

        latencySum += latency_ms;
        latencyCount++;

        // Compute the frequency in Hz.
        double frequency_hz = latency_ms > 0 ? 1000.0 / latency_ms : 0.0;
        
        // Log to a latency file to make sure there are no spikes in compute time. A safe value is >2000 Hz. Anything below that is dangerous!
        fprintf(latencyFile, "%d,%.6f,%.6f\n", iteration, latency_ms, frequency_hz);

        // Log the state to a data file for review & filter testing vs. original data
        fprintf(dataFile, "%d,%.6f,%.6f,%.6f\n", iteration, out.state[1][0], out.state[1][1], out.state[1][2]);
        
        iteration++;

    } // End loop

    // End processing clock
    struct timespec endProcessing;
    clock_gettime(CLOCK_MONOTONIC, &endProcessing);

    // Print performance summary
    if (latencyCount > 0) {
        double avg_latency = latencySum / latencyCount;
        double avg_frequency = avg_latency > 0.0 ? 1000.0 / avg_latency : 0.0;
    
        double processingTime = (endProcessing.tv_sec - startProcessing.tv_sec) * 1000.0 +
                                (endProcessing.tv_nsec - startProcessing.tv_nsec) / 1e6;
    
        fprintf(latencyFile, "\nAverage Latency (ms),%.6f\n", avg_latency);
        fprintf(latencyFile, "Average Frequency (Hz),%.6f\n", avg_frequency);
    
        printf("\n=== Performance Summary ===\n");
        printf("Average Latency: %.6f ms\n", avg_latency);
        printf("Average Frequency: %.6f Hz\n", avg_frequency);
        printf("Total Processing Time: %.6f ms\n", processingTime);
    }

    // Close files
    fclose(latencyFile);
    fclose(dataFile);

    for (int i = 0; i < NUM_FILES; ++i) {
        if (fileStreams[i] != NULL) {
            fclose(fileStreams[i]);
        }
    }


    return 0;
}