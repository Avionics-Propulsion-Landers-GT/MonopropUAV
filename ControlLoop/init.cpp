#include "init.h"
#include "EKF_xy.h"
#include "EKF_z.h"
#include <iostream>

/*

    Init.cpp

    by spencer boebel

    PURPOSE: Initialize the filters for the control loop.

    FOR ELECTRONICS: This thing inherits the dt from the data_mocker.cpp.

*/

// Define global init variables imported from init.h
double INIT_ALTITUDE;
double INIT_LON;
double INIT_LAT;

// Implement SystemComponents filters struct. 
SystemComponents init(std::vector<double> gpsInit, std::vector<std::vector<double>> initState, double dt) {
    std::cout << "Initializing system components..." << std::endl;

    // Initialize Madgwick gains + parameters
    double madgwickGain = 0.1; 
    double madgwickBeta = 0.01;
    double startTime = 0.0;
    std::vector<double> initialOrientation = initState[0];
    Madgwick madgwickFilter(madgwickGain, madgwickBeta, startTime, initialOrientation);

    // Fill the init GPS variables
    INIT_ALTITUDE = gpsInit[2];
    INIT_LON = gpsInit[1];
    INIT_LAT = gpsInit[0];

    // Create the initial state for the ekf_xy filter
    Vector initial_xy_state(4, 0.0);  // [x, y, vx, vy]

    // Define EKF parameters for xy
    double q_scalar_xy = 0.01;
    double r_scalar_xy = 1000; // Larger = smoother, slower response
    double initial_p_xy = 100;
    EKF_Position ekf_xy(initial_xy_state, dt, q_scalar_xy, r_scalar_xy, initial_p_xy);

    // Create the initial state for the ekf_z filter
    Vector initial_z_state(2, 0.0);  // [z, vz]

    // Define EKF parameters for z
    double q_scalar_z = 0.01;
    double r_scalar_z = 1000;
    double initial_p_z = 100;
    EKF_Altitude ekf_z(initial_z_state, dt, q_scalar_z, r_scalar_z, initial_p_z);

    // Return struct of filters
    return {madgwickFilter, ekf_xy, ekf_z};
}
