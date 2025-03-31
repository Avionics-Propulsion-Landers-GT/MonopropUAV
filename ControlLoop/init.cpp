#include "init.h"
#include "EKF_xy.h"
#include "EKF_z.h"
// #include <iostream>
#include <stdio.h>

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
SystemComponents init(const Vector& gpsInit, double state[4][3], double dt) {
    fputs("Initializing system components...\n", stdout);

    // === Madgwick filter ===
    double madgwickGain = 0.1;
    double madgwickBeta = 0.01;
    double startTime = 0.0;

    // Extract initial orientation from state[0]
    Vector initialOrientation(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        initialOrientation(i, 0) = state[0][i];
    }

    Madgwick madgwickFilter(madgwickGain, madgwickBeta, startTime, initialOrientation);

    // === GPS Init Values ===
    INIT_ALTITUDE = gpsInit(2, 0);
    INIT_LON      = gpsInit(1, 0);
    INIT_LAT      = gpsInit(0, 0);

    // === EKF XY ===
    Vector initial_xy_state(4, 0.0);  // [x, y, vx, vy]
    double q_scalar_xy   = 0.01;
    double r_scalar_xy   = 1000;
    double initial_p_xy  = 100;
    EKF_Position ekf_xy(initial_xy_state, dt, q_scalar_xy, r_scalar_xy, initial_p_xy);

    // === EKF Z ===
    Vector initial_z_state(2, 0.0);  // [z, vz]
    double q_scalar_z   = 0.01;
    double r_scalar_z   = 1000;
    double initial_p_z  = 100;
    EKF_Altitude ekf_z(initial_z_state, dt, q_scalar_z, r_scalar_z, initial_p_z);

    // === Return composed filter system ===
    SystemComponents system = {
        madgwickFilter,
        ekf_xy,
        ekf_z
    };
    return system;
}