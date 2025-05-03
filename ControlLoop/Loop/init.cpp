#include "init.h"
#include "../Filters/EKFs/EKF_xy.h"
#include "../Filters/EKFs/EKF_z.h"
#include "../LQR/lqr.h"

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
    // std::cout << "Initializing system components..." << std::endl;

    // Initialize Madgwick gains + parameters
    double madgwickGain = 1; 
    double madgwickBeta = 0.002;
    double startTime = 0.0;
    std::vector<double> initialOrientation = {1,0,0,0};
    Madgwick madgwickFilter(madgwickGain, madgwickBeta, startTime, initialOrientation);// <-- the error is here

    // Fill the init GPS variables
    INIT_ALTITUDE = gpsInit[2];
    INIT_LON = gpsInit[1];
    INIT_LAT = gpsInit[0];

    // Create the initial state for the ekf_xy filter
    Vector initial_xy_state(4, 0.0);  // [x, y, vx, vy]

    // Define EKF parameters for xy
    double q_scalar_xy = 0.001;
    double r_scalar_xy = 1000; // Larger = smoother, slower response
    double initial_p_xy = 100;
    EKF_Position ekf_xy(initial_xy_state, dt, q_scalar_xy, r_scalar_xy, initial_p_xy);

    // Create the initial state for the ekf_z filter
    Vector initial_z_state(2, 0.0);  // [z, vz]

    double q_scalar_Z = 0.01;
    double r_scalar_Z = 1000;
    double initial_p_Z = 10;


    EKF_Altitude ekf_z(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);

    // Define EKF parameters for z
    double q_scalar_z = 0.01;
    double r_scalar_z = 1000;
    double initial_p_z = 10;

    EKF_Altitude ekf_vx(initial_z_state, dt, q_scalar_z, r_scalar_z, initial_p_z);

    EKF_Altitude ekf_vy(initial_z_state, dt, q_scalar_z, r_scalar_z, initial_p_z);

    EKF_Altitude ekf_vz(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);

   

    double q_scalar_pos = 0.001;
    double r_scalar_pos = 1000;
    double initial_p_pos = 10;

    EKF_Altitude ekf_x(initial_z_state, dt, q_scalar_pos, r_scalar_pos, initial_p_pos);

    EKF_Altitude ekf_y(initial_z_state, dt, q_scalar_pos, r_scalar_pos, initial_p_pos);

    EKF_Altitude ekf_z2(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);

    EKF_Altitude ekf_ax(initial_z_state, dt, q_scalar_pos, r_scalar_pos, 0);

    EKF_Altitude ekf_ay(initial_z_state, dt, q_scalar_pos, r_scalar_pos, 0);

    EKF_Altitude ekf_az(initial_z_state, dt, q_scalar_pos, r_scalar_pos, 0);

    q_scalar_pos = 0.00001;
    r_scalar_pos = 100000;
    initial_p_pos = 10;

    EKF_Altitude ekf_a(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);
    EKF_Altitude ekf_b(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);
    EKF_Altitude ekf_t(initial_z_state, dt, q_scalar_Z, r_scalar_Z, initial_p_Z);

   

    EKF_Altitude ekf_ox(initial_z_state, dt, q_scalar_pos, r_scalar_pos, initial_p_pos);

    EKF_Altitude ekf_oy(initial_z_state, dt, q_scalar_pos, r_scalar_pos, initial_p_pos);

    EKF_Altitude ekf_oz(initial_z_state, dt, q_scalar_pos, r_scalar_pos, initial_p_pos);



    // The EKF parameters for q and z for EKF_Altitude are pretty general, they both provide a
    // smoothing of some sort. Therefore they have been repurposed to smooth out some functions
    // (notably velocities, angular velocities, and control states).
    
    LQR lqrController;

    // Return struct of filters
    return {madgwickFilter, ekf_xy, ekf_z, ekf_x, ekf_y, ekf_z2, ekf_vx, ekf_vy, ekf_vz, ekf_ox, ekf_oy, ekf_oz, ekf_a, ekf_b, ekf_t, ekf_ax, ekf_ay, ekf_az, lqrController};
}
