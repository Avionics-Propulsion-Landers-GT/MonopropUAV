#include "init.h"
#include "EKF_xy.h"
#include <iostream>

double INIT_ALTITUDE;
double INIT_LON;
double INIT_LAT;

SystemComponents init(std::vector<double> gpsInit, std::vector<std::vector<double>> initState) {
    std::cout << "Initializing system components..." << std::endl;

    double madgwickGain = 0.1; 
    double madgwickBeta = 0.01;
    double startTime = 0.0;
    std::vector<double> initialOrientation = initState[0];
    Madgwick madgwickFilter(madgwickGain, madgwickBeta, startTime, initialOrientation);

    INIT_ALTITUDE = gpsInit[2];
    INIT_LON = gpsInit[1];
    INIT_LAT = gpsInit[0];

    Eigen::VectorXd initial_xy_state(4);
    initial_xy_state << 0, 0, 0, 0;


    double dt = 0.001;
    double q_scalar_xy = 0.01;
    double r_scalar_xy = 1000; // Make these two bigger for better smoothing 
    double initial_p_xy = 100; // but at an increased risk of acceleration lag
    
    EKF_Position ekf_xy(initial_xy_state, dt, q_scalar_xy, r_scalar_xy, initial_p_xy);

    Eigen::VectorXd initial_z_state(2);
    initial_z_state << 0, 0;

    double q_scalar_z = 0.01;
    double r_scalar_z = 1000;
    double initial_p_z = 100;

    EKF_Altitude ekf_z(initial_z_state, dt, q_scalar_z, r_scalar_z, initial_p_z);


    return {madgwickFilter, ekf_xy, ekf_z};
}
