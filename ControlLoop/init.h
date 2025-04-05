#ifndef INIT_H
#define INIT_H

#include "Madgwick.h"
#include "EKF_xy.h"
#include "EKF_z.h"
#include "lqr.h"

/*

    Header file for init.cpp

*/

// Struct to hold filters
struct SystemComponents {
    Madgwick madgwickFilter;
    EKF_Position ekf_xy;
    EKF_Altitude ekf_z;
    EKF_Altitude ekf_x;
    EKF_Altitude ekf_y;
    EKF_Altitude ekf_z2;
    EKF_Altitude ekf_vx;
    EKF_Altitude ekf_vy;
    EKF_Altitude ekf_vz;
    EKF_Altitude ekf_ox;
    EKF_Altitude ekf_oy;
    EKF_Altitude ekf_oz;
    LQR lqrController;
};


// Initial GPS data
extern double INIT_ALTITUDE;
extern double INIT_LON;
extern double INIT_LAT;

// Function prototype for initializing multiple components
SystemComponents init(std::vector<double> gpsInit, std::vector<std::vector<double>> state, double dt);

#endif // INIT_H