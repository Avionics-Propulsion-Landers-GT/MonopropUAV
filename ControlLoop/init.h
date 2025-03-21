#ifndef INIT_H
#define INIT_H

#include "Madgwick.h"
#include "EKF_xy.h"
#include "EKF_z.h"

/*

    Header file for init.cpp

*/

// Struct to hold filters
struct SystemComponents {
    Madgwick madgwickFilter;
    EKF_Position ekf_xy;
    EKF_Altitude ekf_z;
};


// Initial GPS data
extern double INIT_ALTITUDE;
extern double INIT_LON;
extern double INIT_LAT;

// Function prototype for initializing multiple components
SystemComponents init(std::vector<double> gpsInit, std::vector<std::vector<double>> state, double dt);

#endif // INIT_H