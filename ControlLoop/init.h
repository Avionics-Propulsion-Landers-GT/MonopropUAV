#ifndef INIT_H
#define INIT_H

#include "Madgwick.h"
#include "EKF_xy.h"
#include "EKF_z.h"

// #include "Matrix.h"
// #include "Vector.h"

// Struct to hold multiple initialized objects
struct SystemComponents {
    Madgwick madgwickFilter;
    EKF_Position ekf_xy;
    EKF_Altitude ekf_z;
};

extern double INIT_ALTITUDE;
extern double INIT_LON;
extern double INIT_LAT;

// Function prototype for initializing multiple components
SystemComponents init(std::vector<double> gpsInit, std::vector<std::vector<double>> state);

#endif // INIT_H