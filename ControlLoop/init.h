#ifndef INIT_H
#define INIT_H

#include "Madgwick.h"
#include "EKF_xy.h"
#include "EKF_z.h"
#include "Vector.h"  // Include your custom Vector class

// === Filter Wrapper Struct ===
typedef struct {
    Madgwick madgwickFilter;
    EKF_Position ekf_xy;
    EKF_Altitude ekf_z;
} SystemComponents;

// === Initial GPS globals ===
extern double INIT_ALTITUDE;
extern double INIT_LON;
extern double INIT_LAT;



// gpsInit is a Vector (3x1: lat, lon, alt)
// state is a 4x3 raw double array: {euler, position, angularVelocity, velocity}
SystemComponents init(const Vector& gpsInit, double state[4][3], double dt);

#endif // INIT_H
