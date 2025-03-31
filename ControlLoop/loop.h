#ifndef LOOP_H
#define LOOP_H

#include <stdbool.h>  // for `bool`
#include "init.h"
#include "Madgwick.h"
#include "lqr.h"

/*
    Header file for loop.cpp — STL-Free Edition
*/

// ==== Constants ====
#define STATE_ROWS    4      // eulerAngles, position, angularVelocity, velocity
#define STATE_COLS    3
#define COMMAND_SIZE  7
#define ERROR_SIZE    12
#define STATUS_FLAGS  2
#define SENSOR_ROWS   5      // 9-axis IMU, 6-axis IMU, GPS, LIDAR, UWB
#define SENSOR_COLS   10     // max column width across all sensors

// ==== Structs ====

typedef struct {
    double state[STATE_ROWS][STATE_COLS];   // 4 groups of 3D vectors
    bool status[STATUS_FLAGS];              // gps + lidar
    double command[COMMAND_SIZE];           // thrust, gimbal a/b, adot, bdot, ddot a/b
    double error[STATE_ROWS][STATE_COLS];               // state - setPoint diff
} LoopOutput;

// ==== Globals ====
extern LQR lqrController;

// ==== Function Prototypes ====
void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double* dY, double* dX);

// Weighted average of 2 equal-length arrays
void weightedAverage(const double* v1, const double* v2, double* out, int length, double w1, double w2);

// Loop function
LoopOutput loop(
    double values[SENSOR_ROWS][SENSOR_COLS],
    double inputState[STATE_ROWS][STATE_COLS],
    SystemComponents* system,
    const bool status[STATUS_FLAGS],
    double dt,
    const double setPoint[ERROR_SIZE],
    const double command[COMMAND_SIZE]
);

#endif // LOOP_H
