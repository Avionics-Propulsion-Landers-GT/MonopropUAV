#include "loop.h"
#include "init.h"
#include "lqr.h"
#include "print.h"
// #include <iostream>
// #include <sstream>
// #include <fstream>
// #include <iomanip>
// #include <cmath>
#include <math.h>
#include <stddef.h>
// #include <cppad/cppad.hpp>
// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif

/*

    loop.cpp

    by spencer boebel

    PURPOSE: This is the main loop file. It's job is to take in the values,
    estimate a state based on those + previous values, then generate an optimal
    command based on the current and historical state. 

    THIS CODE IS STILL UNDER CONSTRUCTION.


*/

const double GPS_SANITY_THRESHOLD = 1.5;
const double LIDAR_ALTITUDE_THRESHOLD = 9.0;
const double TIMESTAMP_THRESHOLD = 0.001;

Matrix initAnchors() {
    Matrix anchors(3, 3, 0.0);
    anchors(0, 0) = 10;   anchors(0, 1) = 10;   anchors(0, 2) = 0;
    anchors(1, 0) = -10;  anchors(1, 1) = 10;   anchors(1, 2) = 0;
    anchors(2, 0) = 0;    anchors(2, 1) = -14;  anchors(2, 2) = 0;
    return anchors;
}

const Matrix ANCHORS = initAnchors();


// Define Constants 

const double m = 0.7; // kg; mass of uav
const double f = 1.2; // kg/m3; density of air
const double cDrag = 0.5; // placeholder value
const double prop_BodyCoM_distance = 0.1; // 10cm from UAV prop hinge to the body CoM [placeholder]
const double thrustArm = 0.04; // 4cm thrust arm [placeholder]


// Starting CoM is the origin for the body frame.
const Vector statCoM = Vector(3, 0.0);  // 3x1 vector [0, 0, 0]

// Static inertia tensor as a 3x3 matrix
const Matrix statMoIM = []() {
    Matrix m(3, 3, 0.0);
    m(0, 0) = 0.00940;
    m(1, 1) = 0.00940;
    m(2, 2) = 0.00014;
    return m;
}();

// Convert degree deltas to actual distances using the WSG84 standard Earth ellipsoid.
// This assumes a MSL of zero, we have ignored 3d effects here.
void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double* dY, double* dX) {
    // Convert latitude to radians
    double latRad = lat * 3.14159265358979323846 / 180.0;

    // Compute accurate meters per degree for latitude
    double metersPerDegLat = 111132.92 - 559.82 * cos(2 * latRad) + 
                             1.175 * cos(4 * latRad) - 0.0023 * cos(6 * latRad);

    // Compute accurate meters per degree for longitude
    double metersPerDegLon = 111412.84 * cos(latRad) - 
                             93.5 * cos(3 * latRad) + 
                             0.118 * cos(5 * latRad);

    // Convert degree changes to meters
    *dY = deltaLat * metersPerDegLat;
    *dX = deltaLon * metersPerDegLon;
}

// Do a weighted average of two vector doubles.
void weightedAverage(const double* v1, const double* v2, double* out, int length, double weight1, double weight2) {
    double totalWeight = weight1 + weight2;

    // Protect against divide-by-zero
    if (totalWeight == 0.0) {
        for (int i = 0; i < length; ++i) {
            out[i] = 0.0;
        }
        return;
    }

    for (int i = 0; i < length; ++i) {
        out[i] = (weight1 * v1[i] + weight2 * v2[i]) / totalWeight;
    }
}

Vector trilaterateXY(const Matrix& anchors, double d1, double d2, double d3) {
    // Ensure matrix is 3x3
    if (anchors.getRows() != 3 || anchors.getCols() != 3) {
        // fallback: return empty 2D vector
        return Vector(2, 0.0);
    }

    // Create position vectors from matrix rows
    Vector p1(3, 0.0), p2(3, 0.0), p3(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        p1(i, 0) = anchors(0, i);
        p2(i, 0) = anchors(1, i);
        p3(i, 0) = anchors(2, i);
    }

    // ex = normalize(p2 - p1)
    Vector p2_minus_p1 = p2.add(p1.multiply(-1));
    Vector ex = p2_minus_p1.normalize();

    // i = dot(ex, p3 - p1)
    Vector p3_minus_p1 = p3.add(p1.multiply(-1));
    double i = ex.dotProduct(p3_minus_p1);

    // ey = normalize((p3 - p1) - ex * i)
    Vector ex_i = ex.multiply(i);
    Vector ey_raw = p3_minus_p1.add(ex_i.multiply(-1));
    Vector ey = ey_raw.normalize();

    // j = dot(ey, p3 - p1)
    double j = ey.dotProduct(p3_minus_p1);

    // d = |p2 - p1|
    double d = p2_minus_p1.magnitude();

    // Trilateration equations
    double x = (d1 * d1 - d2 * d2 + d * d) / (2 * d);
    double y = (d1 * d1 - d3 * d3 + i * i + j * j - 2 * i * x) / (2 * j);

    // result = p1 + ex * x + ey * y
    Vector result3D = p1.add(ex.multiply(x)).add(ey.multiply(y));

    // Return XY only
    Vector resultXY(2, 0.0);
    resultXY(0, 0) = result3D(0, 0);
    resultXY(1, 0) = result3D(1, 0);
    return resultXY;
}

// Vector toVector(const std::vector<double>& v) {
//     Vector result(v.size(), 0.0);
//     for (unsigned int i = 0; i < v.size(); ++i) {
//         result(i, 0) = v[i];
//     }
//     return result;
// }

// std::vector<double> toStdVector(const Matrix& mat) {
//     std::vector<double> result;
//     for (unsigned int i = 0; i < mat.getRows(); ++i) {
//         result.push_back(mat(i, 0));  // assuming column vector
//     }
//     return result;
// }


// Execute the control loop.
LoopOutput loop(
    double values[SENSOR_ROWS][SENSOR_COLS],
    double state[STATE_ROWS][STATE_COLS],
    SystemComponents* system,
    const bool status[STATUS_FLAGS],
    double dt,
    const double setPoint[ERROR_SIZE],
    const double command[COMMAND_SIZE]
) {

    // Define output
    LoopOutput out;

    // Read in values
    double nineAxisIMU[10];  // Time, Gyro<[rad/s]>, Accel<[m/s²]>, Mag<[]>
    double sixAxisIMU[6];    // Time, Gyro<[rad/s]>, Accel<[m/s²]>
    double gps[3];           // Latitude [deg], Longitude [deg], Altitude [m above MSL]
    double lidar[1];         // Altitude [m above ground]
    double uwb[4];           // Time, Distance1 [m], Distance2 [m], Distance3 [m]

    // Copy sensor data from `values` array
    for (int i = 0; i < 10; ++i) nineAxisIMU[i] = values[0][i];  // 9-axis IMU
    for (int i = 0; i < 6;  ++i) sixAxisIMU[i]  = values[1][i];  // 6-axis IMU
    for (int i = 0; i < 3;  ++i) gps[i]         = values[2][i];  // GPS
    for (int i = 0; i < 1;  ++i) lidar[i]       = values[3][i];  // LIDAR
    for (int i = 0; i < 4;  ++i) uwb[i]         = values[4][i];  // UWB

    // Read in filters
    Madgwick* madgwickFilter = &system->madgwickFilter;
    EKF_Position* ekf_xy     = &system->ekf_xy;
    EKF_Altitude* ekf_z      = &system->ekf_z;

        

    // -------------------- I. SENSOR FUSION ALGORITHM ------------------------

    // Preprocessing: Triangulation Algorithm

    Vector uwb_xy = trilaterateXY(ANCHORS, uwb[1], uwb[2], uwb[3]);


    // --------------- i. Madgwick Filter for Attitude ------------------------

    // STRATEGY: Run Madgwick on weighted average of 9ax IMU data and 6ax IMU data

    // Gyroscope readings [rad/s]
    double gyro1[3]  = { nineAxisIMU[1], nineAxisIMU[2], nineAxisIMU[3] };
    double gyro2[3]  = { sixAxisIMU[1],  sixAxisIMU[2],  sixAxisIMU[3]  };

    // Accelerometer readings [m/s²]
    double accel1[3] = { nineAxisIMU[4], nineAxisIMU[5], nineAxisIMU[6] };
    double accel2[3] = { sixAxisIMU[4],  sixAxisIMU[5],  sixAxisIMU[6]  };

    // Magnetometer readings [unitless]
    double mag[3]    = { nineAxisIMU[7], nineAxisIMU[8], nineAxisIMU[9] };
    
    // === Convert raw IMU arrays into Vector objects ===
    Vector accel1_vec(3, 0.0), accel2_vec(3, 0.0);
    Vector gyro1_vec(3, 0.0), gyro2_vec(3, 0.0);
    Vector mag_vec(3, 0.0);

    for (int i = 0; i < 3; ++i) {
        accel1_vec(i, 0) = accel1[i];
        accel2_vec(i, 0) = accel2[i];
        gyro1_vec(i, 0)  = gyro1[i];
        gyro2_vec(i, 0)  = gyro2[i];
        mag_vec(i, 0)    = mag[i];
    }

    // === Weight the 9-axis IMU more heavily than 6-axis ===
    Vector accel = accel1_vec.multiply(3.0).add(accel2_vec.multiply(1.0)).multiply(1.0 / 4.0);
    Vector gyro  = gyro1_vec.multiply(3.0).add(gyro2_vec.multiply(1.0)).multiply(1.0 / 4.0);

    // === Convert euler angles (from state[0]) to Vector ===
    Vector euler_attitude(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        euler_attitude(i, 0) = state[0][i];
    }

    // === Convert euler → quaternion
    Vector q = madgwickFilter->eulerToQuaternion(euler_attitude);

    // === Madgwick update step (filtering)
    Vector qnew = madgwickFilter->madgwickUpdate(q, gyro, accel, mag_vec, dt);

    // === Convert quaternion → updated Euler angles
    Vector new_attitude = madgwickFilter->quaternionToEuler(qnew);

    // ---------------------- ii. EKF for Position ----------------------------

    // STRATEGY: We need to split up the altitude from the XY position. Therefore we will do:
    // (a) XY Position Determination ~ Combine UWB + GPS values
    // (b) Z Position Determination ~ Rely entirely on LIDAR and after that rely on GPS altitude. Then put into EKF.
    
    // -------------------------------- (a) ----------------------------
    Vector position(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        position(i, 0) = state[1][i];
    }
    double x_pos1 = uwb_xy[0];
    double y_pos1 = uwb_xy[1];
    double lat = gps[0];
    double lon = gps[1];

    // Get the degree differences based on initial measured positions.
    double deltaLat = lat - INIT_LAT;
    double deltaLon = lon - INIT_LON;
    
    // Fill x_pos2 and y_pos2 with meters based on  WGS84 ellipsoid.
    double x_pos2 = 0.0;
    double y_pos2 = 0.0;
    preciseLatLonToMeters(lat, deltaLat, deltaLon, &x_pos2, &y_pos2);

    bool gpsSanityCheck = status[0];

    // Sanity Check: If x_pos2 or y_pos2 are greater than twice x_pos1 or y_pos1
    // then throw out the gps and only use the UWB
    if ((x_pos2 > GPS_SANITY_THRESHOLD*x_pos1) || (y_pos2 > GPS_SANITY_THRESHOLD*y_pos1)) {
        if (!gpsSanityCheck) {
            //std::cout << "The GPS is too inaccurate. gpsSanityCheck failed." << std::endl;
        }
        
        x_pos2 = x_pos1;
        y_pos2 = y_pos1;
        gpsSanityCheck = true;
    } else {
        if (gpsSanityCheck) {
            // std::cout << "The GPS is accurate again." << std::endl;
        }
        gpsSanityCheck = false;

    }

    Vector measurement_xy(4, 0.0);  // Create 4x1 vector
    measurement_xy(0, 0) = x_pos1;
    measurement_xy(1, 0) = y_pos1;
    measurement_xy(2, 0) = x_pos2;
    measurement_xy(3, 0) = y_pos2;

    ekf_xy->update(measurement_xy);      // Update EKF with measurement
    ekf_xy->predict();                   // Predict next state

    Vector estimated_state_xy = ekf_xy->getState();  // [X, Y, VX, VY]
    double x_actual = estimated_state_xy(0, 0);
    double y_actual = estimated_state_xy(1, 0);

    // -------------------------------- (b) ----------------------------
    double z_pos1 = lidar[0];
    double altitude = gps[2];
    double z_pos2 = altitude - INIT_ALTITUDE; // Measure altitude relative to reference position

    bool lidarStatus = status[1];
    lidarStatus = ((z_pos1 > LIDAR_ALTITUDE_THRESHOLD) || (z_pos2 > LIDAR_ALTITUDE_THRESHOLD)) ? false : true;
    double z_pos = lidarStatus ? z_pos1 : z_pos2;

    Vector measurement_z(2, 0.0);
    measurement_z(0, 0) = z_pos;
    measurement_z(1, 0) = 0.0;  // Initial vertical velocity assumption

    ekf_z->update(measurement_z);
    ekf_z->predict();

    Vector estimated_state_z = ekf_z->getState();
    double z_actual = estimated_state_z(0, 0);

    // ---------------------- iii. checks & postprocessing ----------------------------

    // Combine into one position vector
    double new_position[3] = { x_actual, y_actual, z_actual };

    // Check for timestamp problems
    double time1 = nineAxisIMU[0];
    double time2 = sixAxisIMU[0];
    double time3 = uwb[0];

    if ((time1 - time2 >= TIMESTAMP_THRESHOLD) || (time2 - time3 >= TIMESTAMP_THRESHOLD)) {
        print_str("IMU or UWB timestamps don't match. Continuing, but you have been warned.");
    }

    // === Compute angular velocity (Euler rate) ===
    double angular_velocity[3];
    for (int i = 0; i < 3; ++i) {
        angular_velocity[i] = new_attitude(i, 0) - state[0][i];
    }

    // === Compute linear velocity ===
    double velocity[3];
    for (int i = 0; i < 3; ++i) {
        velocity[i] = new_position[i] - state[1][i];
    }




    // === Write to state vector ===
    for (int i = 0; i < 3; ++i) {
        out.state[0][i] = new_attitude(i, 0);  // euler attitude
        out.state[1][i] = new_position[i];     // position
        out.state[2][i] = angular_velocity[i]; // angular velocity
        out.state[3][i] = velocity[i];         // linear velocity
    }

    // -------------------------- II. LQR --------------------------------------


    // ----------------------- i. set up key quantities ------------------------

    
    
    


    // ---------- ii. Use state matrices to compute optimal controls -----------

    // Vector current_state(12, 0.0);
    // current_state(0, 0) = new_attitude[0];
    // current_state(1, 0) = new_attitude[1];
    // current_state(2, 0) = new_attitude[2];
    // current_state(3, 0) = new_position[0];
    // current_state(4, 0) = new_position[1];
    // current_state(5, 0) = new_position[2];
    // current_state(6, 0) = angular_velocity[0];
    // current_state(7, 0) = angular_velocity[1];
    // current_state(8, 0) = angular_velocity[2];
    // current_state(9, 0) = velocity[0];
    // current_state(10, 0) = velocity[1];
    // current_state(11, 0) = velocity[2];

    // // Set state and setpoint in LQR controller
    // lqrController.setState(current_state);
    // lqrController.setPoint = toVector(setPoint); // Assuming setPoint is already a Vector

    // // Compute error between current state and desired state
    // Matrix neg_setpoint = lqrController.setPoint.multiply(-1.0);
    // Vector state_error = lqrController.getState().add(Vector(neg_setpoint));

    // // Recalculate K if needed (e.g., time-varying system)
    // lqrController.calculateK();

    // // Compute control command: u = -K * state_error
    // Matrix negative_K = lqrController.getK().multiply(-1.0);
    // Matrix control_command = negative_K.multiply(state_error);  // result is 12x1 Matrix

    // std::vector<double> newCommand = toStdVector(control_command);  // control_command is a Matrix (12x1)
    // std::vector<double> error = toStdVector(state_error);           // state_error is a Vector

    for (int i = 0; i < COMMAND_SIZE; ++i) {
        out.command[i] = (i < 3) ? 0.0 : 0.0;  // First 3 are used, rest are placeholder
    }
    
    // === Error placeholder ===
    for (int i = 0; i < STATE_ROWS; ++i) {
        for (int j = 0; j < STATE_COLS; ++j) {
            out.error[i][j] = 0.0;
        }
    }
    
    // === Status ===
    out.status[0] = gpsSanityCheck;
    out.status[1] = lidarStatus;
    
    // === Return complete LoopOutput struct ===
    return out;
}
