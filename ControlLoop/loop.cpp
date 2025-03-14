#include "loop.h"
#include "init.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

/*

    loop.cpp

    by spencer boebel

    PURPOSE: This is the main loop file. It's job is to take in the values,
    estimate a state based on those + previous values, then generate an optimal
    command based on the current and historical state. 

    THIS CODE IS STILL UNDER CONSTRUCTION.


*/


// A print function for high precision to debug degree GPS outputs
void print(double value) {
    std::cout << std::fixed << std::setprecision(8) << value << std::endl;
}

// Convert degree deltas to actual distances using the WSG84 standard Earth ellipsoid.
// This assumes a MSL of zero, we have ignored 3d effects here.
void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double &dY, double &dX) {
    // Convert latitude to radians
    double latRad = lat * M_PI / 180.0;

    // Compute accurate meters per degree for latitude
    double metersPerDegLat = 111132.92 - 559.82 * std::cos(2 * latRad) + 
                              1.175 * std::cos(4 * latRad) - 0.0023 * std::cos(6 * latRad);

    // Compute accurate meters per degree for longitude
    double metersPerDegLon = 111412.84 * std::cos(latRad) - 
                              93.5 * std::cos(3 * latRad) + 
                              0.118 * std::cos(5 * latRad);

    // Convert degree changes to meters
    dY = deltaLat * metersPerDegLat;
    dX = deltaLon * metersPerDegLon;
}

// Do a weighted average of two vector doubles.
std::vector<double> weightedAverage(const std::vector<double>& v1, const std::vector<double>& v2, double weight1, double weight2) {
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("Vectors must be the same size.");
    }

    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = (weight1 * v1[i] + weight2 * v2[i]) / (weight1 + weight2);
    }
    return result;
}

// Execute the control loop.
LoopOutput loop(const std::vector<std::vector<double>>& values, const std::vector<std::vector<double>> state, SystemComponents& system, const std::vector<bool>& status, double dt) {

    // Read in values
    std::vector<double> nineAxisIMU = values[0]; // Time, Gyro<[rad/s]>, Accel<[m/s2]>, Mag<[]>
    std::vector<double> sixAxisIMU = values[1]; // Time, Gyro<[rad/s]>, Accel<[m/s2]>
    std::vector<double> gps = values[2]; // latitude [deg], longitude[deg], altitude[m above MSL]
    std::vector<double> lidar = values[3]; // altitude [m above ground]
    std::vector<double> uwb = values[4]; // Time, X [m], Y [m]

    // Read in filters
    Madgwick& madgwickFilter = system.madgwickFilter;
    EKF_Position& ekf_xy = system.ekf_xy;
    EKF_Altitude& ekf_z = system.ekf_z;
    

    // -------------------- I. SENSOR FUSION ALGORITHM ------------------------


    // --------------- i. Madgwick Filter for Attitude ------------------------

    // STRATEGY: Run Madgwick on weighted average of 9ax IMU data and 6ax IMU data

    std::vector<double> gyro1 = {nineAxisIMU[1], nineAxisIMU[2], nineAxisIMU[3]};
    std::vector<double> gyro2 = {sixAxisIMU[1], sixAxisIMU[2], sixAxisIMU[3]};
    std::vector<double> accel1 = {nineAxisIMU[4], nineAxisIMU[5], nineAxisIMU[6]};
    std::vector<double> accel2 = {sixAxisIMU[4], sixAxisIMU[5], sixAxisIMU[6]};
    std::vector<double> mag = {nineAxisIMU[7], nineAxisIMU[8], nineAxisIMU[9]};
    
    // Weight the 9ax IMU data on a 3:1 ratio with 6ax
    std::vector<double> accel = weightedAverage(accel1, accel2, 3, 1);
    std::vector<double> gyro = weightedAverage(gyro1, gyro2, 3, 1);

    // Read in the attitude state (kept as euler angles)
    std::vector<double> euler_attitude = state[0];
    
    // translate attitude to quaternion
    std::vector<double> q = madgwickFilter.eulerToQuaternion(euler_attitude);

    // Update the quaternion via the Madgwick filter
    std::vector<double> qnew = madgwickFilter.madgwickUpdate(q, gyro, accel, mag, dt);

    // translate attitude back to Euler angles
    std::vector<double> new_attitude = madgwickFilter.quaternionToEuler(qnew);

    // ---------------------- ii. EKF for Position ----------------------------

    // STRATEGY: We need to split up the altitude from the XY position. Therefore we will do:
    // (a) XY Position Determination ~ Combine UWB + GPS values
    // (b) Z Position Determination ~ Rely entirely on LIDAR and after that rely on GPS altitude. Then put into EKF.
    
    // -------------------------------- (a) ----------------------------
    std::vector<double> position = state[1];
    double x_pos1 = uwb[1];
    double y_pos1 = uwb[2];
    double lat = gps[0];
    double lon = gps[1];

    // Get the degree differences based on initial measured positions.
    double deltaLat = lat - INIT_LAT;
    double deltaLon = lon - INIT_LON;
    
    // Fill x_pos2 and y_pos2 with meters based on  WGS84 ellipsoid.
    double x_pos2, y_pos2;
    preciseLatLonToMeters(lat, deltaLat, deltaLon, x_pos2, y_pos2);

    bool gpsSanityCheck = status[0];

    // Sanity Check: If x_pos2 or y_pos2 are greater than twice x_pos1 or y_pos1
    // then throw out the gps and only use the UWB
    if ((x_pos2 > 1.5*x_pos1) || (y_pos2 > 1.5*y_pos1)) {
        if (!gpsSanityCheck) {
            //std::cout << "The GPS is too inaccurate. gpsSanityCheck failed." << std::endl;
        }
        
        x_pos2 = x_pos1;
        y_pos2 = y_pos1;
        gpsSanityCheck = true;
    } else {
        if (gpsSanityCheck) {
            std::cout << "The GPS is accurate again." << std::endl;
        }
        gpsSanityCheck = false;

    }

    Eigen::VectorXd measurement_xy(4); // Create vector of data
    measurement_xy << x_pos1, y_pos1, x_pos2, y_pos2;

    ekf_xy.update(measurement_xy); // Update EKF with data vector
    ekf_xy.predict(); // Predict next state (internal to EKF)
    Eigen::VectorXd estimated_state_xy = ekf_xy.getState(); // Return the "state" variable from EKF, which is a state vector [X,Y, VX, VY]
    double x_actual = estimated_state_xy(0); 
    double y_actual = estimated_state_xy(1); // Ignore velocity components and take only position from state vector

    // -------------------------------- (b) ----------------------------
    double z_pos1 = lidar[0];
    double altitude = gps[2];
    double z_pos2 = altitude - INIT_ALTITUDE; // Measure altitude relative to reference position

    bool lidarStatus = status[1];
    lidarStatus = ((z_pos1 > 9.0) || (z_pos2 > 9.0)) ? false : true;
    double z_pos = lidarStatus ? z_pos1 : z_pos2;

    Eigen::VectorXd measurement_z(2);
    measurement_z << z_pos, 0;

    ekf_z.update(measurement_z);
    ekf_z.predict();
    Eigen::VectorXd estimated_state_z = ekf_z.getState();
    double z_actual = estimated_state_z(0);

    // ---------------------- iii. checks & postprocessing ----------------------------

    // Combine into one position vector
    std::vector<double> new_position = {x_actual, y_actual, z_actual};

    // Check for timestamp problems
    double time1 = nineAxisIMU[0];
    double time2 = sixAxisIMU[0];
    double time3 = uwb[0];

    if ((time1 - time2 >= 0.001) || (time2 - time3 >= 0.001)) {
        std::cout << "IMU or UWB timestamps don't match. Continuing, but you have been warned." << std::endl;
    }

    // Compute velocities
    std::vector<double> angular_velocity = {new_attitude[0] - euler_attitude[0], new_attitude[1] - euler_attitude[1], new_attitude[2] - euler_attitude[2]};
    std::vector<double> velocity = {new_position[0] - position[0], new_position[1] - position[1], new_position[2] - position[2]};

    // Write the state vector
    std::vector<std::vector<double>> newState = {new_attitude, new_position, angular_velocity, velocity};

    // -------------------------- II. LQR --------------------------------------





    std::vector<double> newCommand = {0,0,0};
    std::vector<bool> newStatus = {gpsSanityCheck, lidarStatus};

    return {newState, newStatus, newCommand};
}
