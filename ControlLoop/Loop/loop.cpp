#include "loop.h"
#include "init.h"
#include "../LQR/lqr.h"
#include "../LQR/calculateA.h"
#include "../LQR/calculateB.h"
#include "../LQR/calculateABF.h"
#include "../LQR/calculateBBF.h"
#include <iostream>


/*

    in.cpp

    by spencer boebel

    PURPOSE: This is the main in file. It's job is to take in the values,
    estimate a state based on those + previous values, then generate an optimal
    command based on the current and historical state. 

    THIS CODE IS STILL UNDER CONSTRUCTION.


*/

/*
                       -=-  CONSTANTS -=-
*/ 

const double GPS_SANITY_THRESHOLD = 1.5;
const double LIDAR_ALTITUDE_THRESHOLD = 9.0;
const double TIMESTAMP_THRESHOLD = 0.001;

// Triangulation stuff
Matrix initAnchors() {
    Matrix anchors(3, 3, 0.0);
    anchors(0, 0) = 10;   anchors(0, 1) = 10;   anchors(0, 2) = 0;
    anchors(1, 0) = -10;  anchors(1, 1) = 10;   anchors(1, 2) = 0;
    anchors(2, 0) = 0;    anchors(2, 1) = -14;  anchors(2, 2) = 0;
    return anchors;
}
const Matrix ANCHORS = initAnchors();

// Structures numbers
const double M = 0.7; // kg; mass of uav
const double F = 1.225; // kg/m3; density of air
const std::vector<double> INERTIA = {0.00940, 0, 0, 0, 0.00940, 0, 0, 0, 0.00014}; // kgm2; static inertia tensor about cylidner CoM
const double THRUST_OFFSET = 7*24*0.001; // thrust offset from CoM in meters
std::vector<double> G_VECTOR = {0,0,0, 0,0,-9.80665, 0,0,0, 0,0,0};

static Vector IE = Vector(12,0);
static Vector static_input = Vector(3, 0.0);


// Q & R Matrices
const std::vector<double> Q_MATRIX = { // 12 x 12
    /* Starting values based of Bryson's Rule */
    // Position (x, y, z)
    0.0, 0.00, 0.00,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    0.00, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    0.00, 0.00, 400.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    // Linear velocity (vx, vy, vz)
    0.0, 0.0, 0.0,  0.0, 0.00, 0.00,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,  0.00, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,  0.00, 0.00, 100.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
    // Angular position (roll, pitch, yaw)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  100.0, 0.00, 0.00,  0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.00, 100.0, 0.00,  0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.00, 0.00, 0.0,  0.0, 0.0, 0.0,
    // Angular velocity (wx, wy, wz)
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  2.5, 0.00, 0.00,
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.00, 2.5, 0.00,
    0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.00, 0.00, 0.0
}; 
const std::vector<double> R_MATRIX = { // 3 x 3
/* Starting values based of Bryson's Rule */
    1.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 
    0.0, 0.0, 1.0, 
};


/*
                     -=-  HELPER FUNCTIONS -=-
*/

void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double &dY, double &dX) {
    // Convert degree deltas to actual distances using the WSG84 standard Earth ellipsoid.
    // This assumes a MSL of zero, we have ignored 3d effects here.
    double latRad = lat * 3.14159265358979323846 / 180.0;

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

std::vector<double> weightedAverage(const std::vector<double>& v1, const std::vector<double>& v2, double weight1, double weight2) {
    // Do a weighted average of two vector doubles.
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("Vectors must be the same size.");
    }

    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = (weight1 * v1[i] + weight2 * v2[i]) / (weight1 + weight2);
    }
    return result;
}

Vector toVector(const std::vector<double>& v) {
    Vector result(v.size(), 0.0);
    for (unsigned int i = 0; i < v.size(); ++i) {
        result(i, 0) = v[i];
    }
    return result;
}

Matrix toMatrix(const std::vector<double>& v) {
    if (v.size() != 9) {
        throw std::invalid_argument("Input vector must have exactly 9 elements.");
    }

    Matrix result(3, 3, 0.0);  // 3 rows, 3 cols, initialize to 0.0
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            result(i, j) = v[i * 3 + j];  // row-major order
        }
    }
    return result;
}

Matrix toRectMatrix(const std::vector<double>& v, unsigned int m, unsigned int n) {
    if (v.size() != m * n) {
        // std::cerr << "[ERROR] Vector size does not match matrix dimensions (" 
        //          << v.size() << " vs " << m << "x" << n << ").\n";
        return Matrix(0, 0, 0.0);
    }

    Matrix result(m, n, 0.0);  // m rows, n cols, initialized to 0.0
    for (unsigned int i = 0; i < m; ++i) {
        for (unsigned int j = 0; j < n; ++j) {
            result(i, j) = v[i * n + j];  // row-major order
        }
    }

    return result;
}

std::vector<double> toStdVector(const Matrix& mat) {
    std::vector<double> result;
    for (unsigned int i = 0; i < mat.getRows(); ++i) {
        result.push_back(mat(i, 0));  // assuming column vector
    }
    return result;
}

/*
                     -=-  STRUCTURES REGRESSIONS -=-
*/

double cdRegression(double AoA) {
    return ((2.5/90)*AoA+(1.5/10000)*(AoA*AoA)); //placeholder regression, accurate enough
}

double areaRegression(double AoA) {
    return 0.0072382 + 0.000226775*AoA;
}

double copRegression(double AoA) {
    return 0.048 - 0.00053333333333*AoA; // z-direction offset of CoP
}

std::vector<double> getInertiaA(double a) {
    double m = 0.05; // kg
    double r = 0.01;
    double R = 0.096;
    double factor = m*(R*R + ((3/4)*(r*r)));
    double Ix = 0.5*m*r*r;
    return {Ix, 0, 0, 0, factor*cos(a), -factor*sin(a), 0, factor*sin(a), factor*cos(a)};
}

std::vector<double> getInertiaB(double b) {
    double m = 0.05; // kg
    double r = 0.01;
    double R = 0.096;
    double factor = m*(R*R + ((3/4)*(r*r)));
    double Ix = 0.5*m*r*r;
    return {Ix*cos(b), 0, Ix*sin(b), 0, factor, 0, -factor*sin(b), 0, factor*cos(b)};

}


/*
                     -=-  TRILATERATION -=-
*/

Vector trilaterateXY(const Matrix& anchors, double d1, double d2, double d3) {
    if (anchors.getRows() != 3 || anchors.getCols() != 3) {
        throw std::invalid_argument("Anchors matrix must be 3x3: [3 anchors x (x, y, z)]");
    }

    // Extract anchor positions into Vector objects (one row per anchor)
    Vector p1(3, 0.0), p2(3, 0.0), p3(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        p1(i, 0) = anchors(0, i);
        p2(i, 0) = anchors(1, i);
        p3(i, 0) = anchors(2, i);
    }

    Vector p2_minus_p1 = Vector(p2.add(p1.multiply(-1)));
    Vector ex = p2_minus_p1.normalize();
    Vector p3_minus_p1 = Vector(p3.add(p1.multiply(-1)));
    double i = ex.dotProduct(p3_minus_p1);
    Vector ex_i = Vector(ex.multiply(i));
    Vector ey_raw = Vector(p3_minus_p1.add(ex_i.multiply(-1))); Vector ey = ey_raw.normalize();
    double j = ey.dotProduct(p3_minus_p1); double d = p2_minus_p1.magnitude();

    // Trilateration equations
    double x = (d1 * d1 - d2 * d2 + d * d) / (2 * d);
    double y = (d1 * d1 - d3 * d3 + i * i + j * j - 2 * i * x) / (2 * j);

    // result3D = p1 + ex * x + ey * y
    Vector ex_x = Vector(ex.multiply(x)); Vector ey_y = Vector(ey.multiply(y)); Vector result3D = Vector(p1.add(ex_x).add(ey_y));

    // Extract XY only
    Vector resultXY(2, 0.0); resultXY(0, 0) = result3D(0, 0); resultXY(1, 0) = result3D(1, 0);

    return resultXY;
}

void printVector(const std::vector<double>& vec) {

    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}


double frobeniusNorm(Matrix a) {
    double sum = 0.0;
    unsigned int rows = a.getRows();
    unsigned int cols = a.getCols();

    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            double val = a(i, j);
            sum += val * val;
        }
    }
    return std::sqrt(sum);
}


/*
                     -=- LOOP -=-
*/

LoopOutput loop(LoopInput in) {
    // Begin control loop

    // Read in structures
    std::vector<std::vector<double>>& values = in.values;
    std::vector<std::vector<double>>& state = in.state;
    std::vector<std::vector<double>>& prevState = in.prevState;
    SystemComponents& system = in.system;
    std::vector<bool>& status = in.status;
    double dt = in.dt;
    std::vector<double>& desired_state = in.desired_state;
    std::vector<double>& delta_desired_state = in.delta_desired_state; 
    const std::vector<double>& command = in.command;
    const std::vector<double>& prevCommand = in.prevCommand; 
    const std::vector<double>& prevPrevCommand = in.prevPrevCommand; 
   
    // Read in values
    std::vector<double> nineAxisIMU = values[0]; // Time, Gyro<[rad/s]>, Accel<[m/s2]>, Mag<[]>
    std::vector<double> sixAxisIMU = values[1]; // Time, Gyro<[rad/s]>, Accel<[m/s2]>
    std::vector<double> gps = values[2]; // latitude [deg], longitude[deg], altitude[m above MSL]
    std::vector<double> lidar = values[3]; // Time, altitude [m above ground]
    std::vector<double> uwb = values[4]; // Time, Distance1 [m] Distance2 [m] Distance3 [m] 

    // Read in filters
    Madgwick& madgwickFilter = system.madgwickFilter;
    EKF_Position& ekf_xy = system.ekf_xy;
    EKF_Altitude& ekf_z = system.ekf_z;
    EKF_Altitude& ekf_x = system.ekf_x;
    EKF_Altitude& ekf_y = system.ekf_y;
    EKF_Altitude& ekf_z2 = system.ekf_z2;
    EKF_Altitude& ekf_vx = system.ekf_vx;
    EKF_Altitude& ekf_vy = system.ekf_vy;
    EKF_Altitude& ekf_vz = system.ekf_vz;
    EKF_Altitude& ekf_ox = system.ekf_ox;
    EKF_Altitude& ekf_oy = system.ekf_oy;
    EKF_Altitude& ekf_oz = system.ekf_oz;
    EKF_Altitude& ekf_ax = system.ekf_ax;
    EKF_Altitude& ekf_ay = system.ekf_ay;
    EKF_Altitude& ekf_az = system.ekf_az;
    // EKF_Altitude& ekf_thrust = system.ekf_thrust;
    LQR& lqrController = system.lqrController;

    // Constants
    double m = M;
    double f = F;

    // -------------------- I. SENSOR FUSION ALGORITHM ------------------------

    // Preprocessing: Triangulation Algorithm
    Vector uwb_xy = trilaterateXY(ANCHORS, uwb[1], uwb[2], uwb[3]);


    // --------------- i. Madgwick Filter for Attitude ------------------------

    // STRATEGY: Run Madgwick on weighted average of 9ax IMU data and 6ax IMU data
    std::vector<double> gyro1 = {nineAxisIMU[1], nineAxisIMU[2], nineAxisIMU[3]};
    std::vector<double> gyro2 = {sixAxisIMU[1], sixAxisIMU[2], sixAxisIMU[3]};
    std::vector<double> accel1 = {nineAxisIMU[4], nineAxisIMU[5], nineAxisIMU[6]};
    std::vector<double> accel2 = {sixAxisIMU[4], sixAxisIMU[5], sixAxisIMU[6]};
    std::vector<double> mag = {nineAxisIMU[7], nineAxisIMU[8], nineAxisIMU[9]};

    // Weight the 9ax IMU data on a 0:1 ratio with 6ax (we're not using 9ax anymore)
    std::vector<double> accel = weightedAverage(accel1, accel2, 0, 1);
    std::vector<double> gyro = weightedAverage(gyro1, gyro2, 0, 1);

    // std::cout << "before: \n";
    // toVector(gyro).print();

    // std::cout << "after: \n";
    // toVector(gyro).print();


    // Set up and update Madgwick filter
    // std::vector<double> euler_attitude = state[2];
    
    // std::vector<double> q = madgwickFilter.eulerToQuaternion(euler_attitude);
    // std::vector<double> qnew = madgwickFilter.madgwickUpdate(q, gyro, accel, mag, dt);
    // std::vector<double> new_attitude = madgwickFilter.quaternionToEuler(qnew);

    // std::vector<double> euler_attitude = state[2];
    // std::vector<double> q = madgwickFilter.eulerToQuaternion(euler_attitude);
    // std::vector<double> ang_vel_q = {0, gyro[0], gyro[1], gyro[2]};
    // std::vector<double> qnew = madgwickFilter.quaternionMultiply(ang_vel_q, q);
    // for (double& val : qnew) val *= dt * 0.5;
    // // std::vector<double> new_attitude = {state[2][0] + gyro[0]*dt, state[2][1] + gyro[1]*dt, state[2][2] + gyro[2]*dt};
    // std::vector<double> new_attitude = madgwickFilter.quaternionToEuler(qnew);

    // double accel_sum = 0.0;
    // for (double val : accel) {
    //     accel_sum += val * val;
    // }
    // std::vector<double> accel_norm = {accel[0] / std::sqrt(accel_sum), accel[1] / std::sqrt(accel_sum), accel[2] / std::sqrt(accel_sum)};
    // double accel_roll = std::atan2(accel_norm[1], accel_norm[2]);
    // double accel_pitch = std::atan2(-accel_norm[0], std::sqrt(accel_norm[1] * accel_norm[1] + accel_norm[2] * accel_norm[2]));


    // //TODO: tune this complementary factor:
    // double accel_factor = 0.5;
    // new_attitude[0] = new_attitude[0] * (1 - accel_factor) + accel_roll * accel_factor;
    // new_attitude[1] = new_attitude[1] * (1 - accel_factor) + accel_pitch * accel_factor;

    std::vector<double> new_attitude = {prevState[2][0] + 2*gyro[0]*dt, prevState[2][1] + 2*gyro[1]*dt, prevState[2][2] + 2*gyro[2]*dt};


    

    // ---------------------- ii. EKF for Position ----------------------------

    // STRATEGY: We need to split up the altitude from the XY position. Therefore we will do:
    // (a) XY Position Determination ~ Combine UWB + GPS values
    // (b) Z Position Determination ~ Rely entirely on LIDAR and after that rely on GPS altitude. Then put into EKF.
    
    // -------------------------------- (a) ----------------------------
    std::vector<double> position = state[0];
    std::vector<double> attitude = state[2];
    double x_pos1 = uwb_xy[0];
    double y_pos1 = uwb_xy[1];
    double lat = gps[0];
    double lon = gps[1];

    // Get the degree differences based on initial measured positions.
    double deltaLat = lat - INIT_LAT;
    double deltaLon = lon - INIT_LON;
    
    // Fill x_pos2 and y_pos2 with meters based on  WGS84 ellipsoid.
    double x_pos2, y_pos2;
    preciseLatLonToMeters(lat, deltaLat, deltaLon, x_pos2, y_pos2); // Convert lat + lon to m (for gps)

    Vector measurement_xy(4, 0.0);  // Create 4x1 vector
    measurement_xy(0, 0) = x_pos1;
    measurement_xy(1, 0) = y_pos1;
    measurement_xy(2, 0) = x_pos1; // can change to use ony uwb
    measurement_xy(3, 0) = y_pos1;
    ekf_xy.update(measurement_xy);      // Update EKF with measurement 
    ekf_xy.predict();                   // Predict next state
    Vector estimated_state_xy = ekf_xy.getState();  // [X, Y, VX, VY]
    double x_actual = estimated_state_xy(0, 0);
    double y_actual = estimated_state_xy(1, 0);


    // -------------------------------- (b) ----------------------------

    double z_pos1 = lidar[1];
    double altitude = gps[2];
    double z_pos2 = altitude - INIT_ALTITUDE; // Measure altitude relative to reference position

    bool lidarStatus = status[1];
    lidarStatus = ((z_pos1 > LIDAR_ALTITUDE_THRESHOLD) || (z_pos2 > LIDAR_ALTITUDE_THRESHOLD)) ? false : true;
    double z_pos = lidarStatus ? z_pos1 : z_pos2;

    Vector measurement_z(2, 0.0);
    measurement_z(0, 0) = z_pos; measurement_z(1, 0) = 0;  // Initial vertical velocity assumption

    ekf_z.update(measurement_z); ekf_z.predict();
    Vector estimated_state_z = ekf_z.getState(); double z_actual = estimated_state_z(0, 0);

    // ----------------- iii. Low pass EKFs and postprocessing -----------------------

    // Slap a low pass filter on position
    Vector measurement_x(2, 0.0); Vector measurement_y(2, 0.0); Vector measurement_z2(2, 0.0);
    measurement_x(0, 0) = x_actual; measurement_x(1, 0) = 0;
    measurement_y(0, 0) = y_actual; measurement_y(1, 0) = 0;
    measurement_z2(0, 0) = z_actual; measurement_z2(1, 0) = 0;
    ekf_x.update(measurement_x); ekf_x.predict();
    ekf_y.update(measurement_y); ekf_y.predict();
    ekf_z2.update(measurement_z2); ekf_z2.predict(); 
    // Vector estimated_state_x = ekf_x.getState(); x_actual = estimated_state_x(0, 0);
    // Vector estimated_state_y = ekf_y.getState(); y_actual = estimated_state_y(0, 0);
    Vector estimated_state_z2 = ekf_z2.getState(); z_actual = estimated_state_z2(0, 0);

    // Slap a low pass filter onto velocity
    double vx = (x_actual - prevState[0][0])/(2*dt); 
    double vy = (y_actual - prevState[0][1])/(2*dt);
    double vz = (z_actual - prevState[0][2])/(2*dt);
    Vector measurement_vx(2, 0.0); Vector measurement_vy(2, 0.0); Vector measurement_vz(2, 0.0);
    measurement_vx(0, 0) = vx; measurement_vx(1, 0) = 0;
    measurement_vy(0, 0) = vy; measurement_vy(1, 0) = 0; 
    measurement_vz(0, 0) = vz; measurement_vz(1, 0) = 0;  
    ekf_vx.update(measurement_vx); ekf_vx.predict();
    ekf_vy.update(measurement_vy); ekf_vy.predict();
    ekf_vz.update(measurement_vz); ekf_vz.predict();
    Vector estimated_state_vx = ekf_vx.getState(); double vx_actual = estimated_state_vx(0, 0);
    Vector estimated_state_vy = ekf_vy.getState(); double vy_actual = estimated_state_vy(0, 0);
    Vector estimated_state_vz = ekf_vz.getState(); double vz_actual = estimated_state_vz(0, 0);

    // Slap a low pass filter onto angular velocity
    double ox = (new_attitude[0] - prevState[2][0])/(2*dt); // gyro[0];
    double oy = (new_attitude[1] - prevState[2][1])/(2*dt); // gyro[1];
    double oz = (new_attitude[2] - prevState[2][2])/(2*dt); // gyro[2];

    Vector measurement_ox(2, 0.0); Vector measurement_oy(2, 0.0); Vector measurement_oz(2, 0.0);
    measurement_ox(0, 0) = ox; measurement_ox(1, 0) = 0;
    measurement_oy(0, 0) = oy; measurement_oy(1, 0) = 0; 
    measurement_oz(0, 0) = oz; measurement_oz(1, 0) = 0;  
    ekf_ox.update(measurement_ox); ekf_ox.predict();
    ekf_oy.update(measurement_oy); ekf_oy.predict();
    ekf_oz.update(measurement_oz); ekf_oz.predict();
    Vector estimated_state_ox = ekf_ox.getState(); double ox_actual = estimated_state_ox(0, 0);
    Vector estimated_state_oy = ekf_oy.getState(); double oy_actual = estimated_state_oy(0, 0);
    Vector estimated_state_oz = ekf_oz.getState(); double oz_actual = estimated_state_oz(0, 0);


    // Check for timestamp problems
    double time1 = nineAxisIMU[0];
    double time2 = sixAxisIMU[0];
    double time3 = uwb[0];

    if ((time1 - time2 >= TIMESTAMP_THRESHOLD) || (time2 - time3 >= TIMESTAMP_THRESHOLD)) {
        // std::cout << "IMU or UWB timestamps don't match. Continuing, but you have been warned." << std::endl;
    }

    // Calculate new state vector based on measurements
    std::vector<double> angular_velocity = {ox_actual, oy_actual, oz_actual};
    std::vector<double> velocity = {vx_actual, vy_actual, vz_actual};
    std::vector<double> new_position = {x_actual, y_actual, z_actual};

    std::vector<std::vector<double>> newState = {new_position, velocity, new_attitude, angular_velocity};
  
    // Calculate gimbal angle states
    double omega_a = (command[1]-prevCommand[1])/dt; double omega_b = (command[2]-prevCommand[2])/dt;
    double omega_a_prev = (prevCommand[1]-prevPrevCommand[1])/dt; double omega_b_prev = (prevCommand[2]-prevPrevCommand[2])/dt;
    double alpha_a = (omega_a-omega_a_prev)/dt; double alpha_b = (omega_b-omega_b_prev)/dt;

    std::vector<double> angular_states = {omega_a, omega_b, alpha_a, alpha_b};



    // -------------------------- II. LQR --------------------------------------

    // ----------------------- i. set up key quantities ------------------------

    // 1. Find AoA by dealing with dot product of world frame z axis and wf velocity
    double AoA;
    std::vector<double> z_wf = {sin(state[2][0]), -cos(state[2][0])*sin(state[2][1]), cos(state[2][0])*cos(state[2][1])};
    double v_norm = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
    double z_norm = std::sqrt(z_wf[0]*z_wf[0] + z_wf[1]*z_wf[1] + z_wf[2]*z_wf[2]);

    // Check
    if (v_norm == 0.0 || z_norm == 0.0) {
        AoA = 0;  // avoid division by zero, define AoA as 0
    }

    double dot = -(velocity[0] * z_wf[0] + velocity[1] * z_wf[1] + velocity[2] * z_wf[2]);
    double cos_alpha = dot / (v_norm * z_norm);
    cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));
    AoA = std::acos(cos_alpha)*(180/3.14159265358979323846);

    // 2. From AoA grab Cd, area, and rc
    double Cd = cdRegression(AoA);
    double area = areaRegression(AoA);
    double CoP_offset = copRegression(AoA);
    std::vector<double> rc_raw = {0, 0, 0}; // -COP_OFFSET
    Vector rc = toVector(rc_raw);

    // 3. Construct rt
    std::vector<double> rt_raw = {0, 0, -THRUST_OFFSET}; // May not be totally accurate
    Vector rt = toVector(rt_raw);

    // 4. Construct flat state vector and put it in custom class
    Vector current_state(12, 0.0);
    current_state(0, 0) = new_position[0]; current_state(1, 0) = new_position[1]; current_state(2, 0) = new_position[2];
    current_state(3, 0) = velocity[0]; current_state(4, 0) = velocity[1]; current_state(5, 0) = velocity[2];
    current_state(6, 0) = new_attitude[0]; current_state(7, 0) = new_attitude[1]; current_state(8, 0) = new_attitude[2];
    current_state(9, 0) = angular_velocity[0]; current_state(10, 0) = angular_velocity[1]; current_state(11, 0) = angular_velocity[2];

    Vector previous_state(12, 0.0);
    previous_state(0, 0) = prevState[0][0]; previous_state(1, 0) = prevState[0][1]; previous_state(2, 0) = prevState[0][2];
    previous_state(3, 0) = prevState[1][0]; previous_state(4, 0) = prevState[1][1]; previous_state(5, 0) = prevState[1][2];
    previous_state(6, 0) = prevState[2][0]; previous_state(7, 0) = prevState[2][1]; previous_state(8, 0) = prevState[2][2];
    previous_state(9, 0) = prevState[3][0]; previous_state(10, 0) = prevState[3][1]; previous_state(11, 0) = prevState[3][2];



    // 5. Construct input Vector
    Vector current_input = toVector(command);

    // 6. Get MoIMs
    Matrix inertia = toMatrix(INERTIA);
    Matrix inertia_a = toMatrix(getInertiaA(command[1]));
    Matrix inertia_b = toMatrix(getInertiaB(command[2]));
    
    // 7. Calculate A and B Matrices << NOTE AERO HAS BEEN ZEROED
    Matrix A = calculateA(m, f, area, Cd, toVector(desired_state), static_input, rc, rt, inertia, inertia_a, inertia_b, toVector({0,0,0,0}));
    Matrix B = calculateB(m, f, area, Cd, toVector(desired_state), static_input, rc, rt, inertia, inertia_a, inertia_b, toVector({0,0,0,0}));

    // 8. Sanitize NaNs that pop up from numerical errors close to zero.
    A.sanitizeNaNs();
    B.sanitizeNaNs();


    // std::cout << "" << std::endl;
    // A.print();
    // std::cout << "" << std::endl;
    // B.print();
    // std::cout << "" << std::endl;

    // ---------- ii. Use state matrices to compute optimal controls -----------

    // 1. Determine u_d
    Vector xd = toVector(desired_state);
    Vector xdotd = toVector(delta_desired_state);
    Vector g_bias = toVector(G_VECTOR).multiply(-1);
    Vector Axd = A.multiply(xd);
    Vector negAxd = Axd.multiply(-1);
    Vector r = (xdotd.add(negAxd)).add(g_bias);
    double rankEps = 1e-12;
    int maxIter = 100;
    Matrix B_pinv = B.pseudoInverseJacobi(rankEps, maxIter);
    Vector u_d = B_pinv.multiply(r); 
    std::vector<double> desired_command = {u_d[0], u_d[1], u_d[2]};

    static_input = u_d;

    // std::cout << "xact:\n"; current_state.print();
    // std::cout << "uact:\n"; current_input.print();

    std::vector<double> K_MATRIX = {
        0,0,4,         0,0,1,           0,0,0,          0,0,0,
        0,0,0,         0,0,0,           1,0,0,          0,0,0,
        0,0,0,         0,0,0,           0,1,0,          0,0,0,
    };

    std::vector<double> I_MATRIX = {
        0,0,0.03,  0,0,0.03,      0,0,0,  0,0,0,
        0,0,0,  0,0,0,      0,0,0,  0,0,0,
        0,0,0,  0,0,0,      0,0,0,  0,0,0,
    };

    std::vector<double> D_MATRIX = {
        0,0,1,  0,0,1,     0,0,0,  0,0,0, 
        0,0,0,  0,0,0,     0,0,0,  0.025,0,0,
        0,0,0,  0,0,0,     0,0,0,  0,0.025,0,
    };
    
    Matrix K = toRectMatrix(K_MATRIX, 3, 12);
    Matrix I = toRectMatrix(I_MATRIX, 3, 12);
    Matrix D = toRectMatrix(D_MATRIX, 3, 12);

    double roll  = current_state(6, 0);
    double pitch = current_state(7, 0);
    double yaw   = current_state(8, 0);

    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    // Body-to-world rotation matrix (ZYX order)
    Matrix R(3, 3, 0.0);
    R(0, 0) = cp * cy;
    R(0, 1) = cy * sp * sr - cr * sy;
    R(0, 2) = sr * sy + cr * cy * sp;
    R(1, 0) = cp * sy;
    R(1, 1) = cr * cy + sp * sr * sy;
    R(1, 2) = cr * sp * sy - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cr * cp;

    Matrix T(12, 12, 0.0);

    // Set identity blocks for position and orientation
    for (int i = 0; i < 3; ++i) {
        T(i, i) = 1.0;        // Position block
        T(6 + i, 6 + i) = 1.0; // Euler angles block
    }

    // Copy R into linear velocity block (rows 3–5, cols 3–5)
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            T(3 + i, 3 + j) = R(i, j);

    // Copy R into angular velocity block (rows 9–11, cols 9–11)
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            T(9 + i, 9 + j) = R(i, j);

   
    D = D.multiply(T.pseudoInverseJacobi(rankEps, 100)); I = I.multiply(T.pseudoInverseJacobi(rankEps, 100)); K = K.multiply(T.pseudoInverseJacobi(rankEps, 100));

    // Compute control command: u = -K * state_error
    Vector state_error = current_state.subtract(toVector(desired_state));

    Vector DE = (current_state.subtract(previous_state)).multiply(1/dt); // Assuming u_d rel. unchanged

    IE = IE.add(state_error.multiply(dt));


    // Compute control commands
    Vector control_command = u_d.subtract(((K.multiply(state_error)).add(I.multiply(IE))).add(D.multiply(DE)));  // result is 3x1 Matrix


    //  // Read in Q, R matrices
    //  Matrix Q = toRectMatrix(Q_MATRIX, 12, 12);
    //  Matrix R = toRectMatrix(R_MATRIX, 3, 3);

    //  std::cout << "\nAfn: " << frobeniusNorm(A);
    //  std::cout << "\nBfn: " << frobeniusNorm(B);

 
    //  // Set state and setpoint in LQR controller
    //  lqrController.setA(A);
    //  lqrController.setB(B);
    //  lqrController.setQ(Q);
    //  lqrController.setR(R);
 
    //  // Read in current + desired states
    //  lqrController.setState(current_state);
    //  lqrController.setPoint = toVector(desired_state); // Assuming setPoint is a std::vector
 
    //  // Compute error between current state and desired state
    //  Matrix neg_setpoint = lqrController.setPoint.multiply(-1.0);
    //  Vector state_error = lqrController.getState().add(Vector(neg_setpoint));
 
    //  // Recalculate K if needed (e.g., time-varying system)
    //  if ( iter % 5 == 0) {
    //     lqrController.calculateK(dt);
    //  }
 
    //  // // // // Compute control command: u = -K * state_error

    // Matrix negative_K = lqrController.getK().multiply(-1.0);
    // Vector control_command = u_d.add(negative_K.multiply(state_error)); 
    Vector filtered_command = control_command;
    // Vector change_command = (negative_K.multiply(state_error));

    // sanity checks on T for feedback command
    if (control_command[0] > 15) control_command[0] = 15;
    if (control_command[0] < 0) control_command[0] = 0;

    // sanity checks on filter for output command
    if (filtered_command[0] > 15) filtered_command[0] = 15; if (filtered_command[0] < 0) filtered_command[0] = 0;
    if (filtered_command[1] > 1) filtered_command[1] = 1; if (filtered_command[1] < -1) filtered_command[1] = -1;
    if (filtered_command[2] > 1) filtered_command[2] = 1; if (filtered_command[2] < -1) filtered_command[2] = -1;
    
    std::vector<double> filteredCommand = toStdVector(filtered_command);
    std::vector<double> newCommand = toStdVector(control_command);  
    std::vector<double> error = toStdVector(state_error);
    std::vector<bool> newStatus = {true, true};


    // std::cout << "U:\n"; filtered_command.print();

    // Return pertinent values to data_mocker
    LoopOutput out = {
        newState, 
        newStatus, 
        newCommand, 
        error, 
        desired_command, 
        filteredCommand
    };

    return out;
}