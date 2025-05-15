#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <random>
#include "../CustomLinear/Matrix.h"
#include "../CustomLinear/Vector.h"
#include "../CustomLinear/Quaternion.h"
#include "../Loop/loop.h"
#include "../Loop/init.h"
#include "../LQR/lqr.h"
#include "../LQR/calculateA.h"
#include "../LQR/calculateB.h"
#include "../Filters/EKFs/ExtendedKalmanFilterGeneral.h"
#include "../Filters/EKFs/EKF_xy.h"
#include "../Filters/EKFs/EKF_z.h"
#include "../Filters/Madgwick/Madgwick.h"
#include "../LQR/solveCARE.h"
#include <random>



using namespace std;

//------------------------- Basic Vector Arithmetic Helpers ---------------------------

// may get rid of them later (redundant with vector/matrix class)
// made by Justin E.

Vector vectorAdd(const Vector &a, const Vector &b) {
    return a.add(b);
}

Vector vectorSubtract(const Vector &a, const Vector &b) {
    return a.add(b.multiply(-1.0));
}

Vector vectorScale(const Vector &a, double scalar) {
    return a.multiply(scalar);
}

void printVector(const std::vector<double>& vec, const std::string& label) {
    if (!label.empty()) {
        std::cout << label << ": ";
    }

    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

double stdMagnitude(std::vector<double>& vector) {
    // Calculate the magnitude of a std::vector which is different from Vector
    double sum = 0.0;
    for (unsigned int i = 0; i < vector.size(); ++i) {
        sum += (vector[i] * vector[i]);
    }
    return std::sqrt(sum);
}

std::vector<double> quaternionToEuler(Quaternion quaternion) {

    return {
        std::atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 
                   1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])),
        std::asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1])),
        std::atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 
                   1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]))
    };
}

Quaternion eulerToQuaternion(const std::vector<double>& euler) {
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;

    return {w, x, y, z};
}

// additional helper function (you can get rid of this if these implementations become redundant)
double stdMagnitude(std::vector<double>& vector) {
    // Calculate the magnitude of a std::vector which is different from Vector
    double sum = 0.0;
    for (unsigned int i = 0; i < vector.size(); ++i) {
        sum += (vector[i] * vector[i]);
    }
    return std::sqrt(sum);
}

double stdVectorNorm(std::vector<double>& vector) {
    // Calculate the norm of a std::vector which is different from Vector
    double sum = 0.0;
    for (unsigned int i = 0; i < vector.size(); ++i) {
        sum += (vector[i] * vector[i]);
    }
    return sum;
}

// Sensor Noise Helper Function 
std::vector<double> addSensorNoise(const std::vector<double>& true_sensor_value, const double& dt, const double& sigma_white, 
    const double& sigma_drift, std::vector<double>& bias) {
    
    std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> N01{0.0,1.0};

    double s = sigma_drift * std::sqrt(dt);
    
    std::vector<double> noisy_sensor_value(true_sensor_value.size(), 0.0);
    for (size_t i = 0; i < true_sensor_value.size(); i++) {
        bias[i] += s * N01(rng); // bias random walk
        double noise = sigma_white * N01(rng); // white noise
        // noisy_sensor_value.push_back(specific_sensor_value[i] + noise + bias[i]);
        noisy_sensor_value[i] = true_sensor_value[i] + noise + bias[i];
    }

    return noisy_sensor_value;
}

// Meters to Degrees for GPS Conversion

void preciseMetersToLatLon(double  lat,
    double  dY,
    double  dX,
    double &deltaLat,
    double &deltaLon)
{
    constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
    double latRad = lat * DEG2RAD;

    /* metres per degree — same formula as in loop.cpp (WGS84) */
    double metersPerDegLat = 111132.92
        -   559.82 * std::cos(2 * latRad)
        +     1.175 * std::cos(4 * latRad)
        -     0.0023* std::cos(6 * latRad);

    double metersPerDegLon = 111412.84 * std::cos(latRad)
        -     93.5  * std::cos(3 * latRad)
        +      0.118* std::cos(5 * latRad);

    /* inverse conversion */
    deltaLat = dY / metersPerDegLat;
    deltaLon = dX / metersPerDegLon;
}

// --------------------------- Constants ------------------------------------


//------------------------- State Structure ---------------------------------

struct State {
    Vector pos;         // 3x1 position vector
    Vector vel;         // 3x1 velocity vector
    Vector accel;       // 3x1 acceleration vector
    Quaternion att;     // attitude as a quaternion [w, x, y, z]
    Vector ang_vel;     // 3x1 angular velocity vector
    Vector ang_accel;   // 3x1 angular acceleration vector

    State()
      : pos(3, 0.0),
        vel(3, 0.0),
        accel(3, 0.0),
        // Initialize attitude as identity quaternion: (1, 0, 0, 0)
        att(1.0, 0.0, 0.0, 0.0),
        ang_vel(3, 0.0),
        ang_accel(3, 0.0)
    { }
};

//------------------------- Rocket Parameters Structure ---------------------------

struct RocketParams {
    // Mass and inertia.
    double m_static;
    double m_gimbal_top;
    double m_gimbal_bottom;
    double m;
    
    // Offsets as 3x1 vectors.
    Vector gimbal_top_COM_offset;
    Vector gimbal_bottom_COM_offset;
    Vector COM_offset;
    Vector COP;
    Vector COP_offset;
    Vector gimbal_offset;
    Vector gimbal_x_distance;
    Vector gimbal_y_distance;
    
    // Physical constants.
    double g;         // gravitational acceleration
    double Cd_x, Cd_y, Cd_z;
    double A_x, A_y, A_z;
    double air_density;
    
    // Inertia matrix and its inverse.
    Matrix I;
    Matrix Inv_I;
    
    // Gimbal inertias.
    double gimbal_top_I;
    double gimbal_bottom_I;
    
    // Thrust limits and gimbal limits.
    double T_max, T_min;
    double gimbal_speed;
    double gimbal_acceleration;
    
    double dt;      // simulation timestep.

    RocketParams() 
      : m_static(0.0), m_gimbal_top(0.0), m_gimbal_bottom(0.0), m(0.0),
        // Initialize 3x1 vectors (for example, setting all entries to zero)
        gimbal_top_COM_offset(3, 0.0),
        gimbal_bottom_COM_offset(3, 0.0),
        COM_offset(3, 0.0),
        COP(3, 0.0),
        COP_offset(3, 0.0),
        gimbal_offset(3, 0.0),
        gimbal_x_distance(3, 0.0),
        gimbal_y_distance(3, 0.0),
        // Physical constants.
        g(9.81),
        Cd_x(0.1), Cd_y(0.1), Cd_z(0.1),
        A_x(0.7), A_y(0.7), A_z(0.3),
        air_density(1.225),
        // Initialize a 3x3 Matrix with zeros.
        I(3, 3, 0.0),
        Inv_I(3, 3, 0.0),
        // Gimbal inertias.
        gimbal_top_I(1.0),
        gimbal_bottom_I(1.0),
        // Thrust and gimbal limits.
        T_max(100.0), T_min(0.0),
        gimbal_speed(5.0),
        gimbal_acceleration(2.0),
        dt(0.001)
    {    }
};

//------------------------- Quaternion-Based Dynamics Update ---------------------------

/*
   Update the state (position, velocity, attitude, angular velocity) using Euler integration.
   For the rotational update we use:
       q_dot = 0.5 * (att * ω_quat)
   where ω_quat is [0, ω], and then:
       att_new = att + q_dot * dt, then normalized.
*/
State update_dynamics(const RocketParams &P,
                      const Vector &pos,
                      const Vector &vel,
                      const Quaternion &att,
                      const Vector &ang_vel,
                      const Vector &F_net,
                      const Vector &T_net,
                      double delta_t)
{
    State newState;
    // std::cout << "Translational update\n";
    // --- Translational update ---
    Vector accel = F_net.multiply(1.0 / P.m);
    Vector vel_new = vectorAdd(vel, vectorScale(accel, delta_t));
    Vector pos_new = vectorAdd(pos, vectorScale(vel_new, delta_t));
    // std::cout << "Rotational update\n";

    // --- Rotational update ---
    // Angular acceleration: ang_accel = Inv_I * T_net.
    Vector ang_accel = P.Inv_I.multiply(T_net);
    Vector ang_vel_new = vectorAdd(ang_vel, vectorScale(ang_accel, delta_t));
    
    // Create an "omega quaternion" from the new angular velocity.
    // Standard formulation: ω_quat = [0, ωx, ωy, ωz]
    // std::cout << "Quaternion update\n";

    Quaternion omega(0, ang_vel_new(0,0), ang_vel_new(1,0), ang_vel_new(2,0));
    // Compute quaternion derivative: q_dot = 0.5 * (att * ω_quat)
    Quaternion q_dot = att * omega * 0.5;
    // Euler integrate: att_new = att + q_dot * dt, then normalize.
    Quaternion att_new = att + q_dot * delta_t;
    att_new = att_new.normalize();
    // std::cout << "Assign new values\n";

    newState.pos = pos_new;
    newState.vel = vel_new;
    newState.accel = accel;
    newState.att = att_new;
    newState.ang_vel = ang_vel_new;
    newState.ang_accel = ang_accel;
    // std::cout << "Return new values\n";

    return newState;
    // std::cout << "Returned\n";
}

//------------------------- Extrinsic Rotation Functions ---------------------------

// These use our custom Matrix and Vector classes. Here we assume that get_extrinsic_x_rotation,
// get_extrinsic_y_rotation, and get_extrinsic_z_rotation are defined below (or reuse the ones from earlier).

Matrix get_extrinsic_x_rotation(double x) {
    Matrix R(3,3,0.0);
    R(0,0) = 1;       R(0,1) = 0;         R(0,2) = 0;
    R(1,0) = 0;       R(1,1) = cos(x);    R(1,2) = -sin(x);
    R(2,0) = 0;       R(2,1) = sin(x);    R(2,2) = cos(x);
    return R;
}

Matrix get_extrinsic_y_rotation(double y) {
    Matrix R(3,3,0.0);
    R(0,0) = cos(y);  R(0,1) = 0;         R(0,2) = sin(y);
    R(1,0) = 0;       R(1,1) = 1;         R(1,2) = 0;
    R(2,0) = -sin(y); R(2,1) = 0;         R(2,2) = cos(y);
    return R;
}

Matrix get_extrinsic_z_rotation(double z) {
    Matrix R(3,3,0.0);
    R(0,0) = cos(z);  R(0,1) = -sin(z);   R(0,2) = 0;
    R(1,0) = sin(z);  R(1,1) = cos(z);    R(1,2) = 0;
    R(2,0) = 0;       R(2,1) = 0;         R(2,2) = 1;
    return R;
}

// Given Euler angles [x, y, z] in a Vector (3x1), return extrinsic rotation matrix.
Matrix get_extrinsic_rotation_matrix(const Vector &euler) {
    double x = euler(0,0), y = euler(1,0), z = euler(2,0);
    // Note: Apply rotations in z-y-x order.
    Matrix R = get_extrinsic_z_rotation(z).multiply(get_extrinsic_y_rotation(y)).multiply(get_extrinsic_x_rotation(x));
    return R;
}

//------------------------- Thrust and Drag Models ---------------------------

/*
   get_thrust_body:
   Computes the thrust force and torque in the body frame.
   For simplicity the gimbal (thrust steering) is held zero.
*/
pair<Vector, Vector> get_thrust_body(const RocketParams &P, double F_thrust_mag, const Vector &thrust_gimbal) {
    // Compute rotation matrix from thrust_gimbal; here thrust_gimbal is a 3x1 vector.
    Matrix R = get_extrinsic_rotation_matrix(thrust_gimbal);
    // Unit vector in z direction.
    Vector ez(3, 0.0);
    ez(2,0) = 1.0;
    
    // thrust_force = F_thrust_mag * (R * ez)
    Vector thrust_force = R.multiply(ez).multiply(F_thrust_mag);
    
    // For torque: r = COM_offset + gimbal_offset + (get_extrinsic_x_rotation(thrust_gimbal(0)) * gimbal_x_distance) +
    // (R * gimbal_y_distance)
    Vector r = P.COM_offset;
    r = vectorAdd(r, P.gimbal_offset);
    // (Assuming the offsets are all zero in our simple model, r remains zero.)
    Vector thrust_torque = r.crossProduct(thrust_force);
    
    return make_pair(thrust_force, thrust_torque);
}

/*
   get_drag_body:
   Computes drag force and drag-induced torque.
   Uses the transformation of velocity into the body frame via the attitude quaternion.
*/
pair<Vector, Vector> get_drag_body(const RocketParams &P, const Quaternion &att, const Vector &vel, const Vector &v_wind) {
    Vector vel_rel = vectorSubtract(vel, v_wind); // relative velocity in world frame
    // Convert vel_rel into body frame: use att.inverse().toRotationMatrix()
    Quaternion att_inv = att.inverse();
    Matrix R_body = att_inv.toRotationMatrix();
    Vector vel_rel_body = R_body.multiply(vel_rel);
    
    // vel_rel_body is a 0 vector right now. 
    double v_rel_norm = vel_rel_body.magnitude();
    // Build a diagonal vector for drag coefficients.
    // is this really a diagonal "vector"??? - Justin
    Vector diagVals(3, 0.0);
    // std::cout << "Cd_x: " << P.Cd_x << ", Cd_y: " << P.Cd_y << "\n";
    diagVals(0,0) = P.Cd_x * P.A_x;
    diagVals(1,0) = P.Cd_y * P.A_y;
    diagVals(2,0) = P.Cd_z * P.A_z;
    // cout << "get drag body func check 2\n";
    double factor = 0.5 * P.air_density * v_rel_norm;
    Vector drag_force = vel_rel_body; 
    // Elementwise multiply by diagVals.
    for (unsigned int i = 0; i < 3; i++) {
        double temp = drag_force(i,0) * diagVals(i,0);
        drag_force(i,0) = -factor * temp;
    }
    // cout << "get drag body func check 3\n";
    // Drag torque: COP_offset x drag_force.
    Vector drag_torque = P.COP_offset.crossProduct(drag_force);
    return make_pair(drag_force, drag_torque);
}

/*
   Placeholder for calculating COM and COP offsets.
   In this simplified model (with all offsets zero), nothing is done.
*/
void calculate_COM_and_COP_offset(RocketParams &P, const Vector &thrust_gimbal) {
    // In a detailed model, you would update P.COM_offset and P.COP_offset here.
    // We leave them unchanged.
}

//------------------------- Simulation Function ---------------------------

void simulate(RocketParams &P) {

    LoopOutput loopOutput;

    double t_end = 40.0;         // total simulation time in seconds
    double dt = P.dt;
    int num_steps = static_cast<int>(round(t_end / dt)) + 1;

    // Anchor positions -- This is ORDERED
    Vector anchor1(3, 0.0); anchor1(0,0) = 10.0;  anchor1(1,0) = 10.0;  anchor1(2,0) = 0.0;
    Vector anchor2(3, 0.0); anchor2(0,0) = -10.0; anchor2(1,0) = 10.0;  anchor2(2,0) = 0.0;
    Vector anchor3(3, 0.0); anchor3(0,0) = 0.0;   anchor3(1,0) = -14.0; anchor3(2,0) = 0.0;

    
    // Build time vector
    vector<double> time(num_steps);
    for (int i = 0; i < num_steps; i++) {
        time[i] = i * dt;
    }
    
    // Initial conditions
    Vector pos(3, 0.0);            // 3x1 zero vector for position
    std::vector<double> gpsInit = {pos(0,0), pos(1,0), pos(2,0)};
    // Initialize position as a 3x1 vector, expected by init function defining SystemComponents
    Vector vel(3, 0.0);            // 3x1 zero velocity
    std::vector<double> vel_std = {vel(0,0), vel(1,0), vel(2,0)};

    std::vector<double> init_att = {0, 0, 0};

    Quaternion att = eulerToQuaternion(init_att); // Initial attitude: identity quaternion
    std::vector<double> att_euler_std = quaternionToEuler(att);
    Vector ang_vel(3, 0.0);        // angular velocity zero
    ang_vel(1,0) = 0;
    std::vector<double> ang_vel_std = {ang_vel(0,0), ang_vel(1,0), ang_vel(2,0)};

    
    // Initialize control variables
    Vector thrust_gimbal(3, 0.0);
    double F_thrust_mag = 0.0;
    
    // Initialize LQR components
    std::vector<std::vector<double>> initState = {{gpsInit}, {vel_std}, {init_att}, {ang_vel_std}};
    SystemComponents system = init(gpsInit, initState, dt); 
    unsigned int iter = 0;
    std::vector<bool> status = {true, true};
    
    // Pre-allocate history
    vector<Vector> pos_history;
    vector<Vector> vel_history;
    vector<Vector> command_history;
    vector<Vector> attitude_history;
    vector<Vector> pos_v_history;
    vector<Vector> vel_v_history;
    vector<double> aoa_history;
    pos_history.reserve(num_steps);
    vel_history.reserve(num_steps);
    command_history.reserve(num_steps);
    attitude_history.reserve(num_steps);
    pos_v_history.reserve(num_steps);
    vel_v_history.reserve(num_steps);
    aoa_history.reserve(num_steps);
    
    // Build desired trajectory: pure vertical motion
    vector<Vector> delta_pos_desired;
    delta_pos_desired.reserve(num_steps);

    for (int i = 0; i < num_steps; i++) {
        double t_val = time[i];
        double dz_dt = 0.0;

        if (t_val >= 0.0 && t_val < 10.0) {
            dz_dt = 10.0 * 0.5 * (M_PI / 10.0) * sin(M_PI * t_val / 10.0);
        } else if (t_val >= 10.0 && t_val < 20.0) {
            dz_dt = 0.0;
        } else if (t_val >= 20.0 && t_val < 30.0) {
            dz_dt = -10 * 0.5 * (M_PI / 10.0) * sin(M_PI * (t_val - 20.0) / 10.0);
        } else if (t_val >= 30.0 && t_val <= 40.0) {
            dz_dt = 0.0;
        }

        Vector vel_des(3, 0.0);
        vel_des(2,0) = dz_dt;
        delta_pos_desired.push_back(vel_des);
    }

    vector<Vector> accel_desired;
    accel_desired.reserve(num_steps);

    for (int i = 0; i < num_steps; i++) {
        double t_val = time[i];
        double d2z_dt2 = 0.0;
        double pi_over_10 = M_PI / 10.0;
        double factor = 10.0 * 0.5 * pi_over_10 * pi_over_10;

        if (t_val >= 0.0 && t_val < 10.0) {
            d2z_dt2 = factor * cos(pi_over_10 * t_val);
        } else if (t_val >= 10.0 && t_val < 20.0) {
            d2z_dt2 = 0.0;
        } else if (t_val >= 20.0 && t_val < 30.0) {
            d2z_dt2 = -factor * cos(pi_over_10 * (t_val - 20.0));
        } else if (t_val >= 30.0 && t_val <= 40.0) {
            d2z_dt2 = 0.0;
        }

        Vector acc_des(3, 0.0);
        acc_des(2,0) = d2z_dt2;
        accel_desired.push_back(acc_des);
    }

    vector<Vector> pos_desired;
    pos_desired.reserve(num_steps);

    // Initial position z = 0
    double z = 0.0;
    pos_desired.push_back(Vector(3, 0.0));

    for (int i = 1; i < num_steps; i++) {
        double dt = time[i] - time[i - 1];
        double v_prev = delta_pos_desired[i - 1](2,0);
        double v_curr = delta_pos_desired[i](2,0);

        // Trapezoidal integration
        z += 0.5 * (v_prev + v_curr) * dt;

        Vector pos(3, 0.0);
        pos(2,0) = z;
        pos_desired.push_back(pos);
    }


    

    // Zero wind
    Vector v_wind(3, 0.0);

    // Set up command and state feedback loops
    loopOutput.state = {gpsInit, vel_std, att_euler_std, ang_vel_std};
    std::vector<std::vector<double>> prevState = loopOutput.state;
    std::vector<std::vector<double>> previous_state;

    loopOutput.command = {0,0,0};
    std::vector<double> prevCommand = loopOutput.command;
    std::vector<double> previous_command = prevCommand;
    std::vector<double> previous_previous_command;

    Matrix Kin = Matrix(3,12,0.0);

    Vector accel = Vector(3, 0.0);
    accel(2,0) = -9.81;

    
    bool grounded = false;

    // Main simulation loop
    for (int step = 0; step < num_steps; step++) {

        // -------------------- LQR CONTROL CALCULATION --------------------
        // std::cout << "Step: " << step << "\n";
        // 1. Convert current state to the format expected by the LQR
        std::vector<double> position = {pos(0,0), pos(1,0), pos(2,0)};
        std::vector<double> velocity = {vel(0,0), vel(1,0), vel(2,0)};
        
        // Convert quaternion to Euler angles for the LQR
        Matrix euler_attitude_mat = att.toEulerMatrix();
        // convert euler matrix to std::vector
        std::vector<double> euler_attitude = {euler_attitude_mat(0,0), 
                                              euler_attitude_mat(1,0), 
                                              euler_attitude_mat(2,0)};
        
        std::vector<double> angular_velocity = {ang_vel(0,0), ang_vel(1,0), ang_vel(2,0)};
        
        // Current state in the format expected by LQR
        std::vector<std::vector<double>> current_state = {position, velocity, euler_attitude, angular_velocity};
       
        // 2. Calculate desired state from trajectory
        std::vector<double> desired_position = {
            pos_desired[step](0,0), 
            pos_desired[step](1,0), 
            pos_desired[step](2,0)
        };

        std::vector<double> delta_desired_position = {
            delta_pos_desired[step](0,0),
            delta_pos_desired[step](1,0),
            delta_pos_desired[step](2,0)
        };

        std::vector<double> acc_desired_position = {
            accel_desired[step](0,0),
            accel_desired[step](1,0),
            accel_desired[step](2,0)
        };
        
        // Calculate velocity setpoint based on trajectory (simple difference if needed)
        std::vector<double> desired_velocity = {0, 0, 0};
        if (step < num_steps - 1) {
            desired_velocity[0] = (pos_desired[step+1](0,0) - pos_desired[step](0,0)) / dt;
            desired_velocity[1] = (pos_desired[step+1](1,0) - pos_desired[step](1,0)) / dt;
            desired_velocity[2] = (pos_desired[step+1](2,0) - pos_desired[step](2,0)) / dt;
        }
        
        // Full desired state vector
        std::vector<double> desired_state = {
            desired_position[0], desired_position[1], desired_position[2],
            delta_desired_position[0], delta_desired_position[1], delta_desired_position[2],
            0, 0, 0,  // desired attitude (usually zero for hover)
            0, 0, 0   // desired angular velocity (usually zero)
        };
        
        // Delta desired state for LQR
        std::vector<double> delta_desired_state = {
            delta_desired_position[0], delta_desired_position[1], delta_desired_position[2],
            acc_desired_position[0], acc_desired_position[1], acc_desired_position[2],
            0, 0, 0,  // desired angular velocity (usually zero for hover)
            0, 0, 0   // desired angular acceleration (usually zero)
        };

<<<<<<< HEAD
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.00, 0.00);

        Vector accel = Vector(3,0); accel[0]=0; accel[1]=0; accel[2]=-9.81;
        Matrix WF2BF = att.toRotationMatrix();
        Vector accel_bf = WF2BF.multiply(accel);



        std::vector<double> noise = {dis(gen), dis(gen), dis(gen)};
=======
        // Convert sim position (m) to GPS coordinates (lat, lon).
        double lat, lon;
        double ref_lat = 33; // Reference latitude for conversion (degrees). Approx location of Atlanta.
        double dY = pos(1,0);
        double dX = pos(0,0);
        preciseMetersToLatLon(ref_lat, dY, dX, lat, lon);
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22

        // debug statement for accel
        if (step % 4000 == 0) {
            std::cout << "Accel: " << accel(0,0) << ", " << accel(1,0) << ", " << accel(2,0) << "\n";
        }

        Matrix R_bw = att.toRotationMatrix();
        Matrix WF2BF = R_bw.transpose();

        Vector mag_wf = Vector(3,0); mag_wf[0] = 0.005; mag_wf[1] = 0; mag_wf[2] = 0;
        Vector mag_bf = WF2BF.multiply(mag_wf);
        Vector accel_bf = WF2BF.multiply(accel);


        // 3. Set up the input structure for the LQR loop function. Noiseless data
        std::vector<std::vector<double>> sensor_values = {
<<<<<<< HEAD
            {0, ang_vel(0,0) + noise[0], ang_vel(1,0) + noise[1], ang_vel(2,0) + noise[2], accel_bf[0], accel_bf[1], accel_bf[2], 0, 0, 0}, // Mock IMU data
            {0, ang_vel(0,0) + noise[0], ang_vel(1,0) + noise[1], ang_vel(2,0) + noise[2], accel_bf[0], accel_bf[1], accel_bf[2],}, // Mock 6-axis IMU
            {0, 0, pos(2,0)}, // Mock GPS
=======
            {0, ang_vel(0,0), ang_vel(1,0), ang_vel(2,0), accel_bf(0,0), accel_bf(1,0), accel_bf(2,0), mag_bf(0,0), mag_bf(1,0), mag_bf(2,0)}, // Mock 9-axis IMU 
            {0, ang_vel(0,0), ang_vel(1,0), ang_vel(2,0), accel_bf(0,0), accel_bf(1,0), accel_bf(2,0)}, // Mock 6-axis IMU
            {lat, lon, pos(2,0)}, // Mock GPS
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
            {0, pos(2,0)}, // Mock LIDAR
            {0,
                (pos.subtract(anchor1)).magnitude(),
                (pos.subtract(anchor2)).magnitude(),
                (pos.subtract(anchor3)).magnitude()
            } // mock UWB
        };

        // std::vector<std::vector<double>> noisy_sensor_values = sensor_values;

        // 3.1 Add white gaussian noise + random-walk-bias (drift) to sensor values
        // note: could use a loop to make this look nicer.

        // 9-Axis Noisy
        std::vector<double> imuNineAxis(sensor_values[0].end()-9, sensor_values[0].end());
        std::vector<double> nineAxisBias(imuNineAxis.size(), 0.01);
        // initializes to some small variable. Assumes all components have the same INITIAL bias
        std::vector<double> imuNineAxisNoise = addSensorNoise(imuNineAxis, P.dt, 0.01, 0.01, nineAxisBias);
        std::vector<double> imuNineAxisNoisy(1, 0);
        imuNineAxisNoisy.insert(imuNineAxisNoisy.end(), imuNineAxisNoise.begin(), imuNineAxisNoise.end());

        // 6-Axis Noisy
        std::vector<double> imuSixAxis(sensor_values[1].end()-6, sensor_values[1].end());
        std::vector<double> sixAxisBias(imuSixAxis.size(), 0.01);
        // initializes to some small variable. Assumes all components have the same INITIAL bias
        std::vector<double> imuSixAxisNoise = addSensorNoise(imuSixAxis, P.dt, 0.01, 0.01, sixAxisBias);
        std::vector<double> imuSixAxisNoisy(1, 0);
        imuSixAxisNoisy.insert(imuSixAxisNoisy.end(), imuSixAxisNoise.begin(), imuSixAxisNoise.end());
        
        // GPS Noisy
        std::vector<double> gps(sensor_values[2].end()-2, sensor_values[2].end());
        std::vector<double> gpsBias(gps.size(), 0.1);
        // initializes to some small variable. Assumes all components have the same INITIAL bias
        std::vector<double> gpsNoise = addSensorNoise(gps, P.dt, 0.1, 0.1, gpsBias);
        std::vector<double> gpsNoisy(1, 0);
        gpsNoisy.insert(gpsNoisy.end(), gpsNoise.begin(), gpsNoise.end());
        
        // LIDAR Noisy
        std::vector<double> lidar(sensor_values[3].end()-1, sensor_values[3].end());
        std::vector<double> lidarBias(lidar.size(), 0.1);
        // initializes to some small variable. Assumes all components have the same INITIAL bias
        std::vector<double> lidarNoise = addSensorNoise(lidar, P.dt, 0.1, 0.1, lidarBias);
        std::vector<double> lidarNoisy(1, 0);
        lidarNoisy.insert(lidarNoisy.end(), lidarNoise.begin(), lidarNoise.end());
        
        // UWB Noisy
        std::vector<double> uwb(sensor_values[4].end()-3, sensor_values[4].end());
        std::vector<double> uwbBias(uwb.size(), 0.1);
        // initializes to some small variable. Assumes all components have the same INITIAL bias
        std::vector<double> uwbNoise = addSensorNoise(uwb, P.dt, 0.1, 0.1, uwbBias);
        std::vector<double> uwbNoisy(1, 0);
        uwbNoisy.insert(uwbNoisy.end(), uwbNoise.begin(), uwbNoise.end());

        // combine all noisy components into noisy sensor values vector
        std::vector<std::vector<double>> noisy_sensor_values = {
            imuNineAxisNoisy,
            // sensor_values[0],
            imuSixAxisNoisy,
            // sensor_values[1],
            gpsNoisy,
            lidarNoisy,
            uwbNoisy
        };
        // if (step % 4000 == 0) {
        //     std::cout << "Noisy M9IMU Data: "; printVector(noisy_sensor_values[0], "");
        //     std::cout << "Noisy M6IMU Data: "; printVector(noisy_sensor_values[1], "");
        //     std::cout << "Noisy MGPS Data: "; printVector(noisy_sensor_values[2], "");
        //     std::cout << "Noisy MLIDAR Data: "; printVector(noisy_sensor_values[3], "");
        //     std::cout << "Noisy MUWB Data: "; printVector(noisy_sensor_values[4], "");
        // }

        // std::cout << "IMU Data: "; printVector(sensor_values[0], "");
        // std::cout << "M6IMU Data: "; printVector(sensor_values[1], "");
        // std::cout << "MGPS Data: "; printVector(sensor_values[2], "");
        // std::cout << "MLIDAR Data: "; printVector(sensor_values[3], "");
        // std::cout << "MUWB Data: "; printVector(sensor_values[4], "");

        // Set prevCommands
        previous_state = prevState;
        prevState = loopOutput.state; 
        previous_previous_command = previous_command;
        previous_command = prevCommand;
        prevCommand = loopOutput.filteredCommand; 

        // std::cout << "\nPrevK: "; prevK.print();

        // Create loop input structure
        LoopInput loopInput = {
            noisy_sensor_values,
            loopOutput.state,
            previous_state,
            system,
            status,
            dt,
            desired_state,
            delta_desired_state,
            loopOutput.command,
            previous_command,
            previous_previous_command,
        };
        
        // 4. Call the loop function to get control commands
        loopOutput = loop(loopInput);

        

        // 5. Extract commands from the loop output
        std::vector<double> command = loopOutput.filteredCommand;
<<<<<<< HEAD
       
=======

        // TESTING FOR WIND
        // if (step > 2000 && step < 2500) {
        //     command[1] = 0.01;
        //     command[2] = 0.01;
        // }
        // command[0] = 0;
        // command[1] = 0;
        // command[2] = 0;
        // std::vector<double> command = loopOutput.command;
        
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
        // 6. Convert commands to thrust and gimbal angles
        F_thrust_mag = command[0];  // Thrust magnitude
        thrust_gimbal(0,0) = command[1];  // X-axis gimbal angle
        thrust_gimbal(1,0) = command[2];  // Y-axis gimbal angle
        
        // Clamp thrust to limits
        if (F_thrust_mag < P.T_min) F_thrust_mag = P.T_min;
        if (F_thrust_mag > P.T_max) F_thrust_mag = P.T_max;
        
        // -------------------- DYNAMICS SIMULATION --------------------
        
        // Gravity force (world frame)
        Vector F_gravity(3, 0.0);
        F_gravity(2,0) = -P.m * P.g;

        double AoA = 0;
        double velocity_magnitude = stdMagnitude(velocity);
        std::vector<double> z_wf = {0, 0, 1}; 
        // double z_norm = stdMagnitude(z_wf);
        if (velocity_magnitude > 0) {
            // AOA calc from dot product
            if (velocity[2] < 0) {
                z_wf[2] = -1;
            }
            double dot = (velocity[0] * z_wf[0] + velocity[1] * z_wf[1] + velocity[2] * z_wf[2]);
            double cos_alpha = dot / (velocity_magnitude * 1); // z norm is always 1
            cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));

            AoA = std::acos(cos_alpha); // radians
            // AoA = std::acos(velocity_vector.dotProduct(z_direction) / (velocity_magnitude * z_direction.magnitude()))*(180/3.14159);
        } 
        
        P.Cd_x = -0.449*std::cos(3.028*AoA*180/M_PI) + 0.463;
        P.Cd_y = -0.449*std::cos(3.028*AoA*180/M_PI) + 0.463;
        P.Cd_z = -0.376*std::cos(5.675*AoA*180/M_PI) + 1.854;
        
        // Compute thrust force and torque (body frame)
        pair<Vector, Vector> thrust = get_thrust_body(P, F_thrust_mag, thrust_gimbal);
        Vector F_thrust_body = thrust.first;
        Vector T_thrust_body = thrust.second;
        
<<<<<<< HEAD
        // Compute drag force and torque
=======
        // Compute drag force and torque.
        // Compute Cd_x and Cd_y based on AoA. 

        double AoA = 0;
        double velocity_magnitude = stdMagnitude(velocity);
        std::vector<double> z_wf = {0, 0, 1}; 
        // double z_norm = stdMagnitude(z_wf);
        if (velocity_magnitude > 0) {
            // AOA calc from dot product
            if (velocity[2] < 0) {
                z_wf[2] = -1;
            }
            double dot = (velocity[0] * z_wf[0] + velocity[1] * z_wf[1] + velocity[2] * z_wf[2]);
            double cos_alpha = dot / (velocity_magnitude * 1); // z norm is always 1
            cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));

            AoA = std::acos(cos_alpha); // radians
            // AoA = std::acos(velocity_vector.dotProduct(z_direction) / (velocity_magnitude * z_direction.magnitude()))*(180/3.14159);
        } 
        // Update drag coefficients based on AoA and regression formulas. NOTE: ONLY ACCURATE FOR MONOPROP!

        P.Cd_x = -0.449*std::cos(3.028*AoA*180/M_PI) + 0.463;
        P.Cd_y = -0.449*std::cos(3.028*AoA*180/M_PI) + 0.463;
        P.Cd_z = -0.376*std::cos(5.675*AoA*180/M_PI) + 1.854;
        // debug statement
        // if (step % 4000 == 0) {
        //     std::cout << "AoA: " << AoA << "\n";
        //     std::cout << "Cd_x: " << P.Cd_x << ", Cd_y: " << P.Cd_y << ", Cd_z: " << P.Cd_z << "\n";
        // }

>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
        pair<Vector, Vector> drag = get_drag_body(P, att, vel, v_wind);
        Vector F_drag_body = drag.first;
        Vector T_drag_body = drag.second;
       
<<<<<<< HEAD
=======
        // Vector F_drag_body = Vector(3,0);
        // Vector T_drag_body = Vector(3,0);
        
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
        // Transform body-frame forces to world frame
        Matrix R_att = att.toRotationMatrix();
        Vector F_thrust_world = R_att.multiply(F_thrust_body);
        Vector F_drag_world = R_att.multiply(F_drag_body);
        Vector F_net = vectorAdd(F_gravity, vectorAdd(F_thrust_world, F_drag_world));
        // debug statement for thrust

        if (step % 4000 == 0) {
            // std::cout << "F_thrust_world: "; F_thrust_world.print();
            std::cout << step << endl;
            std::cout << "F_drag_body: "; F_drag_body.print();
            std::cout << "F_drag_world: "; F_drag_world.print();
            std::cout << "F_net: "; F_net.print();
            // std::cout << "F_thrust_world: "; F_thrust_world.print();
            // std::cout << "F_net: "; F_net.print();
        }
        
        // Total torque
        Vector T_net = R_att.multiply(T_thrust_body.add(T_drag_body));
        
        // Update dynamics
        State st = update_dynamics(P, pos, vel, att, ang_vel, F_net, T_net, dt);
        pos = st.pos;
        vel = st.vel;
        att = st.att;
        ang_vel = st.ang_vel;
        accel = st.accel;
        
        // Record history
        pos_history.push_back(pos);
        vel_history.push_back(vel);
        Vector cmd_vec = Vector(3, 0.0);
        cmd_vec(0, 0) = F_thrust_mag;
        cmd_vec(1, 0) = thrust_gimbal(0, 0);
        cmd_vec(2, 0) = thrust_gimbal(1, 0);
        command_history.push_back(cmd_vec);

<<<<<<< HEAD
=======
        // Vector pos_Vec = Vector(3, 0.0);
        // pos_Vec(0,0) = quaternionToEuler(att)[0];
        // pos_Vec(1,0) = quaternionToEuler(att)[1];
        // pos_Vec(2,0) = quaternionToEuler(att)[2];
        // pos_v_history.push_back(pos_Vec);
        
        
        // Vector vel_Vec = Vector(3, 0.0);
        // vel_Vec(0,0) = ang_vel(0,0);
        // vel_Vec(1,0) = ang_vel(1,0);
        // vel_Vec(2,0) = ang_vel(2,0);
        // vel_v_history.push_back(vel_Vec);

        Vector attitude_vec = Vector(3, 0.0);
        attitude_vec(0,0) = quaternionToEuler(att)[0];
        attitude_vec(1,0) = quaternionToEuler(att)[1];
        attitude_vec(2,0) = quaternionToEuler(att)[2];
        attitude_history.push_back(attitude_vec);

>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
        Vector pos_Vec = Vector(3, 0.0);
        pos_Vec(0,0) = quaternionToEuler(att)[0];
        pos_Vec(1,0) = quaternionToEuler(att)[1];
        pos_Vec(2,0) = quaternionToEuler(att)[2];
        pos_v_history.push_back(pos_Vec);
        
        
        Vector vel_Vec = Vector(3, 0.0);
        vel_Vec(0,0) = ang_vel(0,0);
        vel_Vec(1,0) = ang_vel(1,0);
        vel_Vec(2,0) = ang_vel(2,0);
        vel_v_history.push_back(vel_Vec);
<<<<<<< HEAD

        // Vector pos_Vec = Vector(3, 0.0);
        // pos_Vec(0,0) = loopOutput.state[2][0];
        // pos_Vec(1,0) = loopOutput.state[2][1];
        // pos_Vec(2,0) = loopOutput.state[2][2];
        // pos_v_history.push_back(pos_Vec);
        
        
        // Vector vel_Vec = Vector(3, 0.0);
        // vel_Vec(0,0) = loopOutput.state[3][0];
        // vel_Vec(1,0) = loopOutput.state[3][1];
        // vel_Vec(2,0) = loopOutput.state[3][2];
        // vel_v_history.push_back(vel_Vec);
=======
        aoa_history.push_back(AoA*180/M_PI);

        if (grounded && step > 100 && pos(2,0) <= 0.0) {
            grounded = true;
            std::cout << "Vehicle has hit the ground at time: " << step*dt << ".\n";
            std::cout << "Impact speed: " << velocity_magnitude << " m/s.\n";
            std::cout << "Impact angle: " << AoA*180/M_PI << " degrees.\n";
            // break;
        }
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
        
        // Increment iteration counter
        iter++;
    }
    std::cout << "Simulation complete, printing to csv file.\n";

    // Write simulation history to CSV
    ofstream file("simulation_results.csv");
    if (!file) {
        cerr << "Error opening simulation_results.csv for writing." << endl;
        return;
    }
    
    // Write header
    file << "time,x,y,z,vx,vy,vz,thrust,a,b,phi,theta,psi,xac,yac,zac,vxac,vyac,vzac,aoa\n";
    
    // Write data rows
    for (int i = 0; i < num_steps; i++) {
        file << time[i] << ","
             << pos_history[i](0,0) << "," << pos_history[i](1,0) << "," << pos_history[i](2,0) << ","
             << vel_history[i](0,0) << "," << vel_history[i](1,0) << "," << vel_history[i](2,0) << ","
             << command_history[i](0,0) << "," << command_history[i](1,0) << "," << command_history[i](2,0) << ","
             << attitude_history[i](0,0) << "," << attitude_history[i](1,0) << "," << attitude_history[i](2,0) << ","
             << pos_v_history[i](0,0) << "," << pos_v_history[i](1,0) << "," << pos_v_history[i](2,0) << ","
             << vel_v_history[i](0,0) << "," << vel_v_history[i](1,0) << "," << vel_v_history[i](2,0) << ","
             << aoa_history[i] 
             << "\n";
    }
    
    
    file.close();
    cout << "Simulation complete. Results written to simulation_results.csv" << endl;
}

//------------------------- Main Entry Point ---------------------------

int main() {


    // Define and initialize rocket parameters.
    RocketParams P;
    P.m_static = 0.6;
    P.m_gimbal_top = 0.05;
    P.m_gimbal_bottom = 0.05;
    P.m = P.m_static + P.m_gimbal_top + P.m_gimbal_bottom;
    
    // Set all offset vectors to zero (3x1).
    P.gimbal_top_COM_offset = Vector(3, 0.0);
    P.gimbal_bottom_COM_offset = Vector(3, 0.0);
    P.COM_offset = Vector(3, 0.0);
<<<<<<< HEAD
    P.COP = Vector(3, 0.0);          // Can be set as needed.
    P.COP_offset = Vector(3, 0.0);
    // P.COP_offset(2, 0) = 0.01;
=======
    P.COM_offset(2,0) = 7*24*0.001;
    P.COP = Vector(3, 0.0);          // Can be set as needed.
    P.COP_offset = Vector(3, 0.0);
    // P.COP_offset(2, 0) = 0.005;
>>>>>>> d3cceadcdc561096740877513fd6a87c63f8ec22
    P.gimbal_offset = Vector(3, 0.0);
    P.gimbal_offset(2,0) = 7*24*0.001;
    P.gimbal_x_distance = Vector(3, 0.0);
    P.gimbal_y_distance = Vector(3, 0.0);

    P.g = 9.80665;
    // CdX and cdY are not const across time. CdZ is negligibly small for the monoprop!
    // initialize Cd_x and Cd_y. Does it matter what values they are set at the start?
    P.Cd_x = 0.1;  P.Cd_y = 0.1;  
    P.Cd_z = 0;
    // do we need to update reference area?
    P.A_x = 0.7;   P.A_y = 0.7;   P.A_z = 0.3;
    P.air_density = 1.225;
    
    // Inertia: define a 3x3 matrix.
    P.I = Matrix(3, 3, 0.0);
    P.I(0,0) = 0.00940;  P.I(0,1) = 0;  P.I(0,2) = 0;
    P.I(1,0) = 0;  P.I(1,1) = 0.00940;  P.I(1,2) = 0;
    P.I(2,0) = 0;  P.I(2,1) = 0;  P.I(2,2) = 0.00014;
    P.Inv_I = P.I.inverse();
    
    P.gimbal_top_I = 1.0;
    P.gimbal_bottom_I = 1.0;
    
    P.T_max = 100.0;
    P.T_min = 0.0;
    P.gimbal_speed = 5.0;
    P.gimbal_acceleration = 2.0;
    
    P.dt = 0.001;
    
    simulate(P);
    return 0;
}
