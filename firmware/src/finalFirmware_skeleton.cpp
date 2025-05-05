#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VL53L0X.h>
#include "mpu9250.h"  // from bolderflight Mpu9250
#include <TinyGPSPlus.h>
#include <Servo.h>
#include <Loop/loop.h>
#include "Loop/loop.h"
#include "Loop/init.h"

#define DEBUG_NO_SENSORS 1
#define SD_CS_PIN     10
#define ESC_PIN        5
#define SERVO1_PIN     6
#define SERVO2_PIN     7

LoopOutput loopOutput;
// LoopInput loopInput;

HardwareSerial &gpsSerial = Serial1;
TinyGPSPlus gps;
Adafruit_VL53L0X lox;
bfs::Mpu9250 imu;

File logFile;

Servo esc;
Servo servo1;
Servo servo2;

bool gps_ok = false, lidar_ok = false, imu_ok = false;
bool terminal_error = false;

unsigned long last_gps_time   = 0;
unsigned long last_lidar_time = 0;
unsigned long last_imu_time   = 0;
const unsigned long SENSOR_TIMEOUT_MS = 100;

unsigned long dt              = 0;
unsigned long last_loop_time = 0;
const unsigned long LOOP_TIMEOUT_MS = 50;
unsigned int num_steps        = 40000;
unsigned long init_setup_time;
unsigned long path_time = 0;

std::vector<double> time;
std::vector<Vector> delta_pos_desired;
std::vector<Vector> accel_desired;
std::vector<Vector> pos_desired;

std::vector<double> command;
std::vector<double> prevCommand;
std::vector<double> previous_command;
std::vector<double> previous_previous_command;

std::vector<std::vector<double>> previous_state;
std::vector<std::vector<double>> prevState;

std::vector<double> gps_data;
double lidar_data;
// lidar data goes here
std::vector<double> vel_data;
std::vector<double> acc_data;
std::vector<double> att_data;
// Quaternion att_quat;
std::vector<double> omega_data;
std::vector<double> alpha_data;

std::vector<std::vector<double>> current_state = {gps_data, vel_data, att_data, omega_data};
std::vector<double> command;

SystemComponents system;
std::vector<bool> status;

Vector thrust_gimbal(3, 0.0);
double F_thrust_mag = 0.0;

// Watchdog placeholder — Teensy 4.1 uses WDOG1 or Systick (implement later)
#define resetWatchdog()
#define setupWatchdog()

void logError(const char* msg);
void haltSystem();
void readGPS();
void readLiDAR();
void readIMU();
void checkSafety();
void runControlLoop();
float computeESCCommand(double thrust, double min, double max);
float computeServoCommand(double rad, float zero);

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);
    Serial.println("Firmware starting up...");
    gpsSerial.begin(9600);
    Wire.begin();

    setupWatchdog();

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("ERROR: SD card init failed");
        terminal_error = true;
    } else {
        logFile = SD.open("log.txt", FILE_WRITE);
        if (!logFile) {
            Serial.println("ERROR: Cannot open log file");
            terminal_error = true;
        }
    }

    if (!lox.begin()) {
        logError("VL53L0X init failed");
        lidar_ok = false;
    } else {
        lidar_ok = true;
    }

    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

    if (!imu.Begin()) {
        logError("MPU9250 init failed");
        imu_ok = false;
    } else {
        imu_ok = true;
    }

    esc.attach(ESC_PIN, 1000, 2000);
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

     // build time vector
     std::vector<double> time(num_steps);
     for (int i = 0; i < num_steps; i++) {
         time[i] = i * 0.001; // 1 ms time step
     }
 
     readGPS();
     double latInit = gps.location.rawLat().deg;
     double lngInit = gps.location.rawLng().deg;
     double altInit = gps.altitude.meters();
     // declare initial conditions for loop
     std::vector<double> gpsInit = {latInit, lngInit, altInit};
     std::vector<double> velInit = {0,0,0};
     std::vector<double> initAtt = {0,0,0};
     std::vector<double> omegaInit = {0,0,0};
     
     std::vector<std::vector<double>> initState = {gpsInit, velInit, initAtt, omegaInit};
     SystemComponents system = init(gpsInit, initState, dt);
 
     // Build desired trajectory: pure vertical motion
     std::vector<Vector> delta_pos_desired;
     delta_pos_desired.reserve(num_steps);
 
     for (int i = 0; i < num_steps; i++) {
         double t_val = time[i];
         double dz_dt = 0.0;
 
         if (t_val >= 0.0 && t_val < 10.0) {
             dz_dt = 50.0 * 0.5 * (M_PI / 10.0) * sin(M_PI * t_val / 10.0);
         } else if (t_val >= 10.0 && t_val < 20.0) {
             dz_dt = 0.0;
         } else if (t_val >= 20.0 && t_val < 30.0) {
             dz_dt = -49.0 * 0.5 * (M_PI / 10.0) * sin(M_PI * (t_val - 20.0) / 10.0);
         } else if (t_val >= 30.0 && t_val <= 40.0) {
             dz_dt = 0.0;
         }
 
         Vector vel_des(3, 0.0);
         vel_des(2,0) = dz_dt;
         delta_pos_desired.push_back(vel_des);
     }
 
     std::vector<Vector> accel_desired;
     accel_desired.reserve(num_steps);
 
     for (int i = 0; i < num_steps; i++) {
         double t_val = time[i];
         double d2z_dt2 = 0.0;
         double pi_over_10 = M_PI / 10.0;
         double factor = 50.0 * 0.5 * pi_over_10 * pi_over_10;
 
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
 
     std::vector<Vector> pos_desired;
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
 
     // Set up command and state feedback loops
     loopOutput.state = {gpsInit, velInit, initAtt, omegaInit};
     std::vector<std::vector<double>> prevState = loopOutput.state;
     std::vector<std::vector<double>> previous_state;
 
     loopOutput.command = {0,0.5,0.5};
     std::vector<double> prevCommand = loopOutput.command;
     std::vector<double> previous_command = prevCommand;
     std::vector<double> previous_previous_command;
}

void loop() {
    Serial.println("Loop tick");
    delay(1000); // just to reduce spam
    unsigned long now = millis();
    dt = now - last_loop_time;

    path_time = now - init_setup_time;
    if (dt > LOOP_TIMEOUT_MS) {
        logError("Loop timeout detected");
    }

    last_loop_time = now;

    readGPS();
    readLiDAR();
    readIMU();

    if (terminal_error) {
        logError("TERMINAL ERROR: Halting system");
        haltSystem();
    }

    runControlLoop();

    resetWatchdog();
}

void readGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        last_gps_time = millis();
        gps_ok = true;
    } else if (millis() - last_gps_time > SENSOR_TIMEOUT_MS) {
        logError("GPS timeout");
        gps_ok = false;
    }
}

void readLiDAR() {
    VL53L0X_RangingMeasurementData_t measure;
    if (lox.rangingTest(&measure, false)) {
        if (measure.RangeStatus != 4) {
            last_lidar_time = millis();
            lidar_ok = true;
        } else if (millis() - last_lidar_time > SENSOR_TIMEOUT_MS) {
            logError("LiDAR invalid reading");
            lidar_ok = false;
        }
    }
}

void readIMU() {
    if (imu.Read()) {
        last_imu_time = millis();
        imu_ok = true;
    } else if (millis() - last_imu_time > SENSOR_TIMEOUT_MS) {
        logError("IMU read timeout");
        imu_ok = false;
    }
}

void checkSafety() {
    if (!gps_ok || !lidar_ok || !imu_ok) {
        logError("One or more sensors offline");
    }
}

void runControlLoop() {
    checkSafety();

    // code here
    
    // path_time is current timestep for numsteps because both are ms
    // double lat = gps.location.rawLat().deg;
    // double lng = gps.location.rawLng().deg;
    // double altitude = gps.altitude.meters();

    // gps_data = {lat, lng, altitude};
    // get current desired state from trajectory creation
    int step = path_time;
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

    // accel, then gyro for 6ax
    // find out what is outputted by 6ax imu from docs
    acc_data = {imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2()};
    omega_data = {imu.gyro_x_radps(), imu.gyro_y_radps(), imu.gyro_z_radps()};

    lidar_data = lox.readRange();
    lidar_data /= 1000;

    std::vector<std::vector<double>> sensor_values = {
        {0, omega_data[0], omega_data[1], omega_data[2], acc_data[0], acc_data[1], acc_data[2]},
        {gps_data[0], gps_data[1], gps_data[2]},
        {0 , lidar_data}
    };

    // Set prevCommands
    previous_state = prevState;
    prevState = loopOutput.state; 
    previous_previous_command = previous_command;
    previous_command = prevCommand;
    prevCommand = loopOutput.filteredCommand; 

    // Create loop input structure
    LoopInput loopInput = {
        sensor_values,
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
        step
    };

    loopOutput = loop(loopInput);

    // TODO: write up commands once we get more info from structures and avionics
    std::vector<double> command = loopOutput.filteredCommand;
    
    // 6. Convert commands to thrust and gimbal angles
    F_thrust_mag = command[0];  // Thrust magnitude
    thrust_gimbal(0,0) = command[1];  // X-axis gimbal angle in rad
    thrust_gimbal(1,0) = command[2];  // Y-axis gimbal angle in rad

    // TODO: find thrust max in Newtons
    // TODO: find servo zeros with structures,
    //       and maybe rewrite the computeServoCommand to take into account potential nonlinearities

    float esc_command     = computeESCCommand(F_thrust_mag, 0, 10);       // 0.0–1.0
    float servo1_command  = computeServoCommand(thrust_gimbal(0,0), 0.5);    // 0.0–1.0
    float servo2_command  = computeServoCommand(thrust_gimbal(1,0), 0.5);    // 0.0–1.0

    esc_command     = constrain(esc_command, 0.0, 1.0);
    servo1_command  = constrain(servo1_command, 0.0, 1.0);
    servo2_command  = constrain(servo2_command, 0.0, 1.0);

    int esc_pwm = map(esc_command * 1000, 0, 1000, 1000, 2000);
    esc.writeMicroseconds(esc_pwm);

    int servo1_angle = map(servo1_command * 1000, 0, 1000, 0, 180);
    int servo2_angle = map(servo2_command * 1000, 0, 1000, 0, 180);

    servo1.write(servo1_angle);
    servo2.write(servo2_angle);
}

float computeESCCommand(double thrust, double min, double max) {
    // min and max are in thrust units (Newtons)
    thrust = constrain(thrust, min, max);
    float return_var = thrust / (max - min);
    return return_var;
}

float computeServoCommand(double rad, float zero) {
    double rad_to_servo = 0.7957728546;
    float return_var = rad_to_servo * rad + zero;
    return return_var;
}

void logError(const char* msg) {
    Serial.println(msg);
    if (logFile) {
        logFile.print("[ERROR] ");
        logFile.println(msg);
        logFile.flush();
    }
}

void haltSystem() {
    esc.writeMicroseconds(1000);

    Serial.println("System halted.");
    if (logFile) {
        logFile.println("System halted due to terminal error.");
        logFile.flush();
    }

    delay(500);  // Give Serial a moment to flush output
    while (true) {
        delay(1000);  // Idle, but not spinlock
    }
}