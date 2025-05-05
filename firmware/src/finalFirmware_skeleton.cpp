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

// LoopOutput loopOutput;
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
float computeESCCommand();
float computeServo1Command();
float computeServo2Command();

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

    float esc_command     = computeESCCommand();       // 0.0–1.0
    float servo1_command  = computeServo1Command();    // 0.0–1.0
    float servo2_command  = computeServo2Command();    // 0.0–1.0

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

float computeESCCommand() {
    return 0.5; // Placeholder
}

float computeServo1Command() {
    return 0.5; // Placeholder
}

float computeServo2Command() {
    return 0.5; // Placeholder
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