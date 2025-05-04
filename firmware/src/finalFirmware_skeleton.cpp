#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VL53L0X.h>
#include "mpu9250.h"  // from bolderflight Mpu9250
#include <TinyGPSPlus.h>
#include <Servo.h>
#include <Loop/loop.h>

#define SD_CS_PIN     10
#define ESC_PIN        5
#define SERVO1_PIN     6
#define SERVO2_PIN     7

LoopOutput loopOutput;
LoopInput loopInput;

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
;

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
    Serial.begin(115200);
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

    loopOutput.command = {0,0,0};
    std::vector<double> prevCommand = loopOutput.command;
    std::vector<double> previous_command = prevCommand;
    std::vector<double> previous_previous_command;
}

void loop() {
    unsigned long now = millis();
    dt = now - last_loop_time;

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
    if (logFile) {
        logFile.println("System halted due to terminal error.");
        logFile.flush();
    }
    while (true);
}