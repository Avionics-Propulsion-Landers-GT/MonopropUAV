#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VL53L0X.h>
#include "mpu9250.h"  // from bolderflight Mpu9250
#include <TinyGPSPlus.h>
#include <Servo.h>

#define SD_CS_PIN     10
#define ESC_PIN        5
#define SERVO1_PIN     6
#define SERVO2_PIN     7

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

    bfs::MpuConfig config;
    config.accel_fs = bfs::Mpu9250::ACCEL_RANGE_4G;
    config.gyro_fs = bfs::Mpu9250::GYRO_RANGE_500DPS;
    config.dlpf_cfg = bfs::Mpu9250::DLPF_BANDWIDTH_20HZ;
    config.i2c_addr = bfs::Mpu9250::I2C_ADDR_PRIM;

    imu.Config(&Wire, config);

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
