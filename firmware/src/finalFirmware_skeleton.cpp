#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_VL53L0X.h"
#include "MPU9250.h"
#include <TinyGPSPlus.h>
#include <Servo.h>
#include <imxrt.h> // Teensy 4.1 watchdog

#define SD_CS_PIN 10
#define ESC_PIN 5
#define SERVO1_PIN 6
#define SERVO2_PIN 7

HardwareSerial &gpsSerial = Serial1;
TinyGPSPlus gps;
Adafruit_VL53L0X lox;
MPU9250 imu(Wire, 0x68);
File logFile;

Servo esc;
Servo servo1;
Servo servo2;

bool gps_ok = false, lidar_ok = false, imu_ok = false;
bool terminal_error = false;
unsigned long last_gps_time = 0, last_lidar_time = 0, last_imu_time = 0;
const unsigned long SENSOR_TIMEOUT_MS = 100;
unsigned long dt = 0;
unsigned long last_loop_time = 0;
const unsigned long LOOP_TIMEOUT_MS = 50;

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    Wire.begin();

    // Start watchdog: 1 second timeout
    const uint32_t watchdogTimeout = 1000;
    IWDG_SR = 0; // Clear status flags
    IWDG_RLR = watchdogTimeout;
    IWDG_KR = 0xCCCC; // Start watchdog

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

    if (imu.begin() != 0) {
        logError("MPU9250 init failed");
        imu_ok = false;
    } else {
        imu.setAccelRange(MPU9250::ACCEL_RANGE_4G);
        imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
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

    // Reset watchdog
    IWDG_KR = 0xAAAA;

    delay(0.1);
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
    if (imu.readSensor() == 0) {
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

    // Compute control commands here
    float esc_command = computeESCCommand();        // 0.0–1.0
    float servo1_command = computeServo1Command();  // 0.0–1.0
    float servo2_command = computeServo2Command();  // 0.0–1.0

    esc_command = constrain(esc_command, 0.0, 1.0);
    servo1_command = constrain(servo1_command, 0.0, 1.0);
    servo2_command = constrain(servo2_command, 0.0, 1.0);

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
