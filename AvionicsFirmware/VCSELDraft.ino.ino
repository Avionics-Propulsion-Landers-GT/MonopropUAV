
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X tof;

// initializing sensor
void initialize() {

  Serial.println("Initializing VL53L1X...");

  // begin commmunication
  Wire.begin();
  tof.setBus(&Wire);
  tof.setAddress(0x52);

  // initialze time of flight (tof) sensor
    if (!tof.init()) {
    Serial.println("Failed to detect VL53L1X!");
    handleSensorError("VL53L1X");
    return;
  }

  // configure distance measurement parameters
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(16000);
  tof.setTimeout(500);

  // start reading continuiously
  tof.startContinuous(50);
  Serial.println("VL53L1X initialized successfully!");
}

// read data
void reading() {
  uint16_t distance = tof.read();

  if (tof.timeoutOccurred()) {
    Serial.println("VL53L1X timeout!");
    return;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");
  // TODO: sending data

}
// some error handling
void handleSensorError(const char* sensorName) {
  Serial.print("Error initializing ");
  Serial.println(sensorName);
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  initialize();
}

void loop() {
  reading();
  delay(500);

}
