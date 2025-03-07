#include "UbxGpsNavPvt.h"

UbxGpsNavPvt<HardwareSerial> gps(Serial3);

void setup()
{
    Serial.begin(9600);
    gps.begin(9600);
}

void loop()
{
    if (gps.ready())
    {
        Serial.print(gps.lon / 10000000.0, 7);
        Serial.print(',');
        Serial.print(gps.lat / 10000000.0, 7);
        Serial.print(',');
        Serial.print(gps.height / 1000.0, 3);
        Serial.print(',');
        Serial.println(gps.gSpeed * 0.0036, 5);
    }
}