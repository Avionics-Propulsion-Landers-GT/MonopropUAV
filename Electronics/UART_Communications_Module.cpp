/* Documentation

UBX Protocol Specifications: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221
UBX GPS Library: https://github.com/loginov-rocks/UbxGps/blob/main/README.md
Flow Control: https://www.silabs.com/documents/public/application-notes/an0059.0-uart-flow-control.pdf

*/

/*
Flow Control Basics:

GPS SIDE:
1. Configure GPS to monitor incoming data
2. If GPS receives XOFF (0x13 or 19 in decimal), GPS must pause transmission
3. If GPS receives XON (0x11 or 17 in decimal), GPS may continue transmission. XON only has an effect if XOFF has just been received. Receiving XON while tranmission is ongoing should have no effect.

TEENSY SIDE:
1. Send XOFF if input buffer is full
2. Wait a set amount of time. Check if there is space in the buffer. Send XON.

*/

#include <fstream>
#include <iostream>
#include <string>
#include <SoftwareSerial.h>
#include <UbxGpsNavPvt.h> //specifies packet type as UBX-NAV-PVT (0x01 0x07)

#define BAUDRATE 115200
#define GPS_RX 2 //modify as needed after sensor and actuation
#define GPS_TX 3 //modify as needed after sensor and actuation

SoftwareSerial ss(GPS_RX,GPS_TX);
UbxGpsNavPvt<SoftwareSerial> gps(ss); 

void serialConnection()
{
    Serial.begin(BAUDRATE);
    gps.begin(BAUDRATE);

}

int createDataFiles() {
    ofstream latitudeFile("latitude.txt");
    ofstream longitudeFile("longitude.txt");
    ofstream altitudeFile("altitude.txt");
    ofstream timeFile("time.txt");
    latitudeFile.close();
    longitudeFile.close();
    altitudeFile.close();
    timeFile.close();
}

double getTime()
{
    if (gps.ready())
    {
        string month = to_string(gps.month);
        string day = to_string(gps.day);
        string year = to_string(gps.year);
        string hour = to_string(gps.hour);
        string min = to_string(gps.min);
        string sec = to_string(gps.sec);
        string time = month + "/" + day + "/" + year + " " + hour + ":" + min + ":" + sec;
        timeFile.open("time.txt", ofstream::app);
        timeFile << time << "\n";
        timeFile.close();
        return time;
    }

}

double getLatitude()
{
    if (gps.ready())
    {
        double latitude = gps.lat*10000000.0;
        latitudeFile.open("latitude.txt", ofstream::app);
        latitudeFile << latitude << "\n";
        latitudeFile.close();
        return latitude;
    }
}

double getLongitude()
{
    if (gps.ready())
    {
        double longitude = gps.lon*10000000.0;
        longitudeFile.open("longitude.txt", ofstream::app);
        longitudeFile << longitude << "\n";
        longitudeFile.close();
        return longitude;
    }
}

double getAltitude()
{
    if (gps.ready())
    {
        double altitude = gps.height*1000.0;
        altitudeFile.open("altitude.txt", ofstream::app);
        altitudeFile << altitude << "\n";
        altitudeFile.close();
        return altitude;
    }
}


