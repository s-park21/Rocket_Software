#pragma once
#include <SPI.h>             
#include <LoRa.h>

//typedef struct {
//  float accX;
//  float accY;
//  float accZ;
//  float gyroX;
//  float gyroY;
//  float gyroZ;
//  float tempC;
//}imuData;
//
//typedef struct {
//  double latt;
//  double longi;
//  double alt;
//  double tStamp;
//  double speedMps;
//  double heading;
//  int numSat;
//}GPSData;
//
//typedef struct {
//  float altitude;
//  float velocity;
//  float tempF;
//  float battVolt;
//  char event[5];
//}baroData;
//
// A struct to hold the data ready for transmission
typedef struct dataPacket {
  imuData imu;
  GPSData GPS;
  float timeStamp;
  baroData baro;
} dataPacket;


class RF {
    public:
    bool setup(const int csPin, const int resetPin, const int irqPin, const long frequency);
    void sendData(dataPacket* rfPacket, size_t packetSize);
};
