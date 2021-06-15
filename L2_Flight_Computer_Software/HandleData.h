#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SPI.h"
#include <TinyGPS++.h>

typedef struct {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float tempC;
}imuData;

typedef struct {
  double latt;
  double longi;
  double alt;
  double tStamp;
  double speedMps;
  double heading;
  int numSat;
}GPSData;

typedef struct {
  float altitude;
  float velocity;
  float tempF;
  float battVolt;
  char event[5];
}baroData;

class HandleData {
  public:
    HandleData(int LEDPin, Adafruit_MPU6050 mpu, TinyGPSPlus gps):
      LEDPin_{LEDPin},
      mpu_{mpu},
      gps_{gps}
    {
    }
    void setupMPU(int accelRange, int gyroRange);
    void setupGPS(int baudRate, int RX, int TX);
    void setupBaro(int baudRate, int RX, int TX);
    char* getIMUData(imuData* imuData);
    char* getBaroData(baroData* baroData);
    char* getGPSData(GPSData* GPSData);
    int LEDPin_;
    void errorBlink();

  private:
    TinyGPSPlus gps_;
    Adafruit_MPU6050 mpu_;
    bool newBaroData = false;
    bool newGPSData = false;
    Stream * GPSPort;
    Stream * BaroPort;
};
