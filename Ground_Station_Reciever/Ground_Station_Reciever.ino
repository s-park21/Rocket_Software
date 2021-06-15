#include <SPI.h>
#include <LoRa.h>
typedef struct {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float tempC;
} imu;

typedef struct {
  double latt;
  double longi;
  double alt;
  double tStamp;
  double speedMps;
  double heading;
  int numSat;
} GPS;

typedef struct {
  float altitude;
  float velocity;
  float tempF;
  float battVolt;
  char event[5];
} baro;

baro baroData;
GPS GPSData;
imu imuData;

void setup() {
  Serial.begin(115200);
  while (!Serial);


  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet

  int packetSize = LoRa.parsePacket ();
  if (packetSize) // Only read if there is some data to read..
  {
    byte header = LoRa.read();
    if (header == 0x3) {
      Serial.print(0x3);
    }
    else if (header == 0x4) {
      Serial.print(0x4);
    }
    else if (header == 0x5) {
      Serial.print(0x5);
    }
    while (LoRa.available() > 0) {
        Serial.print((char)LoRa.read());
      }
      Serial.println("");
  }
}
