#include <SPI.h>
#include <LoRa.h>

int GPSReadData[30];
int imuReadData[30];
byte baroReadData[30];

const int imuPacketLength = 16;
const int gpsPacketLength = 19;

uint32_t timeVal;

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
  int alt;
  double tStamp;
  float speedMps;
  int heading;
  int numSat;
} GPS;

imu imuData;
GPS GPSData;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.write(0x44);             // Byte to say ground station is starting
  while (!LoRa.begin(915E6)) {
    Serial.write(0xFA);
  }
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      byte header = LoRa.read();

      if (header == 0x3) {

      }
      else if (header == 0x4) {
        // GPS data
        for (int i = 0; i < gpsPacketLength; i++) {
          GPSReadData[i] = LoRa.read();
        }
        GPSData.latt = (double)(GPSReadData[0] << 24 | GPSReadData[1] << 16 | GPSReadData[2] << 8 | GPSReadData[3]) / 1000000;
        GPSData.longi = (double)(GPSReadData[4] << 24 | GPSReadData[5] << 16 | GPSReadData[6] << 8 | GPSReadData[7]) / 1000000;
        GPSData.alt = GPSReadData[8] << 8 | GPSReadData[9];
        GPSData.tStamp = (double)(GPSReadData[10] << 24 | GPSReadData[11] << 16 | GPSReadData[12] << 8 | GPSReadData[13]);
        GPSData.speedMps = (float)(GPSReadData[14] << 8 | GPSReadData[15])/100;
        GPSData.heading = GPSReadData[16] << 8 | GPSReadData[17];
        GPSData.numSat = GPSReadData[18];

        Serial.print("Latt: ");
        Serial.print(GPSData.latt);
        Serial.print("    speedMps: ");
        Serial.println(GPSData.heading);
      }
      else if (header == 0x5) {
        // Imu data
        for (int i = 0; i < imuPacketLength; i++) {
          imuReadData[i] = LoRa.read();
        }
        timeVal = imuReadData[0] << 24 | imuReadData[1] << 16 | imuReadData[2] << 8 | imuReadData[3];
        imuData.accX = (float)(imuReadData[4] << 8 | imuReadData[5]) / 100;
        imuData.accY = (float)(imuReadData[6] << 8 | imuReadData[7]) / 100;
        imuData.accZ = (float)(imuReadData[8] << 8 | imuReadData[9]) / 100;
        imuData.gyroX = (float)(imuReadData[10] << 8 | imuReadData[11]) / 100;
        imuData.gyroY = (float)(imuReadData[12] << 8 | imuReadData[13]) / 100;
        imuData.gyroZ = ((float)(imuReadData[14] << 8 | imuReadData[15])) / 100;

        //Serial.println(timeVal);

//        Serial.print(imuData.accX);
//        Serial.print("      ");
//        Serial.print(imuData.accY);
//        Serial.print("      ");
//        Serial.print(imuData.accZ);
//        Serial.print("      ");
      }

    }
    Serial.write("\r\n");
  }
}
