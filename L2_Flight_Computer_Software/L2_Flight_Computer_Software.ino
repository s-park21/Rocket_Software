/*
   This code will run the Design Methods HPR Team 2 flight computer.
   Its purpose is to log data onto an SD card and transmit data to the ground.
*/

//#include "HandleData.h"
//#include "HandleSD.h"
//#include "RadioRF.h"

#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include <TinyGPS++.h>


/*  Library Declarations  */

char RRC3_Data[20] = {};
char GPS_Data[50] = {};
int LEDPin = 32;


char dirname[] = "/data000";



bool launchProceedure = true;                                                               // If ESP32 reboots, it will recommence launch proceedure by default
bool SDStatus = true;
bool MPUStatus = true;
bool GPSLock = false;
bool BaroStatus = false;

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

float timeStamp = 0;

// Lora module pins
const int csPin = 4;
const int resetPin = 35;
const int irqPin = 27;
const long frequency = 915E6;
SPIClass rfSPI(HSPI);


const char IMUFileName[9] = "/imu.csv";
const char baroFileName[10] = "/baro.csv";
const char GPSFileName[9] = "/GPS.csv";

//dataPacket rfPacket;
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
//HandleData handleData(LEDPin, mpu, gps);
//HandleSD handleSD(LEDPin);
TaskHandle_t Task1;
TaskHandle_t Task2;
//RF loraRadio;
//
//imuData imuData;
//gpsData gpsData;
//baroData baroData;

void setup() {
  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);
  Wire.begin();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);
}

void loop() {

}

//Task1code:
/*
  This code runs the data trasmit to the ground station
*/
void Task1code( void * pvParameters ) {
//  rfSPI.begin(14, 12, 13, 4);
  LoRa.setSPI(rfSPI);
  while (loraRadio.setup(csPin, resetPin, irqPin, frequency)) {
    Serial.println("Could not initalise LoRa");
    delay(1000);
  }
  int i = 0;
  for (;;) {
    while (launchProceedure) {
      //    Transmit data to ground
      rfPacket.accX = accX;
      rfPacket.accY = accY;
      rfPacket.accZ = accZ;
      rfPacket.gyroX = gyroX;
      rfPacket.gyroY = gyroY;
      rfPacket.gyroZ = gyroZ;
      rfPacket.latt = latt;
      rfPacket.longi = longi;
      rfPacket.alt = alt;
      rfPacket.tStamp = tStamp;
      rfPacket.numSat = numSat;
      rfPacket.speedMps = speedMps;
      rfPacket.heading = heading;
      rfPacket.timeStamp = timeStamp;
      rfPacket.baroAltitude = baroAltitude;
      rfPacket.baroVelocity = baroVelocity;
      rfPacket.baroTemperature = baroTemperature;
      strcpy(rfPacket.baroEvent, baroEvent);
      rfPacket.batteryVoltage = batteryVoltage;
      //Serial.println("Sending packet");
      //Serial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d, %f, %d, %d, %f, %f, %f, %f\n", rfPacket.accX, rfPacket.accY, rfPacket.accZ, rfPacket.gyroX, rfPacket.gyroY, rfPacket.gyroZ, rfPacket.latt, rfPacket.longi, rfPacket.alt, rfPacket.tStamp, rfPacket.numSat, rfPacket.speedMps, rfPacket.heading, rfPacket.timeStamp, rfPacket.baroAltitude, rfPacket.baroVelocity, rfPacket.baroTemperature, rfPacket.batteryVoltage);
      loraRadio.sendData(&rfPacket, sizeof(rfPacket));
      delay(1);
    }
  }
}


//Task2code:
/*
  This code runs the data reading and logging to the SD card
*/
void Task2code( void * pvParameters ) {
  mpu.begin();
  handleData.setupMPU(3, 2);
  handleData.setupGPS(36, 34, 9600);
  handleData.setupBaro(16, 17, 9600);
  handleSD.setup(dirname);


  //    This is the Core 2 main loop
  for (;;) {
    while (launchProceedure) {
//      mpu.begin();
//      mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
//      mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
//      while (1) {
//        sensors_event_t acc, gyr, temp;
//        mpu.getEvent(&acc, &gyr, &temp);
//        Serial.println(acc.acceleration.x);
//        delay(1000);
//      }
      Serial.println(handleData.getIMUData(&imuData));

      handleSD.appendFile(IMUFileName, handleData.getIMUData(&imuData));

      char* RRC3Array;
      RRC3Array = handleData.getBaroData(&baroData);

      if (RRC3Array != 0x00) {
        handleSD.appendFile(baroFileName, RRC3Array);
      }

      char* newGPSData;
      newGPSData = handleData.getGPSData(&GPSData);
      if (newGPSData != 0x00) {
        handleSD.appendFile(GPSFileName, newGPSData);
        //Serial.println(newGPSData);
      }

      memset(RRC3_Data, 0, sizeof(RRC3_Data));          //  Empty data arrays
      memset(GPS_Data, 0, sizeof(GPS_Data));
    }
    delay(1);
  }
}


void onReceive(int packetSize) {
  // read packet
  int inByte;
  for (int i = 0; i < packetSize; i++) {
    inByte = LoRa.read();
  }

  if (inByte == 1) {
    // Start launch proceedure
    launchProceedure = true;

  }
  else if (inByte = 2) {
    // Sensor status - Send array of bytes to reflect states
  }
  else if (inByte == 0) {
    // Shut down sensor
    launchProceedure = false;
  }

}
