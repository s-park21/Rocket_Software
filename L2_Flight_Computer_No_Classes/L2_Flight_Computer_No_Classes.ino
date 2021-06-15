/*
   This code will run the Design Methods HPR Team 2 flight computer.
   Its purpose is to log data onto an SD card and transmit data to the ground.
*/

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
char imuArray[100];
char GPSArray[120];
char dirname[] = "/data000";



bool launchProceedure = true;                                                               // If ESP32 reboots, it will recommence launch proceedure by default
bool SDStatus = true;
bool MPUStatus = true;
bool GPSLock = false;
bool BaroStatus = false;
bool newGPSData = false;
bool newBaroData = false;

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


float timeStamp = 0;

// Lora module pins
const int csPin = 15;
const int resetPin = 16;
const int irqPin = 27;
const long frequency = 915E6;
SPIClass rfSPI(HSPI);

char a[20], b[20], c[20], d[20], e[20], f[20], g[20], h[20];               //  Initalise char containers

char IMUFileName[9] = "";
char baroFileName[10] = "";
char GPSFileName[9] = "";

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
TaskHandle_t Task1;
TaskHandle_t Task2;

imu imuData;
GPS GPSData;
baro baroData;

File baroFile;
File IMUDataFile;
File GPSDataFile;

const int GPS_RX = 35;
const int GPS_TX = 17;
const int SERIAL1_RX = 34;
const int SERIAL1_TX = 4;

void setup() {

  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);
  Wire.begin();
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);


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

  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSPI(rfSPI);
  while (!LoRa.begin(frequency)) {
    Serial.println("LoRa failed");
  }

  for (;;) {
    while (launchProceedure) {
      //    Transmit data to ground
      //Serial.printf("RFPacket:      imuData.accX: %d    GPSData.latt: %d   baroData.altiutude: %d\n", packet.imuData.accX, packet.GPSData.latt, packet.baroData.altitude);
      LoRa.beginPacket();
      LoRa.write(0x5);
      LoRa.println(imuArray);
      //LoRa.write((uint8_t*)&imuData, sizeof(imuData));
      LoRa.endPacket(false);

      LoRa.beginPacket();
      LoRa.write(0x3);  
      LoRa.println(RRC3_Data);// Send header byte
      //LoRa.write((uint8_t*)&baroData, sizeof(baroData));
      LoRa.endPacket(false);

      LoRa.beginPacket();
      LoRa.write(0x4);
      LoRa.println(GPSArray);
      LoRa.write((uint8_t*)&GPSData, sizeof(GPSData));
      LoRa.endPacket(false);


      vTaskDelay(10);
    }
  }
}


//Task2code:
/*
  This code runs the data reading and logging to the SD card
*/
void Task2code( void * pvParameters ) {
  while (!SD.begin()) {
    Serial.println("Card failed, or not present");
    errorBlink(1);
  }

  for (int i = 0; i < 100; i++) {
    if (SD.exists(dirname)) {
      dirname[strlen(dirname) - 3] = '\0';
      sprintf(dirname, "%s%03d", dirname, i);
    }
    else {
      sprintf(dirname, "%s%03d", dirname, i);
      dirname[strlen(dirname) - 3] = '\0';
      SD.mkdir(dirname);
      break;
    }
  }

  strcpy(baroFileName, dirname);
  char baroF[10] = "/baro.csv";
  strcat(baroFileName, baroF);
  baroFile = SD.open(baroFileName, FILE_WRITE);
  String AltimeterHead = "Time (ms), Altitude (m), Velocity (m/s), Temperature (degC), Events";
  baroFile.print(AltimeterHead);
  baroFile.flush();

  strcpy(IMUFileName, dirname);
  char imuF[10] = "/imu.csv";
  strcat(IMUFileName, imuF);
  IMUDataFile = SD.open(IMUFileName, FILE_WRITE);
  String imuDataHead = "Time (ms), XAccel (m/s^2), YAccel (m/s^2), ZAccel (m/s^2), X Angular Rate (rad/s), Y Angular Rate (rad/s), Z Angular Rate (rad/s)";
  IMUDataFile.println(imuDataHead);
  IMUDataFile.flush();

  strcpy(GPSFileName, dirname);
  char GPSF[10] = "/GPS.csv";
  strcat(GPSFileName, GPSF);
  GPSDataFile = SD.open(GPSFileName, FILE_WRITE);
  String GPSDataHead = "Lattitude , Longitude, Altitude (m), Timestamp (HHMMSSCC), Number of Satellites, Heading (deg), Speed (m/s)";
  GPSDataFile.println(GPSDataHead);
  GPSDataFile.flush();

  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    errorBlink(2);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

  //    This is the Core 2 main loop
  for (;;) {
    while (launchProceedure) {
      //  Get imu data
      sensors_event_t acc, gyr, temp;
      mpu.getEvent(&acc, &gyr, &temp);



      imuData.accX = acc.acceleration.x;
      imuData.accY = acc.acceleration.y;
      imuData.accZ = acc.acceleration.z;

      imuData.gyroX = gyr.gyro.x;
      imuData.gyroY = gyr.gyro.y;
      imuData.gyroZ = gyr.gyro.z;

      dtostrf(millis(), 10, 0, a);                                         //  Convert to string
      dtostrf(imuData.accX, 4, 2, b);
      dtostrf(imuData.accY, 4, 2, c);
      dtostrf(imuData.accZ, 4, 2, d);
      dtostrf(imuData.gyroX, 4, 2, e);
      dtostrf(imuData.gyroY, 4, 2, f);
      dtostrf(imuData.gyroZ, 4, 2, g);
      dtostrf(imuData.tempC, 4, 2, h);

      sprintf(imuArray, "%s,%s,%s,%s,%s,%s,%s,%s", a, b, c, d, e, f, g, h);     //  Convert to character array

      //  Write imu data to SD
      //Serial.println(imuArray);
      IMUDataFile.println(imuArray);

      // *************************************************************************  Barometer *********************************************************************************
      char RRC3_Data[30];

      static byte ndx = 0;                    // Read binary uart data into buffer -> RRC3_Data
      if (Serial1.available() > 0) {
        while (Serial1.available() > 0) {
          char inByte = Serial1.read();
          if (inByte != '\n') {
            newBaroData = true;
            RRC3_Data[ndx] = inByte;
            ndx++;
          }
        }
      }
      ndx = 0;
      if (newBaroData) {
        int i = 0;
        while (RRC3_Data[i] != 0x2C) {    // Discard first packet
          i++;
        }
        char buff[20];
        while (RRC3_Data[i] != 0x2C) {    // Extract altitude data
          buff[i] = RRC3_Data[i];
          i++;
        }
        baroData.altitude = float(atof(buff));
        memset(buff, 0, sizeof(buff));

        while (RRC3_Data[i] != 0x2C) {    // Extract velocity data
          buff[i] = RRC3_Data[i];
          i++;
        }
        baroData.velocity = float(atof(buff));
        memset(buff, 0, sizeof(buff));

        while (RRC3_Data[i] != 0x2C) {    // Extract temperature data
          buff[i] = RRC3_Data[i];
          i++;
        }
        baroData.tempF = float(atof(buff));
        memset(buff, 0, sizeof(buff));

        while (RRC3_Data[i] != 0x2C) {
          buff[i] = RRC3_Data[i];
          i++;
        }
        strcpy(baroData.event, buff);
        memset(buff, 0, sizeof(buff));

        while (RRC3_Data[i] != 0x2C) {
          buff[i] = RRC3_Data[i];
          i++;
        }
        baroData.battVolt = float(atof(buff));
        memset(buff, 0, sizeof(buff));

        // Write to SD card
        //Serial.println(RRC3_Data);
        baroFile.println(RRC3_Data);
      }
      newBaroData = false;

      if (Serial2.available() > 0) {
        while (Serial2.available() > 0) {
          char b = Serial2.read();
          gps.encode(b);
          //Serial.print(b);
          if (gps.location.isValid()) {
            newGPSData = true;
            GPSData.latt = gps.location.lat();
            GPSData.longi = gps.location.lng();
            GPSData.alt = gps.altitude.meters();
            GPSData.tStamp = gps.time.value();
            GPSData.numSat = gps.satellites.value();
            GPSData.heading = gps.course.deg();
            GPSData.speedMps = gps.speed.mps();
          }
        }
      }

      if (newGPSData) {
        
        char lt[20], ln[20], al[20], ts[20], ns[20], hd[20], sm[20];      //  Initalise char container
        dtostrf(GPSData.latt, 9, 5, lt);                                          //  Convert to string
        dtostrf(GPSData.longi, 9, 5, ln);
        dtostrf(GPSData.alt, 5, 2, al);
        dtostrf(GPSData.tStamp, 10, 2, ts);
        dtostrf(GPSData.numSat, 2, 0, ns);
        dtostrf(GPSData.heading, 4, 2, hd);
        dtostrf(GPSData.speedMps, 4, 2, sm);

        sprintf(GPSArray, "%s,%s,%s,%s,%s,%s,%s", lt, ln, al, ts, ns, hd, sm);    //  Convert to character array
        Serial.println(GPSArray);
        GPSDataFile.println(GPSArray);

        memset(RRC3_Data, 0, sizeof(RRC3_Data));          //  Empty data arrays
        memset(GPSArray, 0, sizeof(GPS_Data));
      }
      newGPSData = false;
      IMUDataFile.flush();
      baroFile.flush();
      GPSDataFile.flush();
    }

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

void errorBlink(int scaleFactor) {
  digitalWrite(LEDPin, HIGH);
  delay(500 * scaleFactor);
  digitalWrite(LEDPin, LOW);
  delay(500 * scaleFactor);
}
