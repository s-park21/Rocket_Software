#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SPI.h"
#include <TinyGPS++.h>
#include "HandleData.h"


void HandleData::setupMPU(int accelRange, int gyroRange) {
  Serial.println("Setting up MPU");
  Wire.begin();
  while (!mpu_.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    errorBlink();
  }

  if (accelRange == 0) {
    mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
  }
  else if (accelRange == 1) {
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
  }
  else if (accelRange == 2) {
    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  }
  else if (accelRange == 3) {
    mpu_.setAccelerometerRange(MPU6050_RANGE_16_G);
  }

  if (gyroRange == 0) {
    mpu_.setGyroRange(MPU6050_RANGE_250_DEG);
  }
  else if (gyroRange == 1) {
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  }
  else if (gyroRange == 2) {
    mpu_.setGyroRange(MPU6050_RANGE_1000_DEG);
  }
  else if (gyroRange == 3) {
    mpu_.setGyroRange(MPU6050_RANGE_2000_DEG);
  }



}

void HandleData::setupGPS(int RX, int TX, int baudRate) {
  Serial2.begin(baudRate, SERIAL_8N1, RX, TX);
}

void HandleData::setupBaro(int baudRate, int RX, int TX) {
  Serial1.begin(baudRate, SERIAL_8N1, RX, TX);
}

char* HandleData::getIMUData(imuData* imuData) {
  static char imuArray[50];

  /* Get new sensor events with the readings */  
  sensors_event_t acc, gyr, temp;
  mpu_.getEvent(&acc, &gyr, &temp);


  // Handle IMU Data
  imuData->accX = acc.acceleration.x;                                          //  Read data from MPU-6050
  imuData->accY = acc.acceleration.y;
  imuData->accZ = acc.acceleration.z;

  imuData->gyroX = gyr.gyro.x;
  imuData->gyroY = gyr.gyro.y;
  imuData->gyroZ = gyr.gyro.z;

  char a[20], b[20], c[20], d[20], e[20], f[20], g[20];               //  Initalise char container

  dtostrf(millis(), 8, 2, a);                                         //  Convert to string
  dtostrf(imuData->accX, 4, 2, b);
  dtostrf(imuData->accY, 4, 2, c);
  dtostrf(imuData->accZ, 4, 2, d);
  dtostrf(imuData->gyroX, 4, 2, e);
  dtostrf(imuData->gyroY, 4, 2, f);
  dtostrf(imuData->gyroZ, 4, 2, g);

  // Can get temperature values from MPU6050 -- maybe implement into packet format

  //Serial.printf("accX %s, accY %s,accZ %s,gyroX %s,gyroY %s,gyroZ %s\n", acc.acceleration.x, acc.acceleration.y, acc.acceleration.z, gyr.gyro.x, gyr.gyro.y, gyr.gyro.z);
  sprintf(imuArray, "%s,%s,%s,%s,%s,%s,%s", a, b, c, d, e, f, g);     //  Convert to character array
  Serial.println(imuArray);

  return imuArray;
}

char* HandleData::getBaroData(baroData* baroData) {
  bool newBaroData = false;
  char RRC3_Data[30];

  static byte ndx = 0;
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
    baroData->altitude = float(atof(buff));
    memset(buff, 0, sizeof(buff));

    while (RRC3_Data[i] != 0x2C) {    // Extract velocity data
      buff[i] = RRC3_Data[i];
      i++;
    }
    baroData->velocity = float(atof(buff));
    memset(buff, 0, sizeof(buff));

    while (RRC3_Data[i] != 0x2C) {    // Extract temperature data
      buff[i] = RRC3_Data[i];
      i++;
    }
    baroData->tempF = float(atof(buff));
    memset(buff, 0, sizeof(buff));

    while (RRC3_Data[i] != 0x2C) {
      buff[i] = RRC3_Data[i];
      i++;
    }
    strcpy(baroData->event, buff);
    memset(buff, 0, sizeof(buff));

    while (RRC3_Data[i] != 0x2C) {
      buff[i] = RRC3_Data[i];
      i++;
    }
    baroData->battVolt = float(atof(buff));
    memset(buff, 0, sizeof(buff));

    return RRC3_Data;
  }
  return 0x00;
}

char* HandleData::getGPSData(GPSData* GPSData) {
  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      newGPSData = true;
      char b = Serial2.read();
      gps_.encode(b);
      //Serial.print(b);
      if (gps_.location.isValid()) {
        GPSData->latt = gps_.location.lat();
        GPSData->longi = gps_.location.lng();
        GPSData->alt = gps_.altitude.meters();
        GPSData->tStamp = gps_.time.value();
        GPSData->numSat = gps_.satellites.value();
        GPSData->heading = gps_.course.deg();
        GPSData->speedMps = gps_.speed.mps();
      }
    }
  }

  if (!newGPSData) return 0x00;

  char GPSArray[120];
  char lt[20], ln[20], al[20], ts[20], ns[20], hd[20], sm[20];      //  Initalise char container
  dtostrf(GPSData->latt, 9, 5, lt);                                          //  Convert to string
  dtostrf(GPSData->longi, 9, 5, ln);
  dtostrf(GPSData->alt, 5, 2, al);
  dtostrf(GPSData->tStamp, 10, 2, ts);
  dtostrf(GPSData->numSat, 2, 0, ns);
  dtostrf(GPSData->heading, 4, 2, hd);
  dtostrf(GPSData->speedMps, 4, 2, sm);

  sprintf(GPSArray, "%s,%s,%s,%s,%s,%s,%s", lt, ln, al, ts, ns, hd, sm);    //  Convert to character array

  return GPSArray;
}

void HandleData::errorBlink() {
  pinMode(LEDPin_, OUTPUT);
  digitalWrite(LEDPin_, HIGH);
  delay(500);
  digitalWrite(LEDPin_, LOW);
  delay(500);
}
