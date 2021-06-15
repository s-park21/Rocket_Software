#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "HandleSD.h"


bool HandleSD::setup(char* dirname) {
    while (!SD.begin()) {
    Serial.println("Card failed, or not present");
    errorBlink();
    }

    for(int i=0; i<100; i++) {
    if(SD.exists(dirname)) {
      dirname[strlen(dirname)-3] = '\0';
      sprintf(dirname, "%s%03d",dirname,i);
    }
    else {
      sprintf(dirname, "%s%03d",dirname,i);
      dirname[strlen(dirname)-3] = '\0';
      SD.mkdir(dirname);
      break;
    }
  }

  strcpy(baroFile, dirname);
  char baroF[10] = "/baro.csv";
  strcat(baroFile, baroF);
  File AltimeterDataFile = SD.open(baroFile, FILE_WRITE);
  String AltimeterHead = "Time (ms), Altitude (m), Velocity (m/s), Temperature (degC), Events\n";
  AltimeterDataFile.print(AltimeterHead);
  AltimeterDataFile.close();


  strcpy(imuFile, dirname);
  char imuF[10] = "/imu.csv";
  strcat(imuFile, imuF);
  File imuDataFile = SD.open(imuFile, FILE_WRITE);
  String imuDataHead = "Time (ms), XAccel (m/s^2), YAccel (m/s^2), ZAccel (m/s^2), X Angular Rate (rad/s), Y Angular Rate (rad/s), Z Angular Rate (rad/s)";
  imuDataFile.println(imuDataHead);
  imuDataFile.close();


  strcpy(GPSFile, dirname);
  char GPSF[10] = "/GPS.csv";
  strcat(GPSFile, GPSF);
  File GPSDataFile = SD.open(GPSFile, FILE_WRITE);
  String GPSDataHead = "Lattitude , Longitude, Altitude (m), Timestamp (HHMMSSCC), Number of Satellites, Heading (deg), Speed (m/s)";
  GPSDataFile.println(GPSDataHead);
  GPSDataFile.close();
}

void HandleSD::appendFile(const char* filename, char* data) {
    File DataFile = SD.open(filename, FILE_APPEND);       //  Save to SD card
    DataFile.println(data);
    DataFile.close();
}

void HandleSD::errorBlink() {
  digitalWrite(LEDPin_, HIGH);
  delay(200);
  digitalWrite(LEDPin_, LOW);
  delay(200);
}
