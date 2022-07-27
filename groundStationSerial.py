import serial
from struct import *
import zlib
import os.path
from os import path
import time

COMPort = "COM4"
serialPort = serial.Serial(port = COMPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

baroData=[]
imuData=[]
GPSData=[]
header=0
MSAltitude=0
timeMs=0
MSVelocity=0
global totalMillis; global latt; global longi; global alt; global tStamp; global heading; global numSat; global checkSum
global accX; global accY; global accZ; global gyroX; global gyroY; global gyroZ;  global tempC

# Open log file
os.chdir("FlightData")
directory = "/data"
# print(directory)
dirCount = 1
while (path.exists(directory)):
    directory = "data"
    directory = directory+str(dirCount)
    dirCount+=1
os.mkdir(directory)
print("Writing data to: "+directory)

imuFile = open(directory+"/imu.csv", "w")
GPSFile = open(directory+"/GPS.csv", "w")
RRC3File = open(directory+"/RRC3.csv", "w")
baroFile = open(directory+"/baro.csv", "w")

imuFile.write("Time (ms), Time (us), accX, accY, accZ, gyroX, gyroY, gyroZ, tempC, Packet Error\n")
GPSFile.write("Time (ms), Time (us), latt, longi, alt, tStamp, speedMps, heading, numSat, Packet Error\n")
baroFile.write("Time (ms), Altitude (m), Pressure (Pa), Temperature (degC), Packet Error\n")

while(1):
    if(serialPort.in_waiting > 0):
        header = serialPort.read(1)

        if(header == 0x44):
            print("Ground station booting")

        elif(header == 0xFA):
            print("Ground station LoRa failed to boot")

        elif(header == b'\x03'):
            dataByte = serialPort.read(30)

        elif(header == b'\x06'):
            dataByte = serialPort.read(40)
            # print(dataByte.hex())
            checkSum_calculated = zlib.crc32(dataByte[0:36])
            totalMillis, totalMicros, latt, longi, alt, tStamp, speedMps, heading, numSat, checkSum = unpack('LLlliLiiiI', dataByte)
            if (checkSum_calculated == checkSum):
                latt = latt/1000000
                longi = longi/1000000
                speedMps = round(speedMps/100,2)
                GPSString = str(totalMillis) + ", " + str(totalMicros) + ", " + str(latt) + ", " + str(longi) + ", " + str(alt) + ", " + str(tStamp) + ", " + str(speedMps) + ", " +  str(heading) + ", " + str(numSat) + ", 0\n"
                GPSFile.write(GPSString)
                GPSFile.flush()
                print(str(GPSString))
            else: 
                print("GPS packet error") # Write to log file
                GPSString = str(totalMillis) + ", " + str(totalMicros) + ", " + str(latt) + ", " + str(longi) + ", " + str(alt) + ", " + str(tStamp) + ", " + str(speedMps) + ", " +  str(heading) + ", " + str(numSat) + ", 1\n"
                GPSFile.write(GPSString)
                GPSFile.flush()
                # print(hex(checkSum_calculated), ", ",hex(checkSum))

            # print(latt, " ", longi, " ", alt)
            # print(dataByte.hex())

        elif(header== b'\x05'):
            dataByte = serialPort.read(40)
            # print(dataByte)
            
            checkSum_calculated = zlib.crc32(dataByte[0:36])
            totalMillis, totalMicros, accX, accY, accZ, gyroX, gyroY, gyroZ, tempC, checkSum = unpack('LLiiiiiiiI', dataByte) 
            if (checkSum_calculated == checkSum):
                accX = round(accX/100,2)
                accY = round(accX/100,2)
                accZ = round(accX/100,2)
                gyroX = round(gyroZ/100,2)
                gyroY = round(gyroZ/100,2)
                gyroZ = round(gyroZ/100,2)
                tempC = round(tempC/100,2)
                imuString = str(totalMillis)+", "+str(totalMicros)+", "+str(accX)+", "+str(accY)+", "+str(accZ)+", "+str(gyroX)+", "+str(gyroY)+", "+str(gyroZ) + ", " + str(tempC)+", 0\n"
                imuFile.write(str(imuString))
                imuFile.flush()
                # print(dataByte[8:12].hex(),", ",checkSum)
                # print(totalMillis, " ", accX, " ", accY, " ", accZ, " ", gyroX, " ", gyroY, " ", gyroZ, " ", tempC)

                # Incoming packets appear to be correct therefore the problems with the imu data must be arising from the unpacking of the data            

            else: 
                print("IMU packet error") # Write bad packets to log file
                imuString = str(totalMillis)+", "+str(totalMicros)+", "+str(accX)+", "+str(accY)+", "+str(accZ)+", "+str(gyroX)+", "+str(gyroY)+", "+str(gyroZ) + ", " + str(tempC)+", 1\n"
                imuFile.write(str(imuString))
                imuFile.flush()

        elif(header== b'\x04'):
            # Barometer data
            dataByte = serialPort.read(20)
            checkSum_calculated = zlib.crc32(dataByte[0:16])
            global MSPressure
            global MSTempC
            prevAlt = MSAltitude
            totalMillis, MSAltitude, MSPressure, MSTempC, checkSum = unpack('LfffI', dataByte)
            MSVelocity = round((MSAltitude-prevAlt)/(timeMs-time.time()*1000),2)
            timeMs = time.time()*1000
            if (checkSum == checkSum_calculated):
                MSAltitude = round(MSAltitude,2)
                MSPressure = round(MSPressure,0)
                MSTempC = round(MSTempC,2)
                baroFile.write(str(totalMillis) + ", " + str(MSAltitude) + ", " + str(MSPressure) + ", " + str(MSTempC)+", 0\n")
                baroFile.flush()
            else:
                print("baro packet error")
                baroFile.write(str(totalMillis) + ", " + str(MSAltitude) + ", " + str(MSPressure) + ", " + str(MSTempC)+", 1\n")
                baroFile.flush()
        # print("Altitude: "+str(MSAltitude)+"     Velocity: "+str(MSVelocity))