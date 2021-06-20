import serial
from struct import *
import zlib
import os.path
from os import path

COMPort = "COM16"
serialPort = serial.Serial(port = COMPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

baroData=[]
imuData=[]
GPSData=[]
header=0

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

imuFile = open("imu.csv", "w")
GPSFile = open("GPS.csv", "w")
baroFile = open("baro.csv", "w")

imuFile.write("totalMillis, accX, accY, accZ, gyroX, gyroY, gyroZ, tempC")
GPSFile.write("latt, longi, alt, tStamp, speedMps, heading, numSat")




f = open("demofile2.txt", "a")
f.write("Now the file has more content!")

while(1):
    if(serialPort.in_waiting > 0):
        while(serialPort.read(1) != b'\n'):
            header = serialPort.read(1)

        header = serialPort.read(1)

        if(header == 0x44):
            print("Ground station booting")

        elif(header == 0xFA):
            print("Ground station LoRa failed to boot")

        elif(header == b'\x03'):
            dataByte = serialPort.read(1)
            while(dataByte != b'\r'):
                baroData+=dataByte
                dataByte = serialPort.read(1)

        elif(header == b'\x04'):
            dataByte = serialPort.read(28)
            latt, longi, alt, tStamp, speedMps, heading, numSat = unpack('lliLiii', dataByte)
            latt = latt/1000000
            longi = longi/1000000
            speedMps = round(speedMps/100,2)
            GPSString = latt,", ",longi,", ",alt,", ",tStamp,", ",heading,", ",numSat
            GPSFile.write(GPSString)

            # print(latt, " ", longi, " ", alt)
            # print(dataByte)

        elif(header== b'\x05'):
            dataByte = serialPort.read(33)
            # print(bin(dataByte))
            checkSum_calculated = sum(dataByte[0:32])&0xFF
            checkSum_calculated = (checkSum_calculated^0xFF)
            checkSum_calculated = checkSum_calculated+1;
            totalMillis, accX, accY, accZ, gyroX, gyroY, gyroZ, tempC, checkSum = unpack('LiiiiiiiB', dataByte) 
            accX = round(accX/100,2)
            accY = round(accX/100,2)
            accZ = round(accX/100,2)
            gyroX = round(gyroZ/100,2)
            gyroY = round(gyroZ/100,2)
            gyroZ = round(gyroZ/100,2)
            # print(totalMillis, " ", accX, " ", accY, " ", accZ, " ", gyroX, " ", gyroY, " ", gyroZ, " ", hex(checkSum), hex(checkSum_calculated))
            # byteString = b'\xa8\x84\xd\x00\xb4\x00\x00\x00\x00\x00\x00\x00\xd0\x02\x00\x00\xfb\xff\xff\xff\xff\xff\xff\xff\x01\x00\x00\x00\x00\x00\x00\x00\xad'
            # arduiBytes = b'\xA8\x84\xD\x00\xB4\x00\x00\x00\x00\xFD\xFF\xFF\xFF\xC7\x02\x00\x00\xFB\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x01\x00\x00\x00\x00\x00\x00\x00\xAD 0 0 0

#9C 98 1 0 AE 0 0 0 F9 FF FF FF D3 2 0 0 FB FF FF FF FF FF FF FF 1 0 0 0 0 0 0 0 DD 0 0 0

            recieved = 0xad
            calculatefd = 0x4c
            # print(hex(((sum(byteString[0:32])&0xFF)^0xFF) +1))
            # print(hex(((sum(arduiBytes[0:32])&0xFF)^0xFF) +1))
    
