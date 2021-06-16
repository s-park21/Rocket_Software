import serial
from struct import *

COMPort = "COM16"
serialPort = serial.Serial(port = COMPort, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

baroData=[]
imuData=[]
GPSData=[]
header=0
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
            dataByte = serialPort.read(1)
            while(dataByte != b'\r'):
                GPSData+=dataByte
                print(dataByte)
                dataByte = serialPort.read(1)

        elif(header== b'\x05'):
            dataByte = serialPort.read(1)
            i=1
            while(dataByte != b'\r'):
                imuData.append(dataByte)
                dataByte = serialPort.read(1)
                print(dataByte)
                i += 1
            i=1
            # timeMs = (int(imuData[0])<<8) | int(imuData[1])
            # accX = float((int(imuData[2])<<8) | int(imuData[3]))/100
            # accY = float((int(imuData[4])<<8) | int(imuData[5]))/100
            # accZ = float((int(imuData[6])<<8) | int(imuData[7]))/100
            # gyroX = float((int(imuData[8])<<8) | int(imuData[9]))/100
            # gyroY = float((int(imuData[10])<<8) | int(imuData[11]))/100
            # gyroZ = float((int(imuData[12])<<8) | int(imuData[13]))/100
            # print(timeMs, " ",accX, " ", accY, " ", accZ, " ", gyroX, " ", gyroY, " ", gyroZ)
            print(imuData[0],imuData[1], imuData[2], imuData[3], imuData[4], imuData[5])
            imuData = []
            print()

            # print(imuData)
            # print()
            # print()

            

    
