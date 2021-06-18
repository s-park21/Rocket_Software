import serial
from struct import *
import zlib

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
            dataByte = serialPort.read(29)
            checksum_calculated = b'%02X' % (sum(dataByte) & 0xFF)

            latt, longi, alt, tStamp, speedMps, heading, numSat ,checksum_recieved = unpack('lliLiiiB', dataByte)
            latt = latt/1000000
            longi = longi/1000000
            speedMps = round(speedMps/100,2)
            print("crc_calculated: ", hex(checksum_calculated), "     crc_recieved: ", hex(checksum_recieved))

        elif(header== b'\x05'):
            dataByte = serialPort.read(28)
            totalMillis, accX, accY, accZ, gyroX, gyroY, gyroZ = unpack('Liiiiii', dataByte) 
            accX = round(accX/100,2)
            accY = round(accX/100,2)
            accZ = round(accX/100,2)
            gyroX = round(gyroZ/100,2)
            gyroY = round(gyroZ/100,2)
            gyroZ = round(gyroZ/100,2)
            print(totalMillis, " ", accX, " ", accY, " ", accZ, " ", gyroX, " ", gyroY, " ", gyroZ)
            

    
