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
        if(header == b'3'):
            dataByte = serialPort.read(1)
            while(dataByte != b'\r'):
                baroData+=dataByte
                # print(dataByte)
                dataByte = serialPort.read(1)
            # baroStruct = unpack('ffffc?', baroData)
        elif(header == b'4'):
            dataByte = serialPort.read(1)
            while(dataByte != b'\r'):
                GPSData+=dataByte
                # print(dataByte)
                dataByte = serialPort.read(1)
            # GPSStruct = unpack('ddddddi?', GPSData)
        elif(header== b'5'):
            dataByte = serialPort.read(1)
            while(dataByte != b'\r'):
                imuData+=dataByte
                # print(dataByte)
                dataByte = serialPort.read(1)
            # imuStruct = unpack('fffffff', bytearray(imuData))
            # print(''.join(chr(i) for i in imuData))
            print(baroData)
            print(GPSData)
            print(imuData)
            imuData=[]
            print()
            print()
            print()
            

    
