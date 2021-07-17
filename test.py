import serial

COMPort = "COM3"
serialPort = serial.Serial(port = COMPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

while(1):
        print(serialPort.read(100).hex())