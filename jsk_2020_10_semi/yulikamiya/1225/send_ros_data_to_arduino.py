import serial

class Arduino:
    ser = None

    def __init__(self, COM, speed, timeout):
        self.ser = serial.Serial(COM, speed, timeout = timeout)
        self.ser.readline()

    def __del__(self):
        self.ser.close()

    def motor(self, deg):
        self.ser.write(bytes([deg]))

ard =Arduino('/dev/ttyACM0', 57600, 1)
while True:
    print("executed")
    angle = input()
    #if(angle.isdecimal()):
    ard.motor(int(angle))