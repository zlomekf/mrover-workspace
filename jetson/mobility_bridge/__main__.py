import serial
#import lcm
#from rover_msgs import AutonState, NavStatus
import Adafruit_BBIO.UART as UART
import numpy as np
import struct


baud = 115200


def main():
    UART.setup("UART4")
    #ttyS4 board specific or how determined?
    with serial.Serial(port="/dev/ttyS4", baudrate=baud) as ser:
        ser.close()
        ser.open()
        while 1:
            try:
                # ser.write(int.to_bytes(-100, 1, byteorder='little', signed=True))
                rpm = np.int16(70<<8|100)  #decimal ver of 70, -100
                ser.write(struct('<B', -25530)) # < makes it little endian and flips it to #-100, 70
                #https://pymotw.com/2/struct/
            except (ser.SerialTimeoutException):
                print("Serial is not open")
 #       while(True):
 #           lcm_.handle()
'''        print("start")
        while 1:
                reading = ser.read()#.decode('utf-8') #ser.read() for a byte
                int_val = int.from_bytes(reading, byteorder='little', signed=True)
                print(int_val)
'''
if __name__ == "__main__":
    main()