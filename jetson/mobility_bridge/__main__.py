import serial
#import lcm
#from rover_msgs import AutonState, NavStatus
import Adafruit_BBIO.UART as UART


baud = 115200


def main():
    UART.setup("UART4")
    #ttyS4 board specific or how determined?
    with serial.Serial(port="/dev/ttyS4", baudrate=baud) as ser:
        ser.close()
        ser.open()
        while 1:
            try:
                ser.write(int.to_bytes(-100, 1, byteorder='little', signed=True))
            except ser.SerialTimeoutException:
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