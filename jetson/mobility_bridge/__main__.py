import serial
#import lcm
from rover_msgs import AutonState, NavStatus
import Adafruit_BBIO.UART as UART


baud = 115200


def main():
    UART.setup("UART4")
    #ttyS4 board specific or how determined?
    with serial.Serial(port="/dev/ttyS4", baudrate=baud) as ser:
        ser.close()
        ser.open()
        try:
            ser.write("A".encode('utf-8'))
        except ser.SerialTimeoutException:
            print("Serial is not open")
 #       while(True):
 #           lcm_.handle()

if __name__ == "__main__":
    main()