import serial
#import lcm
#from rover_msgs import AutonState, NavStatus
import Adafruit_BBIO.UART as UART
import time

#TODO
'''
functions for setting and reading individual registers? readability, style
correct sleep time and timing invariants
program control flow
error handling
LCM integration
'''
#defines
baud = 115200
uartPort = "/dev/ttyS4"

def serialWrite(command, data):
    with serial.Serial(port= uartPort, baudrate=baud) as ser:       #matching serial declaration
        buf = ((data <<8) | (0xFF & command))                       #create a 2 byte buffer to send 
        buf = int.to_bytes(buf, 2, byteorder='little', signed=True) #convert to 2 byte Little Endian
        ser.write(buf)
    
def serialRead(command):
    with serial.Serial(port= uartPort, baudrate=baud) as ser:            #matching serial declaration
        buf = int.to_bytes(command, 1, byteorder= 'little', signed=True) #create input byte buffer
        return ser.read(buf)                                             #return read value


def main():
    UART.setup("UART4")
    with serial.Serial(port= uartPort, baudrate=baud) as ser:
        ser.close()
        ser.open()
        while 1:
            try:
                serialWrite(-101, 70) #write 70 to 101
            #Serial port exception
            except ser.SerialTimeoutException:
                print("Serial is not open")
            
            time.sleep(0.001) #small delay TODO
 #       while(True):
 #           lcm_.handle()

if __name__ == "__main__":
    main()
    