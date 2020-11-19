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

#######################################################################

#write commands
#   registers -102 through -113 PID ... see setP, setI, setD functions
#   registers -116 through -127 Not Implemented as of 11/18/20

# RPM valid inputs: -100 to 100. driver must be enabled
RPM_write = -101

#current unsigned 8-bit 0 to 255 in deciamps. 
#if exceeded fault reg overcurrent set to 1, enters disabled state 
current_limit_write = -114

#enable by writing 8-bit unsigned non zero value
#disable by writing 0. RPM register will be reset to 0
enable_write = -115

#any write -> brake enabled, RPM set to 0
#to disable brake, write new target RPM
brake_write = -128

#######################################################################

#read commands
#   registers 102 through 113 PID ... see readP, readI, readD functions
#   registers 118 through 127 Not Implemented as of 11/18/20

#read last set RPM. -100 to 100 8-bit signed 
set_RPM_read = 101 

#read last set current limit. 0 to 255 deciamps 8-bit unsigned
set_current_limit_read = 114

#read measured current. 0 to 255 deciamps 8-bit unsigned
measured_current_read = 115

#read fault status. REGISTER CLEARED UPON READ
#bit0 -> Bad Packet         ... faulty packet recieved
#bit1 -> Invalid Velocity   ...invalid RPM set
#bit2 -> Overcurrent        ...current limit exceeded
#bit3 -> Watchdog           ...purpose?
#bit4 -> wiring             ...a motor or encoder wire disconnected 
#bit5-7 Not Implemented (always 0) as of 11/18/20
fault_read = 116

#measured RPM -100 to 100. 8-bit signed  
measured_PRM_read = 117

#######################################################################
#defines
baud = 115200
uartPort = "/dev/ttyS4"


#write function. takes in command and data to be written
#inputs:    command: command for register to be written to 
#                   valid inputs: -101 to -115, -128 brake
#           data: 8-bit value 
#returns:   N/A
def serialWrite(command, data):
    with serial.Serial(port= uartPort, baudrate=baud) as ser:       #matching serial declaration
        buf = ((data <<8) | (0xFF & command))                       #create a 2 byte buffer to send 
        buf = int.to_bytes(buf, 2, byteorder='little', signed=True) #convert to 2 byte Little Endian
        ser.write(buf)

#read function which reads register specified by command
#input:    command: command for register to be written to 
#   valid inputs: 101 to 117
#returns:   byte that was read
def serialRead(command):
    with serial.Serial(port= uartPort, baudrate=baud) as ser:            #matching serial declaration
        buf = int.to_bytes(command, 1, byteorder= 'little', signed=True) #create input byte buffer
        return ser.read(buf)                                             #return read value


#sets P to target P
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setP(targetP):
    target0 = (targetP & 0xFF)
    target1 = (targetP & 0xFF << 8) >> 8
    target2 = (targetP & 0xFF << 16) >> 16
    target3 = (targetP & 0xFF << 24) >> 24
    serialWrite(-102, target0)
    serialWrite(-103, target1)
    serialWrite(-104, target2)
    serialWrite(-105, target3)

#sets I to target I
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setI(targetI):
    target0 = (targetI & 0xFF)
    target1 = (targetI & 0xFF << 8) >> 8
    target2 = (targetI & 0xFF << 16) >> 16
    target3 = (targetI & 0xFF << 24) >> 24
    serialWrite(-106, target0)
    serialWrite(-107, target1)
    serialWrite(-108, target2)
    serialWrite(-109, target3)

#sets D to target D
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setD(targetD):
    target0 = (targetD & 0xFF)
    target1 = (targetD & 0xFF << 8) >> 8
    target2 = (targetD & 0xFF << 16) >> 16
    target3 = (targetD & 0xFF << 24) >> 24
    serialWrite(-110, target0)
    serialWrite(-111, target1)
    serialWrite(-112, target2)
    serialWrite(-113, target3)

#reads last set P
#returns 32 bit float
def readP():
    last0 = serialRead(102)
    last1 = serialRead(103)
    last2 = serialRead(104)
    last3 = serialRead(105)
    return (last3 << 24) | (last2 << 16) | (last1 << 8) | (last0) 

#reads last set I
#returns 32 bit float
def readI():
    last0 = serialRead(106)
    last1 = serialRead(107)
    last2 = serialRead(108)
    last3 = serialRead(109)
    return (last3 << 24) | (last2 << 16) | (last1 << 8) | (last0) 

#reads last set D
#returns 32 bit float
def readD():
    last0 = serialRead(110)
    last1 = serialRead(111)
    last2 = serialRead(112)
    last3 = serialRead(113)
    return (last3 << 24) | (last2 << 16) | (last1 << 8) | (last0) 

def main():
    UART.setup("UART4")
    with serial.Serial(port= uartPort, baudrate=baud) as ser:
        try:
            ser.close()
            ser.open()
            #TODO set PID values before active
            #write positive value to enable reg
            serialWrite(enable_write, 3)
            #set current limit to 25 deciAmps (TODO should this be after EN)
            serialWrite(current_limit_write, 25)
            #set RPM value
            serialWrite(RPM_write, 10)
            print(serialRead(set_RPM_read))
            print(serialRead(set_current_limit_read))
            print(serialRead(measured_current_read))
            print(serialRead(measured_RPM_read))
            print(serialRead(readP))
            print(serialRead(readI))
            print(serialRead(readD))
        except ser.SerialTimeoutException:
            print("Serial is not open")
        time.sleep(0.001) #small delay TODO correct timing

        '''
        while 1:
            try:
                serialWrite(-101, 70) #write 70 to 101
            #Serial port exception
            except ser.SerialTimeoutException:
                print("Serial is not open")
        '''    
 #       while(True):
 #           lcm_.handle()

if __name__ == "__main__":
    main()
    