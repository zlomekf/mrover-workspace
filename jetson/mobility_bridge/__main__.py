import serial
#import lcm
#from rover_msgs import AutonState, NavStatus
import Adafruit_BBIO.UART as UART
import time
import struct

#TODO
'''
timing invariants
functions for setting and reading individual registers? readability, style
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
measured_RPM_read = 117

#######################################################################
#defines
baud = 115200
uartPort = "/dev/ttyS4"

# configure the serial connections (set port yourself)
ser = serial.Serial(
    port=uartPort,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

#write function. takes in command and data to be written
#inputs:    command: command for register to be written to 
#                   valid inputs: -101 to -115, -128 brake
#           data: 8-bit value 
#returns:   N/A
def serialWrite(command, data):
        num = ((command <<8) | (0xFF & data))                       #create a 2 byte buffer to send 
        buf = int.to_bytes(num, 2, byteorder='big', signed=True) #send as 2 byte big Endian
        ser.write(buf)
        time.sleep(0.01)


#read function which reads register specified by command
#input:    command: command for register to be written to 
#   valid inputs: 101 to 117
#returns:   byte that was read
def serialRead(command):
        serialWrite(command, 0)                                          #write command
        rcvd = ser.read()
        time.sleep(0.01)
        return rcvd                                               #return read value


#sets P to target P
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setP(targetP):
    ba = bytearray(struct.pack("<f", targetP))
    serialWrite(-102, ba[0])
    serialWrite(-103, ba[1])
    serialWrite(-104, ba[2])
    serialWrite(-105, ba[3])

#sets I to target I
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setI(targetI):
    ba = bytearray(struct.pack("<f", targetI))
    serialWrite(-106, ba[0])
    serialWrite(-107, ba[1])
    serialWrite(-108, ba[2])
    serialWrite(-109, ba[3])

#sets D to target D
#input:  32 bit float
# DRIVER MUST BE DISABLED
def setD(targetD):
    ba = bytearray(struct.pack("<f", targetD))
    serialWrite(-110, ba[0])
    serialWrite(-111, ba[1])
    serialWrite(-112, ba[2])
    serialWrite(-113, ba[3])

#reads last set P
#returns 32 bit float
def readP():
    last = bytearray(4)
    last[0] = (serialRead(102))
    last[1]= (serialRead(103))
    last[2] = (serialRead(104))
    last[3] = (serialRead(105))
    return struct.unpack('<f', last)

#reads last set I
#returns 32 bit float
def readI():
    last = bytearray(4)
    last[0] = (serialRead(106))
    last[1] = (serialRead(107))
    last[2] = (serialRead(108))
    last[3] = (serialRead(109))
    return struct.unpack('<f', last)

#reads last set D
#returns 32 bit float
def readD():
    last = bytearray(4)
    last[0] = (serialRead(110))
    last[1] = (serialRead(111))
    last[2] = (serialRead(112))
    last[3] = (serialRead(113))
    return struct.unpack('<f', last)

#set target 
#read target
#read actual velocity
#enable
#diable
def main():
    UART.setup("UART4")
    try:
        ser.close()
        ser.open()
        print("initial")
        #TODO set PID values before active??
        #setP(0.0)
        #setI(0.0)
        #setD(0.0)
        #write positive value to enable reg
        serialWrite(enable_write, 0)
        print("wrote enable")
        #set current limit to 25 deciAmps (TODO should this be after EN)
        #serialWrite(current_limit_write, 25)
        #print("wrote current")
        #set RPM value
        serialWrite(RPM_write, 10)
        print("wrote RPM")
        '''  
        print(int.from_bytes(serialRead(set_RPM_read), byteorder='big', signed=True))
        print("read 1")
        print(int.from_bytes(serialRead(set_current_limit_read), byteorder='big', signed=False))
        print("read 2")
        print(int.from_bytes(serialRead(measured_current_read), byteorder='big', signed=False))
        print("read 3")
        print(int.from_bytes(serialRead(fault_read), byteorder='big', signed=False))
        print("read 4")
        print(int.from_bytes(serialRead(measured_RPM_read), byteorder='big', signed=True))
        print("read 5")
        print(readP())
        print("read 6")
        print(readI())
        print("read 7")
        print(readD()) 
        '''
    except ser.SerialTimeoutException:
        print("Serial is not open")
    while 1:    
        try:
            serialWrite(RPM_write, 10)
        #Serial port exception
        except ser.SerialTimeoutException:
            print("Serial is not open")
        time.sleep(1)
        

if __name__ == "__main__":
    main()
    