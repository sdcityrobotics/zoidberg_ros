"""
===================================
Doppler velocity log interface node
===================================
This script creates a ROS node which published to the /dvl topic.
"""

import serial, time
# Can be Downloaded from this Link
# https://pypi.python.org/pypi/pyserial
from zoidberg_nav.msg import dvl

class DVL_Exception(Exception):
    """Custom exception class to indicate problems with the DVL"""
    def __init__(self, message, errors):
        """Pass along user defined error message"""
        super().__init__(message)
        self.errors = errors

class DVL:
    """
    Persistant object to handle serial comunications with DVL over serial
    """
    def __init__():
        """
        Function to Initialize the Serial Port
        """
        ser = serial.Serial()
        ser.baudrate = 115200
        ser.port = 'COM4'  # COM Port Name Start from 0
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE

        # Specify the TimeOut in seconds, so that SerialPort doesn't hang
        ser.timeout = 10
        ser.open()  # Opens SerialPort
        self.ser = ser  # make serial port persistant
        self._init_dvl()
        

    def _init_dvl(self):
        """Send startup message to dvl over serial port"""
        num_trys = 5
        is_start = False
        print("dvl startup loop")
        for i in range(num_trys):
            print("loop number")
            print(dvlLoopCnt)
            self.ser.send_break(duration=0.5)
            time.sleep(0.5)
            # flush out the buffer
            print(self.ser.read_all())
            # send start message
            self.ser.write(bytes(b'start\r\n'))
            time.sleep(0.5)
            # check that first message is valid
            outputCheck = self.ser.readline()
            splitLine = outputCheck.split(bytes(b','))
            if splitLine[0] != b'$DVLNAV':
                is_start = True
                break
        if not is_start:
            raise(DVL_Exception('Communication not initialized', IOError))



# Call the Serial Initilization Function, Main Program Starts from here

init_serial()
init_dvl()
errNum = float(9999)
cnt = 0
while ser.isOpen():
    cnt = cnt + 1
    if cnt > 10:
        ser.send_break(1)
        ser.write(bytes(b'stop\r\n'))
        ser.close()
        print("quitting")
        break
    lineRead = ser.readline()
    splitLine = lineRead.split(bytes(b','))

    if not splitLine[0] == b'$DVLNAV':
        continue
    print(splitLine)
    # get all variables, if no reading, indicate error with 9999
    Vx = float(splitLine[4]) if splitLine[4] else errNum
    Vy = float(splitLine[5]) if splitLine[5] else errNum
    Vz = float(splitLine[6]) if splitLine[6] else errNum
    # no error codes for x,y & alt coordinates, reading starts at 0
    xCoord = float(splitLine[7])
    yCoord = float(splitLine[8])
    altCoord = float(splitLine[9])
    depthCoord = float(splitLine[10]) if splitLine[10] else errNum

    # now lets print all data to verify all variables are properly saved
    print('\n' + "X Velocity: ")
    print(Vx)
    print('\n' + "Y Velocity: ")
    print(Vy)
    print('\n' + "Z Velocity: ")
    print(Vz)
    print('\n' + "X Coordinate: ")
    print(xCoord)
    print('\n' + "Y Coordinate: ")
    print(yCoord)
    print('\n' + "Altitude: ")
    print(altCoord)
    print('\n' + "Depth: ")
    print(depthCoord)
