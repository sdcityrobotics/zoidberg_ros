"""
===================================
Doppler velocity log interface node
===================================
This script creates a ROS node which published to the /dvl topic.
"""

import serial, time
# Can be Downloaded from this Link
# https://pypi.python.org/pypi/pyserial
import rospy
from zoidberg_nav.msg import DVL

class DVL:
    """
    Persistant object to handle serial comunications with DVL over serial
    """
    def __init__(self):
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
        ser.timeout = 1
        ser.open()  # Opens SerialPort
        self.ser = ser  # make serial port persistant
        self.pub = rospy.Publisher('dvl', DVL, queue_size=10)
        self.rate = rospy.rate(10)  # 10 Hz


    def init_dvl(self):
        """Send startup message to dvl over serial port"""
        is_start = False
        # break and then wait before sending start message
        self.ser.send_break(duration=1)
        time.sleep(1)
        # send start message
        self.ser.write(bytes(b'start\r\n'))
        time.sleep(1)
        # flush out the buffer
        self.ser.flush()
        # check that first message is valid
        outputCheck = self.ser.readline()
        splitLine = outputCheck.split(bytes(b','))
        if splitLine[0] == b'$DVLNAV':
            is_start=True
            return
        # check that second message is valid
        outputCheck = self.ser.readline()
        splitLine = outputCheck.split(bytes(b','))
        if splitLine[0] == b'$DVLNAV':
            is_start=True
            return
        # if the first two lines don't match raise error

        raise(ConnectionError('Communication not initialized'))


    def publish_dvl(self):
        """Start the publishing of dvl messages"""
        # This should fill in similarly to talker() in:
        # http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
        rospy.init_node('dvl_node')
        while not rospy.is_shutdown():
            pass

    def close(self):
        """Close the serial port"""
        self.ser.send_break(1)
        self.ser.write(bytes(b'stop\r\n'))
        self.ser.close()


# Call the Serial Initilization Function, Main Program Starts from here
if __name__ == '__main__':
    dvl_connection = DVL()
    try:
        dvl_connection.init_dvl()
        dvl_connection.publish_dvl()
    except ConnectionError as err:
        rospy.logerror('Can not connect with DVL')
    except rospy.ROSInterruptException:
        pass
    # ensure that the COMM port is closed
    finally:
        dvl_connection.close()

1/0
# This stuff will be moved into the publish_dvl method

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
