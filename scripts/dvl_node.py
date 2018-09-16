#!/usr/bin/env python
"""
===================================
Doppler velocity log interface node
===================================
This script creates a ROS node which published to the /dvl topic.
"""

from __future__ import print_function, division
import serial, time
# Can be Downloaded from this Link
# https://pypi.python.org/pypi/pyserial
import rospy
from zoidberg_ros.msg import DVL

class DVLNode:
    """
    Persistant object to handle serial comunications with DVL over serial
    """
    def __init__(self):
        """
        Function to Initialize the Serial Port
        """
        ser = serial.Serial()
        ser.baudrate = 115200
        ser.port = '/dev/serial/by-id/usb-FTDI_US232R_FT0TFKDN-if00-port0'
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        self.errNum = 9999

        # Specify the TimeOut in seconds, so that SerialPort doesn't hang
        ser.timeout = 1
        ser.open()  # Opens SerialPort
        self.ser = ser  # make serial port persistant
        self.pub = rospy.Publisher('dvl', DVL, queue_size=10)
        self.rate = None


    def init_dvl(self):
        """Send startup message to dvl over serial port"""
        is_start = False
        # break and then wait before sending start message
        self.ser.send_break(duration=1)
        time.sleep(5)
        # send start message
        self.ser.write(bytes(b'start\r\n'))
        time.sleep(5)
        # flush out the buffer
        print(self.ser.read_all())
        # check that first message is valid
        outputCheck = self.ser.readline()
        print(outputCheck)
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
        print(outputCheck)

        raise(IOError('Communication not initialized'))


    def publish_dvl(self):
        """Start the publishing of dvl messages"""
        rospy.init_node('dvl_node')
        self.rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            lineRead = self.ser.readline()
            splitLine = lineRead.split(bytes(b','))
            if not splitLine[0] == b'$DVLNAV': #if string is not DVLNAV go back and read next line
                continue
            Vx = float(splitLine[4]) if splitLine[4] else self.errNum
            Vy = float(splitLine[5]) if splitLine[5] else self.errNum
            Vz = float(splitLine[6]) if splitLine[6] else self.errNum
            # no error codes for x,y & alt coordinates, reading starts at 0
            xCoord = float(splitLine[7])
            yCoord = float(splitLine[8])
            altCoord = float(splitLine[9])
            depthCoord = float(splitLine[10]) if splitLine[10] else self.errNum
            msg = DVL(x_velocity=Vx,
                      y_velocity=Vy,
                      z_velocity=Vz,
                      x_position=xCoord,
                      y_position=yCoord,
                      altitude=altCoord)
            # flush any unread messages from buffer
            self.ser.read_all()
            # publish current dvl reading
            self.pub.publish(msg)
            #rospy.loginfo(msg)
            self.rate.sleep()
        self.close()

    def close(self):
        """Close the serial port"""
        self.ser.send_break(1)
        self.ser.write(bytes(b'stop\r\n'))
        self.ser.close()


# Call the Serial Initilization Function, Main Program Starts from here
if __name__ == '__main__':
    dvl_connection = DVLNode()
    try:
        dvl_connection.init_dvl()
        dvl_connection.publish_dvl()
    except IOError as err:
        rospy.logerr('Can not connect with DVL')
    except rospy.ROSInterruptException:
        pass
    # ensure that the COMM port is closed finally: dvl_connection.close()
