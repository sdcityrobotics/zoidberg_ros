import serial, time

# Can be Downloaded from this Link
# https://pypi.python.org/pypi/pyserial

# Global Variables
# from numpy.doc.constants import new_lines
from zoidberg_nav.msg import dvl

# Function to Initialize the Serial Port
def init_serial():
    global ser  # Must be declared in Each Function
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = 'COM4'  # COM Port Name Start from 0
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE

    # Specify the TimeOut in seconds, so that SerialPort
    # Doesn't hangs
    ser.timeout = 10
    ser.open()  # Opens SerialPort
    # if ser.is_open():
    #    print("Port is now open")
    # else:
    #    print("Error: Port did not open")
    # print port open or closed

def init_dvl():
    dvlLoopCnt = 0
    print("dvl startup loop")
    while 1:
        if dvlLoopCnt == 10:
            print("error, dvl not started")
            break
        print("loop number")
        print(dvlLoopCnt)
        ser.send_break(duration=1)
        time.sleep(2)
        print(ser.read_all())
        ser.write(bytes(b'start\r\n'))
        time.sleep(2)
        outputCheck = ser.readline()
        splitLine = outputCheck.split(bytes(b','))
        if splitLine[0] != b'$DVLNAV':
            break


# Function Ends Here


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

    if splitLine[0] == b'$DVLNAV':
        print(splitLine)
        # get all variables, if no reading use 0.0 for now, maybe change to 9999 to indicate error
        if splitLine[4]:
            Vx = float(splitLine[4])
        else:
            Vx = errNum
        if splitLine[5]:
            Vy = float(splitLine[5])
        else:
            Vy = errNum
        if splitLine[6]:
            Vz = float(splitLine[6])
        else:
            Vz = errNum
        xCoord = float(splitLine[7]) # no error codes for x,y & alt coordinates, reading starts at 0
        yCoord = float(splitLine[8])
        altCoord = float(splitLine[9])
        if splitLine[10]:
            depthCoord = float(splitLine[10])
        else:
            depthCoord = errNum
        # now lets print all data to verify all variables are properly saved
	curr_msg = dvl
	curr_msg.x_velocity = Vx

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
