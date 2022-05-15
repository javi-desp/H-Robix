import os
import rospy
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_LIMIT_VEL = 44
VEL_LIMIT = 155

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel



BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)



def create_topics_and_services(id):


    def set_goal_pos_callback(data):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, int( str(id) + "0"), ADDR_GOAL_POSITION, data.position)

    def get_present_pos(req):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, int( str(id) + "0"), ADDR_PRESENT_POSITION)
        return dxl_present_position

    rospy.Subscriber('set_position_coxa_'+ str(id), SetPosition, set_goal_pos_callback, )
    rospy.Service('get_position_coxa_'+ str(id), GetPosition, get_present_pos)

def enable_arm():
    for id in range (1,7):
        # Enable Dynamixel Torque
        dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, int(str(id) + "0"), ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        if dxl_comm_result1 != COMM_SUCCESS :
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))

            print("Press any key to terminate...")
            print("se")
            getch()
            quit()
        elif dxl_error1 != 0 :
            print("%s" % packetHandler.getRxPacketError(dxl_error1))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print("DYNAMIXEL arm ",id," has been successfully connected")
            print("Ready to get & set Position.")

            create_topics_and_services(id)

def main():
    # Default setting
    DXL_ID_arm_1                = 1                 # Dynamixel ARM ID
    DXL_ID_arm_2                = 2                 # Dynamixel ARM ID
    DXL_ID_arm_3                = 3                 # Dynamixel ARM ID
    DXL_ID_arm_4                = 4                 # Dynamixel ARM ID
    DXL_ID_arm_5                = 5                 # Dynamixel ARM ID
    DXL_ID_arm_6                = 6                 # Dynamixel ARM ID

    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    rospy.init_node('read_write_py_node')
    enable_arm()

    rospy.spin()


if __name__ == '__main__':
    main()

