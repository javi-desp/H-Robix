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



def create_topics_and_services(rospy, id):

    def set_goal_pos_callback(data):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, data.position)

    def get_present_pos(req):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION)
        return dxl_present_position

    rospy.Subscriber('set_position_tibia_'+ str(id), SetPosition, set_goal_pos_callback)
    rospy.Service('get_position_tibia_'+ str(id), GetPosition, get_present_pos)

    def set_goal_pos_callback(data):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, int(str(id) + "0"), ADDR_GOAL_POSITION, data.position)

    def get_present_pos(req):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, int(str(id) + "0"), ADDR_PRESENT_POSITION)
        return dxl_present_position

    rospy.Subscriber('set_position_coxa_'+ str(id), SetPosition, set_goal_pos_callback)
    rospy.Service('get_position_coxa_'+ str(id), GetPosition, get_present_pos)

    def set_goal_pos_callback(data):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, int( str(id) + "1"), ADDR_GOAL_POSITION, data.position)

    def get_present_pos(req):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, int( str(id) + "1"), ADDR_PRESENT_POSITION)
        return dxl_present_position

    rospy.Subscriber('set_position_femur_'+ str(id), SetPosition, set_goal_pos_callback, )
    rospy.Service('get_position_femur_'+ str(id), GetPosition, get_present_pos)

def enable_arm(rospy, DXL_ID_arm):
    # Enable Dynamixel Torque
    dxl_comm_result1, dxl_error1 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_arm[0], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result2, dxl_error2 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_arm[1], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result3, dxl_error3 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_arm[2], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    if dxl_comm_result1 != COMM_SUCCESS or dxl_comm_result2 != COMM_SUCCESS or dxl_comm_result3 != COMM_SUCCESS :
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error1 != 0 or dxl_error2 != 0 or dxl_error3 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))
        print("%s" % packetHandler.getRxPacketError(dxl_error2))
        print("%s" % packetHandler.getRxPacketError(dxl_error3))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL arm ",DXL_ID_arm[0]," has been successfully connected")
        print("Ready to get & set Position.")

        create_topics_and_services(rospy, DXL_ID_arm[0])

def main():
    # Default setting
    DXL_ID_arm_1                = [1, 10, 11]                 # Dynamixel ARM ID
    DXL_ID_arm_2                = [2, 20, 21]                 # Dynamixel ARM ID
    DXL_ID_arm_3                = [3, 30, 31]                 # Dynamixel ARM ID
    DXL_ID_arm_4                = [4, 40, 41]                 # Dynamixel ARM ID
    DXL_ID_arm_5                = [5, 50, 51]                 # Dynamixel ARM ID
    DXL_ID_arm_6                = [6, 60, 61]                 # Dynamixel ARM ID

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
    enable_arm(rospy, DXL_ID_arm_1)
    #enable_arm(rospy, DXL_ID_arm_2)
    #enable_arm(rospy, DXL_ID_arm_3)
    #enable_arm(rospy, DXL_ID_arm_4)
    #enable_arm(rospy, DXL_ID_arm_5)
    #enable_arm(rospy, DXL_ID_arm_6)

    rospy.spin()


if __name__ == '__main__':
    main()

