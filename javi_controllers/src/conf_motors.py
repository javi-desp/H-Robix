#!/usr/bin/env python3

import rospy
import time
import signal
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#TODO check if this works
class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self, *args):
    self.kill_now = True


#TODO -> read some topic and change pwm limit, spped profile and acc profile
class set_motor_settings:
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        self.BAUDRATE                    = 57600             #default baudrate
        self.DEVICENAME                  = '/dev/ttyUSB1'
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Control table address
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        #self.ADDR_GOAL_POSITION      = 116
        #self.ADDR_PRESENT_POSITION   = 132
        #self.ADDR_LIMIT_VEL = 44
        #self.VEL_LIMIT = 155
        self.GOAL_VELOCITY = 112
        self.GOAL_ACCELERATION = 108
        self.ADDR_PWM_LIMIT = 36

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        #self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        #self.DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        #self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        self.COMM_SUCCESS                = 0

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids
        
        self.open_port()

    def open_port(self):
        #open port to be able to enable motor torque
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()

    def reboot_motor_group(self, array_ids):
        for id in range (6):
            print("See the Dynamixel LED flickering")

            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, array_ids[id])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] reboot Succeeded\n" % array_ids[id])

    def torque_motor_group(self, array_ids, state):
        for id in range (6):
            # Enable Dynamixel Torque
            dxl_comm_result1, dxl_error1 = self.packetHandler.write1ByteTxRx(self.portHandler, array_ids[id], self.ADDR_TORQUE_ENABLE, state)

            if dxl_comm_result1 != self.COMM_SUCCESS :
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
                quit()
            elif dxl_error1 != 0 :
                print("%s" % self.packetHandler.getRxPacketError(dxl_error1))
                quit()
            
        print("Torque enabled - DYNAMIXEL motor group with ids =  ",array_ids)
        
    def establish_motor_speed_profile(self, array_ids, velocity):
        for id in range (6):
            # Enable Dynamixel Torque
            dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, array_ids[id], self.GOAL_VELOCITY, velocity)

            if dxl_comm_result1 != self.COMM_SUCCESS :
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
                quit()
            elif dxl_error1 != 0 :
                print("%s" % self.packetHandler.getRxPacketError(dxl_error1))
                quit()

        print( "Velocity changed - DYNAMIXEL motor group with ids =  ",array_ids,)

    def establish_motor_acceleration_profile(self, array_ids, acceleration):
        for id in range (6):
            # Enable Dynamixel Torque
            dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, array_ids[id], self.GOAL_ACCELERATION, acceleration)

            if dxl_comm_result1 != self.COMM_SUCCESS :
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
                quit()
            elif dxl_error1 != 0 :
                print("%s" % self.packetHandler.getRxPacketError(dxl_error1))
                quit()
            
        print( "Acceleration changed - DYNAMIXEL motor group with ids =  ",array_ids,)

    def establish_pwm_limit_value(self, array_ids, pwm_limit):
        for id in range (6):
            # Enable Dynamixel Torque
            dxl_comm_result1, dxl_error1 = self.packetHandler.write2ByteTxRx(self.portHandler, array_ids[id], self.ADDR_PWM_LIMIT, pwm_limit)

            if dxl_comm_result1 != self.COMM_SUCCESS :
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
                quit()
            elif dxl_error1 != 0 :
                print("%s" % self.packetHandler.getRxPacketError(dxl_error1))
                quit()
            
        print( "Acceleration changed - DYNAMIXEL motor group with ids =  ",array_ids,)

    def default_configuration(self):
        self.reboot_motor_group(self.array_tibia_ids)
        self.reboot_motor_group(self.array_femur_ids)
        self.reboot_motor_group(self.array_coxa_ids)

        self.torque_motor_group(self.array_tibia_ids,1)
        self.torque_motor_group(self.array_femur_ids,1)
        self.torque_motor_group(self.array_coxa_ids,1)

        self.establish_motor_speed_profile(self.array_tibia_ids,10)
        self.establish_motor_speed_profile(self.array_femur_ids,10)
        self.establish_motor_speed_profile(self.array_coxa_ids,10)

        self.establish_motor_acceleration_profile(self.array_tibia_ids,20)
        self.establish_motor_acceleration_profile(self.array_femur_ids,20)
        self.establish_motor_acceleration_profile(self.array_coxa_ids,20)

        self.establish_pwm_limit_value(self.array_coxa_ids,400)
        self.establish_pwm_limit_value(self.array_coxa_ids,400)
        self.establish_pwm_limit_value(self.array_coxa_ids,400)

    def disable_motors(self):
        self.torque_motor_group(self.array_tibia_ids,1)
        self.torque_motor_group(self.array_femur_ids,1)
        self.torque_motor_group(self.array_coxa_ids,1)


def main():
    #TODO read from json conf file 
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]

    motors = set_motor_settings(ID_tibia, ID_femur, ID_coxa)
    motors.default_configuration()

    killer = GracefulKiller()
    while not killer.kill_now:
        time.sleep(0.5)
    motors.disable_motors()

if __name__ == '__main__':
    main()

