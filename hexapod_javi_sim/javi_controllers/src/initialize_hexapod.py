#!/usr/bin/env python3

import rospy
import time
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import math


BAUDRATE                    = 57600             #default baudrate
DEVICENAME                  = '/dev/ttyUSB0'
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_LIMIT_VEL = 44
VEL_LIMIT = 155


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class init_motors: 
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        rospy.init_node('set_initial_motor_position')
        self.pub_to_motor_group = rospy.Publisher('set_dinamixel_motor_group_data', SetGroupMotorData, queue_size=10)
        self.rate = rospy.Rate(20) # 50hz

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids

        self.set_position_tibia_group = SetGroupMotorData()
        self.set_position_femur_group = SetGroupMotorData() 
        self.set_position_coxa_group = SetGroupMotorData()

        self.set_position_tibia_group = self.init_msg_motor(self.array_tibia_ids, self.set_position_tibia_group, "position", 3200)
        self.set_position_femur_group = self.init_msg_motor(self.array_femur_ids, self.set_position_femur_group, "position", 1375)
        self.set_position_coxa_group = self.init_msg_motor(self.array_coxa_ids, self.set_position_coxa_group, "position", 2047)

        self.open_port()
        self.enable_motor_group(self.array_tibia_ids)
        self.enable_motor_group(self.array_femur_ids)
        self.enable_motor_group(self.array_coxa_ids)

        rospy.wait_for_service('get_dinamixel_motor_group_data')
        self.motor_group_service = rospy.ServiceProxy('get_dinamixel_motor_group_data', GetGroupMotorData)

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
            self.portHandler.setBaudRate(BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()

    def enable_motor_group(self, array_ids):
        for id in range (6):
            # Enable Dynamixel Torque
            dxl_comm_result1, dxl_error1 = self.packetHandler.write1ByteTxRx(self.portHandler, array_ids[id], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

            if dxl_comm_result1 != COMM_SUCCESS :
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result1))
                quit()
            elif dxl_error1 != 0 :
                print("%s" % self.packetHandler.getRxPacketError(dxl_error1))
                quit()
            
        print("DYNAMIXEL motor group with ids =  ",array_ids, "torque enabled ")

    def init_msg_motor(self, array_ids, set_position_motor_group, mode, init_data):
        print(set_position_motor_group)
        set_position_motor_group.data_required = mode

        set_position_motor_group.motor1_id = array_ids[0]
        set_position_motor_group.motor2_id = array_ids[1]
        set_position_motor_group.motor3_id = array_ids[2]
        set_position_motor_group.motor4_id = array_ids[3]
        set_position_motor_group.motor5_id = array_ids[4]
        set_position_motor_group.motor6_id = array_ids[5]

        set_position_motor_group.motor1_data = init_data
        set_position_motor_group.motor2_data = init_data
        set_position_motor_group.motor3_data = init_data
        set_position_motor_group.motor4_data = init_data
        set_position_motor_group.motor5_data = init_data
        set_position_motor_group.motor6_data = init_data

        return set_position_motor_group

    def compare_tor_group_with_msg(set_position_motor_group, motor_group_current_position):
        if (set_position_motor_group.motor1_data < motor_group_current_position.motor1_data + 20 and set_position_motor_group.motor1_data > motor_group_current_position.motor1_data - 20 and
        set_position_motor_group.motor2_data < motor_group_current_position.motor2_data + 20 and set_position_motor_group.motor2_data > motor_group_current_position.motor2_data - 20 and
        set_position_motor_group.motor3_data < motor_group_current_position.motor3_data + 20 and set_position_motor_group.motor3_data > motor_group_current_position.motor3_data - 20 and
        set_position_motor_group.motor4_data < motor_group_current_position.motor4_data + 20 and set_position_motor_group.motor4_data > motor_group_current_position.motor4_data - 20 and
        set_position_motor_group.motor5_data < motor_group_current_position.motor5_data + 20 and set_position_motor_group.motor5_data > motor_group_current_position.motor5_data - 20 and
        set_position_motor_group.motor6_data < motor_group_current_position.motor6_data + 20 and set_position_motor_group.motor6_data > motor_group_current_position.motor6_data - 20):
            return True
        else:
            return False

    def run_zero_position(self):
        #INTERPOLAR POSICION ACTUAL CON OBJETIVO 0 --> T=3200, F=1375,C=2047

        while True:
            self.rate.sleep()

            self.pub_to_motor_group.publish(self.set_position_tibia_group)
            self.pub_to_motor_group.publish(self.set_position_femur_group)
            self.pub_to_motor_group.publish(self.set_position_coxa_group)
            print("YES")

            self.tibia_group_current_position = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                    self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"position")

            self.femur_group_current_position = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                    self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"position")

            self.coxa_group_current_position = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                    self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"position")

            if (self.compare_tor_group_with_msg(self.set_position_tibia_group, self.tibia_group_current_position) and 
                self.compare_tor_group_with_msg(self.set_position_femur_group, self.femur_group_current_position) and 
                self.compare_tor_group_with_msg(self.set_position_coxa_group, self.coxa_group_current_position)):
                print("HECHO")
                break
                

def main():
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]
    initialize = init_motors(ID_tibia, ID_femur, ID_coxa)

    initialize.run_zero_position()
    rospy.spin()

if __name__ == '__main__':
    main()

