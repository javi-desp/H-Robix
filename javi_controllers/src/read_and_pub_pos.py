#!/usr/bin/env python3

import rospy
import time
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import math


class set_motor_settings:
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        self.BAUDRATE                    = 57600             #default baudrate
        self.DEVICENAME                  = '/dev/ttyUSB0'
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Control table address
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132
        self.ADDR_LIMIT_VEL = 44
        self.VEL_LIMIT = 155
        self.GOAL_VELOCITY = 112
        self.GOAL_ACCELERATION = 108

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        self.COMM_SUCCESS                = 0

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids
        
        self.open_port()
        self.torque_motor_group(self.array_tibia_ids,1)
        self.torque_motor_group(self.array_femur_ids,1)
        self.torque_motor_group(self.array_coxa_ids,1)

        self.establish_motor_speed_profile(self.array_tibia_ids,10)
        self.establish_motor_speed_profile(self.array_femur_ids,10)
        self.establish_motor_speed_profile(self.array_coxa_ids,10)

        self.establish_motor_acceleration_profile(self.array_tibia_ids,20)
        self.establish_motor_acceleration_profile(self.array_femur_ids,20)
        self.establish_motor_acceleration_profile(self.array_coxa_ids,20)


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
        
    def set_msg_motor(array_ids,tibia_group_current_position, mode, init_data):

        set_position_tibia_group = SetGroupMotorData()
        print(set_position_tibia_group)
        set_position_tibia_group.data_required = mode

        set_position_tibia_group.motor1_id = array_ids[0]
        set_position_tibia_group.motor2_id = array_ids[1]
        set_position_tibia_group.motor3_id = array_ids[2]
        set_position_tibia_group.motor4_id = array_ids[3]
        set_position_tibia_group.motor5_id = array_ids[4]
        set_position_tibia_group.motor6_id = array_ids[5]

        set_position_tibia_group.motor1_data = tibia_group_current_position.motor1_data + int((init_data - tibia_group_current_position.motor1_data)/4)
        set_position_tibia_group.motor2_data = tibia_group_current_position.motor2_data + int((init_data - tibia_group_current_position.motor2_data)/4)
        set_position_tibia_group.motor3_data = tibia_group_current_position.motor3_data + int((init_data - tibia_group_current_position.motor3_data)/4)
        set_position_tibia_group.motor4_data = tibia_group_current_position.motor4_data + int((init_data - tibia_group_current_position.motor4_data)/4)
        set_position_tibia_group.motor5_data = tibia_group_current_position.motor5_data + int((init_data - tibia_group_current_position.motor5_data)/4)
        set_position_tibia_group.motor6_data = tibia_group_current_position.motor6_data + int((init_data - tibia_group_current_position.motor6_data)/4)

        return set_position_tibia_group


class service_to_joint_state: 
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        rospy.init_node('pub_motor_data')
        self.leg_lenghts = {'coxa': 0.0, 'femur': 0.1, 'tibia': 0.15049059936089032}
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(100) # 50hz

        self.message_joint_state = JointState()
        self.message_joint_state.header = Header()
        self.message_joint_state.name = ['tibia_joint_LB', 'tibia_joint_LM', 'tibia_joint_LF', 
                        'tibia_joint_RB', 'tibia_joint_RM', 'tibia_joint_RF', 
                        'femur_joint_LB', 'femur_joint_LM', 'femur_joint_LF', 
                        'femur_joint_RB', 'femur_joint_RM', 'femur_joint_RF', 
                        'coxa_joint_LB', 'coxa_joint_LM', 'coxa_joint_LF', 
                        'coxa_joint_RB', 'coxa_joint_RM', 'coxa_joint_RF']
        self.message_joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.message_joint_state.velocity = []
        self.message_joint_state.effort = []
    
        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids

        rospy.wait_for_service('get_dinamixel_motor_group_data')
        self.motor_group_service = rospy.ServiceProxy('get_dinamixel_motor_group_data', GetGroupMotorData)

    def update_joint_state_message(self):
        self.message_joint_state.position = [
                self.tibia_group_current_position.motor1_data*2*(math.pi/4095) - math.pi,
                self.tibia_group_current_position.motor2_data*2*math.pi/4095 - math.pi,
                self.tibia_group_current_position.motor3_data*2*math.pi/4095 - math.pi,
                -self.tibia_group_current_position.motor4_data*2*math.pi/4095 + math.pi,
                -self.tibia_group_current_position.motor5_data*2*math.pi/4095 + math.pi,
                -self.tibia_group_current_position.motor6_data*2*math.pi/4095 + math.pi, 

                -self.femur_group_current_position.motor1_data*2*math.pi/4095 + math.pi,
                -self.femur_group_current_position.motor2_data*2*math.pi/4095 + math.pi,
                -self.femur_group_current_position.motor3_data*2*math.pi/4095 + math.pi,
                self.femur_group_current_position.motor4_data*2*math.pi/4095 - math.pi,
                self.femur_group_current_position.motor5_data*2*math.pi/4095 - math.pi,
                self.femur_group_current_position.motor6_data*2*math.pi/4095 - math.pi,  

                -self.coxa_group_current_position.motor1_data*2*math.pi/4095 + math.pi,
                -self.coxa_group_current_position.motor2_data*2*math.pi/4095 + math.pi,
                -self.coxa_group_current_position.motor3_data*2*math.pi/4095 + math.pi,
                -self.coxa_group_current_position.motor4_data*2*math.pi/4095 + math.pi,
                -self.coxa_group_current_position.motor5_data*2*math.pi/4095 + math.pi,
                -self.coxa_group_current_position.motor6_data*2*math.pi/4095 + math.pi
        ]

    def run(self):
        while not rospy.is_shutdown():
            start = time.time()        
            
            self.tibia_group_current_position = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                    self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"position")

            self.femur_group_current_position = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                    self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"position")

            self.coxa_group_current_position = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                    self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"position")


            self.update_joint_state_message()

            self.message_joint_state.header.stamp = rospy.Time.now()
            self.pub_joint_state.publish(self.message_joint_state)
            self.rate.sleep()


def main():
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]

    motors = set_motor_settings()

    hexapod = service_to_joint_state(ID_tibia, ID_femur, ID_coxa)

    hexapod.run()

if __name__ == '__main__':
    main()

