#!/usr/bin/env python3

import rospy
import time
import math
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class service_to_joint_state: 
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids

        rospy.init_node('read_motor_data')
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(100) # 100hz

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
            try:
                self.tibia_group_current_position = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                        self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"position")

                self.femur_group_current_position = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                        self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"position")

                self.coxa_group_current_position = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                        self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"position")
            except: 
                pass
            if (self.tibia_group_current_position == False or self.femur_group_current_position == False or self.coxa_group_current_position == False):
                print("couldnt update position!!")
            else:
                self.update_joint_state_message()

                self.message_joint_state.header.stamp = rospy.Time.now()
                self.pub_joint_state.publish(self.message_joint_state)
                self.rate.sleep()


def main():
    #TODO this only reads crr pos ant send it ---> it can be done in server side
    #TODO read from json conf file 
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]

    hexapod = service_to_joint_state(ID_tibia, ID_femur, ID_coxa)
    hexapod.run()

if __name__ == '__main__':
    main()

