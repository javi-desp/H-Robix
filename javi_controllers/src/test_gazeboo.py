#!/usr/bin/env python3

import rospy
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import math

class service_to_joint_state: 
    def __init__(self):
        rospy.init_node('pub_motor_data')
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
    

    def run(self):
        while not rospy.is_shutdown():
            start = time.time()        
            self.message_joint_state.position[0] += 0
            self.message_joint_state.position[6] += 0 
            self.message_joint_state.position[12] += 0 
            self.message_joint_state.header.stamp = rospy.Time.now()
            self.pub_joint_state.publish(self.message_joint_state)
            self.rate.sleep()

            end = time.time()
            print("RESPONSE IN ", end-start)

def main():
    hexapod = service_to_joint_state()

    hexapod.run()

if __name__ == '__main__':
    main()

