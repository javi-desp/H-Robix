import os
import rospy
import time
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header



def main():
    pub = rospy.Publisher('joint_states', JointState, queue_size=2)
    rospy.init_node('joint_states_update')
    rate = rospy.Rate(100) # 100hz


    message_rviz = JointState()
    message_rviz.header = Header()
    message_rviz.name = ['tibia_joint_RF', 'tibia_joint_RM', 'tibia_joint_RB', 
                        'tibia_joint_LF', 'tibia_joint_LM', 'tibia_joint_LB', 
                        'femur_joint_RF', 'femur_joint_RM', 'femur_joint_RB', 
                        'femur_joint_LF', 'femur_joint_LM', 'femur_joint_LB', 
                        'coxa_joint_RF', 'coxa_joint_RM', 'coxa_joint_RB', 
                        'coxa_joint_LF', 'coxa_joint_LM', 'coxa_joint_LB']

    message_rviz.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    message_rviz.velocity = []
    message_rviz.effort = []

    try:
        while not rospy.is_shutdown():
            start = time.time()
            message_rviz.header.stamp = rospy.Time.now()
            pub.publish(message_rviz)
            rate.sleep()
            end = time.time()
            print("final time: ", end- start)

    except KeyboardInterrupt:
        exit()

if __name__ == '__main__':
    main()

