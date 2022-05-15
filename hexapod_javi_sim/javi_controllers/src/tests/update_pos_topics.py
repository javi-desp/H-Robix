import os
import rospy
import time
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def read_arm_info(id):
    rospy.wait_for_service('get_position_coxa_' + str(id))
    rospy.wait_for_service('get_position_femur_' + str(id))
    rospy.wait_for_service('get_position_tibia_' + str(id))

    coxa = rospy.ServiceProxy('get_position_coxa_' + str(id), GetPosition)
    femur = rospy.ServiceProxy('get_position_femur_' + str(id), GetPosition)
    tibia = rospy.ServiceProxy('get_position_tibia_' + str(id), GetPosition)

    arm = [coxa, femur, tibia]

    return arm

def main():
    pub = rospy.Publisher('joint_states', JointState, queue_size=2)
    rospy.init_node('update_current_data')
    rate = rospy.Rate(100) # 100hz

    arm_list = []
    for id in range (1,7):
        arm = read_arm_info(id)
        arm_list.append(arm)

    message_rviz = JointState()
    message_rviz.header = Header()
    message_rviz.name = ['coxa_joint_RB', 'femur_joint_RB', 'tibia_joint_RB', 
                        'coxa_joint_RM', 'femur_joint_RM', 'tibia_joint_RM', 
                        'coxa_joint_RF', 'femur_joint_RF', 'tibia_joint_RF', 
                        'coxa_joint_LB', 'femur_joint_LB', 'tibia_joint_LB', 
                        'coxa_joint_LM', 'femur_joint_LM', 'tibia_joint_LM', 
                        'coxa_joint_LF', 'femur_joint_LF', 'tibia_joint_LF']
    message_rviz.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    message_rviz.velocity = []
    message_rviz.effort = []

    try:
        while not rospy.is_shutdown():
            start = time.time()
            pos = 0
            for i in range (6):

                message_rviz.position[pos] = float(arm_list[i][0]().position)*math.pi/4095
                rate.sleep()
                message_rviz.position[pos+1] = float(arm_list[i][1]().position)*math.pi/4095
                rate.sleep()
                message_rviz.position[pos+2] = float(arm_list[i][2]().position)*math.pi/4095
                rate.sleep()
                pos = pos +3
            
            message_rviz.header.stamp = rospy.Time.now()
            pub.publish(message_rviz)
            rate.sleep()
            end = time.time()
            print("final time: ", end- start)

    except KeyboardInterrupt:
        exit()

if __name__ == '__main__':
    main()

