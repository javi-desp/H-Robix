import os
import rospy
import time
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def read_arm_info():
    rospy.wait_for_service('get_position_coxa_1')

    coxa = rospy.ServiceProxy('get_position_coxa_1' , GetPosition)

    return coxa

def main():
    pub = rospy.Publisher('joint_states', JointState, queue_size=2)
    rospy.init_node('update_current_data')
    rate = rospy.Rate(100) # 100hz


    coxa = read_arm_info()


    try:
        while not rospy.is_shutdown():
            start = time.time()
            print(coxa().position)
            end = time.time()

            print("RESPONSE IN ", end-start)

    except KeyboardInterrupt:
        exit()

if __name__ == '__main__':
    main()

