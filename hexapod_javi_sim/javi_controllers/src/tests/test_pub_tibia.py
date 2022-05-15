import os
import rospy
from dynamixel_sdk import *
from javi_controllers.srv import *
from javi_controllers.msg import *


def main():
    pub1 = rospy.Publisher('set_position_tibia_1', SetPosition, queue_size=20)
    pub2 = rospy.Publisher('set_position_tibia_2', SetPosition, queue_size=20)
    pub3 = rospy.Publisher('set_position_tibia_3', SetPosition, queue_size=20)
    pub4 = rospy.Publisher('set_position_tibia_4', SetPosition, queue_size=20)
    pub5 = rospy.Publisher('set_position_tibia_5', SetPosition, queue_size=20)
    pub6 = rospy.Publisher('set_position_tibia_6', SetPosition, queue_size=20)
    rospy.init_node('test_caca')
    rate = rospy.Rate(50) # 10hz

    i = 1200
    while not rospy.is_shutdown():
        i = i+3
        pub1.publish(i)
        rate.sleep()
        pub2.publish(i)
        rate.sleep()
        pub3.publish(i)
        rate.sleep()
        pub4.publish(i)
        rate.sleep()
        pub5.publish(i)
        rate.sleep()
        pub6.publish(i)
        rate.sleep()


if __name__ == '__main__':
    main()
