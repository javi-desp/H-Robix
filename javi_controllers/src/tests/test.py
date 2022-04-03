import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.name = ['coxa_joint_RB', 'femur_joint_RB', 'tibia_joint_RB', 
                        'coxa_joint_RM', 'femur_joint_RM', 'tibia_joint_RM', 
                        'coxa_joint_RF', 'femur_joint_RF', 'tibia_joint_RF', 
                        'coxa_joint_LB', 'femur_joint_LB', 'tibia_joint_LB', 
                        'coxa_joint_LM', 'femur_joint_LM', 'tibia_joint_LM', 
                        'coxa_joint_LF', 'femur_joint_LF', 'tibia_joint_LF']
    hello_str.position = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hello_str.velocity = []
    hello_str.effort = []
    i = 0
    while not rospy.is_shutdown():
        i = i + 0.01
        print("YES")
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = [i, i, i, i, i, i, i, i, i, i, i, i,i, i, i, i, i, i]
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass