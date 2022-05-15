import rospy, tf2_ros, geometry_msgs.msg
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        pointstamp = PointStamped()
        pointstamp.header.frame_id = "base_laser"
        pointstamp.header.stamp = rospy.Time(0)
        pointstamp.point.x = 1.0
        pointstamp.point.y = 2.0
        pointstamp.point.z = 3.0
        try:
            listener.sendTransform("base_link", pointstamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
            pass
        rate.sleep()