import rospy, tf2_ros, geometry_msgs.msg


def callback(data):
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    tf2Stamp = geometry_msgs.msg.TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = "base_link"
    tf2Stamp.child_frame_id = "base_laser"
    tf2Stamp.transform.translation = (0.1, 0.0, 0.2)
    tf2Stamp.transform.rotation = (0.0, 0.0, 0.0)
    tf2Broadcast.sendTransform(tf2Stamp)

if __name__ == "__main__":
    rospy.init_node("talker")
    rospy.Subscriber("topic_name", geometry_msgs.msg, callback)
    rospy.spin()