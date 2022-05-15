#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf2 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        i = 0.01
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            i += 0.01
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "base_laser1"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "base_link"


            t.transform.translation.x = 0.3
            t.transform.translation.y = 0.3
            t.transform.translation.z = 0

            t.transform.rotation.x = 1
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()