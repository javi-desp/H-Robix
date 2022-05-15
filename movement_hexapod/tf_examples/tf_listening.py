
import rospy, tf2_ros, geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np

legs = ["LF", "LM", "LB", "LF", "LM", "LB"]

if __name__ == '__main__':
    rospy.init_node('listener')
    rate = rospy.Rate(10.0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    i = 0

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if i == 5:
            i = 0
            break
        else:
            i += 1
        try:
            trans = tfBuffer.lookup_transform('coxa_LB', 'base_link', rospy.Time())
            print(trans)


            trans = tfBuffer.lookup_transform('body_link', 'coxa_'+ legs[i], rospy.Time())
            #print(trans)
            #print(trans.transform.rotation)
            r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            #print(trans.transform.rotation)
            #print(r.as_matrix())
            T_bc = np.identity(4)
            T_bc[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z] 
            T_bc[:3, :3] = r.as_matrix()
            print(T_bc)
            #print(T_bc)
            #print()
            # This will give you the coordinate of the child in the parent frame
        except Exception as e:
            print(e)
            pass
        
        rate.sleep()
"""
#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("coxa_LF", 'base_link', rospy.Time())
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
"""