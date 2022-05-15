import rospy, tf2_ros, geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np

legs = ["LF", "LM", "LB", "LF", "LM", "LB"]

if __name__ == '__main__':
    rospy.init_node('listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for i in range (6):
            print("HOLA")
            try:
                trans = tfBuffer.lookup_transform("coxa_"+ legs[i], 'base_link', rospy.Time())
                #print(trans)
                #print(trans.transform.rotation)
                r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                print(trans.transform.rotation)
                #print(r.as_matrix())
                T_bc = np.identity(4)
                T_bc[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z] 
                T_bc[:3, :3] = r.as_matrix()

                print(T_bc)
                print()
                # This will give you the coordinate of the child in the parent frame
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                print("EXCEPT")
                pass
        exit()
        
        rate.sleep()