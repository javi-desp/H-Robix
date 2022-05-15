from ntpath import join
import rospy
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import rospy, tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import math


class hexapod_class:
    def __init__(self):
        rospy.init_node('hexapod_node')
        self.rate = rospy.Rate(10.0)
        self.robot_description = URDF.from_parameter_server()
        
        self.floor_height = 0
        self.T_base2coxa_list = {}
        self.q_joints = {"coxa":0, "femur": 0, "tibia": 0}
        self.xyz_point = {"x":0, "y": 0, "z": 0}
        self.leg_positions = {"coxa":0, "femur": 0, "tibia": 0}
        self.leg_lenghts = {"coxa":0, "femur": 0, "tibia": 0}
        self.offset_angle_kinematics = {"coxa":0, "femur": 0, "tibia": 0}
        self.create_msg_motor_control()
        self.get_params()
        

    def get_params(self):
        for joint in self.robot_description.joints:
            if "coxa" in joint.name:
                while True:
                    try:
                        tfBuffer = tf2_ros.Buffer()
                        listener = tf2_ros.TransformListener(tfBuffer)
                        self.rate.sleep()
                        trans = tfBuffer.lookup_transform('leg_center_' + joint.name[len(joint.name)-2 :], 'body_link',rospy.Time(0), rospy.Duration(1.0))

                        r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

                        T_base2coxa = np.identity(4)
                        T_base2coxa[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z] 
                        T_base2coxa[:3, :3] = r.as_matrix()
                        self.T_base2coxa_list['coxa_' + joint.name[len(joint.name)-2 :]] = T_base2coxa
                        #point = [0, 0, 0, 1]
                        #point2 = np.dot(self.T_base2coxa, point)
                        print("OK")
                        break

                    except Exception as e:
                        print("EXCEPTION")
                        print(e)

        for joint in self.robot_description.joints:
            if "femur" in joint.name:
                self.leg_positions["coxa"] = [0, 0, 0]
                break

        for joint in self.robot_description.joints:
            if "tibia" in joint.name:
                self.leg_positions["femur"] = [abs(ele) for ele in joint.origin.xyz]
                a = abs(math.atan2(self.leg_positions["femur"][0], self.leg_positions["femur"][2]))
                self.offset_angle_kinematics["femur"] = abs(math.atan2(self.leg_positions["femur"][0], self.leg_positions["femur"][2]))
                break

        for joint in self.robot_description.joints:
            if "foot" in joint.name:
                self.leg_positions["tibia"] = [abs(ele) for ele in joint.origin.xyz]
                self.offset_angle_kinematics["tibia"] = abs(math.atan2(self.leg_positions["tibia"][0], self.leg_positions["tibia"][2]))
                break

        for joint in self.robot_description.joints:
            if "base" in joint.name:
                self.floor_height = joint.origin.xyz[2]
                break
            
        for joint in self.leg_positions:
            self.leg_lenghts[joint] = math.sqrt(self.leg_positions[joint][0]**2 + self.leg_positions[joint][1]**2 + self.leg_positions[joint][2]**2)

    def create_msg_motor_control(self):
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(100) # 50hz

        self.message_joint_state = JointState()
        self.message_joint_state.header = Header()
        self.message_joint_state.name = ['tibia_joint_LB', 'tibia_joint_LM', 'tibia_joint_LF', 
                        'tibia_joint_RB', 'tibia_joint_RM', 'tibia_joint_RF', 
                        'femur_joint_LB', 'femur_joint_LM', 'femur_joint_LF', 
                        'femur_joint_RB', 'femur_joint_RM', 'femur_joint_RF', 
                        'coxa_joint_LB', 'coxa_joint_LM', 'coxa_joint_LF', 
                        'coxa_joint_RB', 'coxa_joint_RM', 'coxa_joint_RF']
        self.message_joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.message_joint_state.velocity = []
        self.message_joint_state.effort = []

    def inverse_kinematics(self, x, y, z):
        try:
            theta_1 = math.atan2(y, x)
            cos_theta3 = ((x**2 + y+ (z- self.leg_lenghts["coxa"])**2 - self.leg_lenghts["femur"]**2 - self.leg_lenghts["tibia"]**2) / (2 * self.leg_lenghts["femur"] * self.leg_lenghts["tibia"]))
            theta_3 = math.atan2( math.sqrt(1 -cos_theta3**2), cos_theta3)
            theta_2 = (math.atan2(z - self.leg_lenghts["coxa"], math.sqrt(x**2 + y**2)) - math.atan2((self.leg_lenghts["tibia"] * math.sin(theta_3)) , (self.leg_lenghts["femur"] + self.leg_lenghts["tibia"] * math.cos(theta_3))))

            return ([theta_1 , theta_2, theta_3])

        except Exception as e:
            print(e)
            return ([0 , 0, 0])

    def forward_kinematics(self, q_coxa, q_femur, q_tibia):
        x = math.cos(q_coxa) * (self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        y = math.sin(q_coxa) *(self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        z = self.leg_lenghts["coxa"] + self.leg_lenghts["femur"] * math.sin(q_femur) + self.leg_lenghts["tibia"] * math.sin(q_femur + q_tibia)

        return ([x, y, z])

    def run(self):
        while not rospy.is_shutdown():
            self.message_joint_state.position[2] = -math.pi/2
            self.message_joint_state.position[8] = - (math.pi/2)
            self.message_joint_state.position[14] = math.pi/3
            self.message_joint_state.header.stamp = rospy.Time.now()
            self.pub_joint_state.publish(self.message_joint_state)
            self.rate.sleep()
            print("F_K = ", hexapod.forward_kinematics(self.message_joint_state.position[14], -self.message_joint_state.position[8], self.message_joint_state.position[2]))
            exit()

hexapod = hexapod_class()
print()
print()
print()
print(hexapod.leg_lenghts)

ang = hexapod.inverse_kinematics(0.07524529968044517, 0.1303286820772772, 0.1)

print("I_K = ", ang)
point = hexapod.forward_kinematics(ang[0], ang[1], ang[2])
print("F_K = ",point)

#point2 = np.dot(hexapod.T_base2coxa_list["coxa_LF"], point.append(1))
#print(point2)
hexapod.run()

print(hexapod.leg_lenghts)
"""
i = 0.001
while True:
    i += 0.00001
    point = [0.125 , i, 0.1, 1]
    #point2 = np.dot(hexapod.T_base2coxa_list["coxa_LF"], point)
    #print(point2)
    ang = hexapod.inverse_kinematics(point)
    print(ang)
    hexapod.run(ang)
    print(ang)
    #hexapod.run()

"""
