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
        self.off_coxa = [math.pi/4 + math.pi, -math.pi/2, -math.pi/4, -math.pi/4 - math.pi, math.pi/2, math.pi/4]
        self.create_msg_motor_control_test()
        self.get_params()
        self.create_default_poses()

    def create_default_poses(self):
        pos1 = [-0.19, 0.17, 0, 1]
        pos1 = np.dot(self.T_base2coxa_list["coxa_LB"], pos1)
        pos2 = [0.0, 0.24, 0, 1]
        pos2 = np.dot(self.T_base2coxa_list["coxa_LM"], pos2)
        pos3 = [0.19, 0.17, 0, 1]
        pos3 = np.dot(self.T_base2coxa_list["coxa_LF"], pos3)
        pos4 = [-0.19, -0.17, 0, 1]
        pos4 = np.dot(self.T_base2coxa_list["coxa_RB"], pos4)
        pos5 = [0.0, -0.24, 0, 1]
        pos5 = np.dot(self.T_base2coxa_list["coxa_RM"], pos5)
        pos6 = [0.19, -0.17, 0, 1]
        pos6 = np.dot(self.T_base2coxa_list["coxa_RF"], pos6)

        self.poses_zero = []
        self.poses_home_movement = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_goal_movement_p1 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_goal_movement_p2 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}

        print(self.poses_home_movement)
        print(self.poses_goal_movement_p1)
        print(self.poses_goal_movement_p2)
        self.poses_sleep = {}

    def get_params(self):
        for joint in self.robot_description.joints:
            if "coxa" in joint.name:
                while True:
                    try:
                        tfBuffer = tf2_ros.Buffer()
                        listener = tf2_ros.TransformListener(tfBuffer)
                        self.rate.sleep()
                        trans = tfBuffer.lookup_transform('leg_center_' + joint.name[len(joint.name)-2 :], 'base_floor_link',rospy.Time(0), rospy.Duration(1.0))

                        r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

                        T_base2coxa = np.identity(4)
                        T_base2coxa[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z] 
                        T_base2coxa[:3, :3] = r.as_matrix()
                        self.T_base2coxa_list['coxa_' + joint.name[len(joint.name)-2 :]] = T_base2coxa
                        break

                    except Exception as e:
                        print("EXCEPTION")
                        print(e)

        for joint in self.robot_description.joints:
            if "femur" in joint.name:
                self.leg_positions["coxa"] = [abs(ele) for ele in joint.origin.xyz]
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

    def create_msg_motor_control_test(self):
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

    def inverse_kinematics(self, x, y, z, key_leg):
        try:
            if "L" in key_leg:
                z = -z
            theta_1 = math.atan2(y,x)
            theta_1 = theta_1
            y = math.sqrt(x**2 + y**2) - self.leg_lenghts["coxa"]
            cos_theta3 = (y**2+ z**2 - self.leg_lenghts["femur"]**2 - self.leg_lenghts["tibia"]**2) / (2 * self.leg_lenghts["femur"] * self.leg_lenghts["tibia"])


            theta_3_1 = math.atan2( math.sqrt(1 -cos_theta3**2), cos_theta3)
            theta_2_1 = (math.atan2(z, y) - math.atan2((self.leg_lenghts["tibia"] * math.sin(theta_3_1)) , (self.leg_lenghts["femur"] + self.leg_lenghts["tibia"] * math.cos(theta_3_1))))
            theta_3_2 = math.atan2( -math.sqrt(1 -cos_theta3**2), cos_theta3)
            theta_2_2 = (math.atan2(z, y) - math.atan2((self.leg_lenghts["tibia"] * math.sin(theta_3_2)) , (self.leg_lenghts["femur"] + self.leg_lenghts["tibia"] * math.cos(theta_3_2))))
            
            #DOS RESULTADOS !!!!!!!!
            if theta_2_1 < theta_2_2:
                if "L" in key_leg:
                    theta_2 = theta_2_1
                    theta_3 = theta_3_1
                else:
                    theta_2 = theta_2_2
                    theta_3 = theta_3_2                     
            else:
                if "L" in key_leg:
                    theta_2 = theta_2_2
                    theta_3 = theta_3_2     
                else:
                    theta_2 = theta_2_1
                    theta_3 = theta_3_1

            return ([theta_1 , theta_2, theta_3])

        except Exception as e:
            print(e)
            return None

    def forward_kinematics(self, q_coxa, q_femur, q_tibia):
        x = math.cos(q_coxa)*self.leg_lenghts["coxa"] + math.cos(q_coxa)*(self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        y = math.sin(q_coxa)*self.leg_lenghts["coxa"] + math.sin(q_coxa)*(self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        z = -self.leg_lenghts["femur"] * math.sin(q_femur) - self.leg_lenghts["tibia"] * math.sin(q_femur + q_tibia)

        return ([x, y, z])

    def calculate_goal_points_movement(self, ang, h_hop, heigh_p):
        for key in self.poses_home_movement :
            radius = 0.05
            self.poses_home_movement[key][2] = heigh_p
            self.poses_goal_movement_p1[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang + math.pi/4)/2 ,self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4)/2,heigh_p + h_hop]
            self.poses_goal_movement_p2[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang+ math.pi/4) ,self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4),heigh_p]
        pass

    def run(self, poses_legs):
        i = 0
        for key in poses_legs:
            print()
            print(key)
            print(poses_legs[key])
            ang = hexapod.inverse_kinematics(poses_legs[key][0], poses_legs[key][1], poses_legs[key][2],  key)
            print(ang)
            print(i)
            self.message_joint_state.position[i] = ang[2]
            self.message_joint_state.position[i+6] = -ang[1]
            self.message_joint_state.position[i+12] = ang[0] + self.off_coxa[i]
            i+=1
        self.message_joint_state.header.stamp = rospy.Time.now()
        self.pub_joint_state.publish(self.message_joint_state)
        self.rate.sleep()

hexapod = hexapod_class()
print()
print()
print()
print(hexapod.leg_lenghts)


#hexapod.run()

dir = 0
while dir < 2*math.pi:
     
    hexapod.calculate_goal_points_movement(dir, 0.05, -0.1)
    print(hexapod.poses_goal_movement_p2)

    #pos[0] += 0.0001
    dir += 0.4
    hexapod.run(hexapod.poses_home_movement)
    time.sleep(0.2)
    print("ture")
    hexapod.run(hexapod.poses_goal_movement_p1)
    time.sleep(0.2)
    hexapod.run(hexapod.poses_goal_movement_p2)
    time.sleep(0.5)
    print("ture")
#pos =hexapod.forward_kinematics(ang[0], ang[1], ang[2])
#print("FINAL POS= ", pos)
