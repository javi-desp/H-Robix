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

from javi_controllers.srv import *
from javi_controllers.msg import *


class hexapod_class:
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        rospy.init_node('remote_hexapod_control')
        self.rate = rospy.Rate(100)# 100hz
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

        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids

        self.coxa_group_current_position = [0, 0, 0, 0, 0, 0]
        self.femur_group_current_position = [0, 0, 0, 0, 0, 0]
        self.tibia_group_current_position = [0, 0, 0, 0, 0, 0]

        self.pub_to_motor_group = rospy.Publisher('set_dinamixel_motor_group_data', SetGroupMotorData, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.get_current_motor_data)

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
        self.poses_home = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
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
                break

        for joint in self.robot_description.joints:
            if "foot" in joint.name:
                self.leg_positions["tibia"] = [abs(ele) for ele in joint.origin.xyz]
                break

        for joint in self.robot_description.joints:
            if "base" in joint.name:
                self.floor_height = joint.origin.xyz[2]
                break
            
        for joint in self.leg_positions:
            self.leg_lenghts[joint] = math.sqrt(self.leg_positions[joint][0]**2 + self.leg_positions[joint][1]**2 + self.leg_positions[joint][2]**2)

    def create_msg_motor_control_test(self):
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

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

    def create_msg_motor_control_real(self, array_ids, mode, group_motor_data):

        set_position_leg_group = SetGroupMotorData()
        print(set_position_leg_group)
        set_position_leg_group.data_required = mode

        set_position_leg_group.motor1_id = array_ids[0]
        set_position_leg_group.motor2_id = array_ids[1]
        set_position_leg_group.motor3_id = array_ids[2]
        set_position_leg_group.motor4_id = array_ids[3]
        set_position_leg_group.motor5_id = array_ids[4]
        set_position_leg_group.motor6_id = array_ids[5]

        set_position_leg_group.motor1_data = group_motor_data[0]
        set_position_leg_group.motor2_data = group_motor_data[1]
        set_position_leg_group.motor3_data = group_motor_data[2]
        set_position_leg_group.motor4_data = group_motor_data[3]
        set_position_leg_group.motor5_data = group_motor_data[4]
        set_position_leg_group.motor6_data = group_motor_data[5]

        return set_position_leg_group

    def move_joints_hexapod(self, goal_coxa, goal_femur, goal_tibia):
        print(self.array_tibia_ids)
        set_position_tibia_group = self.create_msg_motor_control_real(self.array_tibia_ids, "position", goal_tibia)
        set_position_femur_group = self.create_msg_motor_control_real(self.array_femur_ids, "position", goal_femur)
        set_position_coxa_group = self.create_msg_motor_control_real(self.array_coxa_ids, "position", goal_coxa)
        self.pub_to_motor_group.publish(set_position_tibia_group)
        self.rate.sleep()
        self.pub_to_motor_group.publish(set_position_femur_group)
        self.rate.sleep()
        self.pub_to_motor_group.publish(set_position_coxa_group)
        self.rate.sleep()

    def convert_radians2binary_data(self, coxa_group_position, femur_group_position, tibia_group_position):
        tibia_group_position[0] = int (-tibia_group_position[0]/(2*(math.pi/4095) - math.pi))
        tibia_group_position[1] = int (-tibia_group_position[1]/(2*(math.pi/4095) - math.pi))
        tibia_group_position[2] = int (-tibia_group_position[2]/(2*(math.pi/4095) - math.pi))
        tibia_group_position[3] = int (tibia_group_position[3]/(2*(math.pi/4095) + math.pi))
        tibia_group_position[4] = int (tibia_group_position[4]/(2*(math.pi/4095) + math.pi))
        tibia_group_position[5] = int (tibia_group_position[5]/(2*(math.pi/4095) + math.pi))

        femur_group_position[0] = int (-femur_group_position[0]/(2*(math.pi/4095) + math.pi))
        femur_group_position[1] = int (-femur_group_position[1]/(2*(math.pi/4095) + math.pi))
        femur_group_position[2] = int (-femur_group_position[2]/(2*(math.pi/4095) + math.pi))
        femur_group_position[3] = int (femur_group_position[3]/(2*(math.pi/4095) - math.pi))
        femur_group_position[4] = int (femur_group_position[4]/(2*(math.pi/4095) - math.pi))
        femur_group_position[5] = int (femur_group_position[5]/(2*(math.pi/4095) - math.pi))
        
        coxa_group_position[0] = int (-coxa_group_position[0]/(2*(math.pi/4095) + math.pi))
        coxa_group_position[1] = int (-coxa_group_position[1]/(2*(math.pi/4095) + math.pi))
        coxa_group_position[2] = int (-coxa_group_position[2]/(2*(math.pi/4095) + math.pi))
        coxa_group_position[3] = int (-coxa_group_position[3]/(2*(math.pi/4095) + math.pi))
        coxa_group_position[4] = int (-coxa_group_position[4]/(2*(math.pi/4095) + math.pi))
        coxa_group_position[5] = int (-coxa_group_position[5]/(2*(math.pi/4095) + math.pi))

        return coxa_group_position, femur_group_position, tibia_group_position

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
            
            #CHOOSING IF CODO ARRIBA OR CODO ABAJO
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

    def get_current_motor_data(self, data):
        self.coxa_group_current_position = list(data.position[12:18])
        self.femur_group_current_position = list(data.position[6:12])
        self.tibia_group_current_position = list(data.position[0:6])
        print(self.tibia_group_current_position)
        print()
        print()
        print()
        print(data.position)

    def calculate_goal_points_movement(self, ang, h_hop, heigh_p):
        for key in self.poses_home:
            radius = 0.05
            self.poses_home_movement[key][2] = heigh_p
            self.poses_goal_movement_p1[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang + math.pi/4)/2 ,self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4)/2,heigh_p + h_hop]
            self.poses_goal_movement_p2[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang+ math.pi/4) ,self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4),heigh_p]
            
            
        pass

    def command_joints_debug_mode(self, poses_legs, group_ids):
        i = 0
        for key in poses_legs:
            if i in group_ids: 
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

    def run_debug_tripod_mode(self, dir, high = - 0.1, h_hop = 0.05):
        if type(dir) == list:
            dir = math.atan2(dir[1], dir[0])
        
        id_group_1 = [0, 2, 4]
        id_group_2 = [1, 3, 5 ]
        self.calculate_goal_points_movement(dir, h_hop, high)

        for i in range(2):
            if i == 0:
                #PHASE 1
                self.command_joints_debug_mode(self.poses_goal_movement_p1, id_group_1)
                self.command_joints_debug_mode(self.poses_home_movement, id_group_2)
                time.sleep(0.2)
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_goal_movement_p2, id_group_1)
                time.sleep(0.5)
                print("phase 1")
            else:
                #PHASE 2
                self.command_joints_debug_mode(self.poses_goal_movement_p1, id_group_2)
                self.command_joints_debug_mode(self.poses_home_movement, id_group_1)
                time.sleep(0.2)
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_goal_movement_p2, id_group_2)
                time.sleep(0.5)
                print("phase 2")                       

    def run_debug_wave_mode(self, dir, high = - 0.1, h_hop = 0.05):
        if type(dir) == list:
            dir = math.atan2(dir[1], dir[0])

        id_group_1 = [0, 1, 2]
        id_group_2 = [3, 4, 5 ]
        self.calculate_goal_points_movement(dir, h_hop, high)
        for phase in range(2):
            if phase == 1:
                print("phase 1")
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_home_movement, id_group_2)
                for i in range(3):
                        #PHASE 1
                        self.command_joints_debug_mode(self.poses_goal_movement_p1, [i])
                        time.sleep(0.2)
                        self.command_joints_debug_mode(self.poses_goal_movement_p2, [i])
                        time.sleep(0.5)
            else:
                print("phase 2")   
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_goal_movement_p1, id_group_1)
                for i in range(3):
                        #PHASE 2
                        self.command_joints_debug_mode(self.poses_goal_movement_p1, [i+3])
                        time.sleep(0.2)
                        self.command_joints_debug_mode(self.poses_goal_movement_p2, [i+3])
                        time.sleep(0.5)
                                          
    def run_debug_ripple_mode(self, dir, high = - 0.1, h_hop = 0.05):
        if type(dir) == list:
            dir = math.atan2(dir[1], dir[0])
        
        id_group_1 = [0, 2, 4]
        id_group_2 = [1, 3, 5 ]
        self.calculate_goal_points_movement(dir, h_hop, high)

        for i in range(2):
            if i == 0:
                #PHASE 1
                self.command_joints_debug_mode(self.poses_goal_movement_p1, id_group_1)
                self.command_joints_debug_mode(self.poses_home_movement, id_group_2)
                time.sleep(0.2)
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_goal_movement_p2, id_group_1)
                time.sleep(0.5)
                print("phase 1")
            else:
                #PHASE 2
                self.command_joints_debug_mode(self.poses_goal_movement_p1, id_group_2)
                self.command_joints_debug_mode(self.poses_home_movement, id_group_1)
                time.sleep(0.2)
                #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                self.command_joints_debug_mode(self.poses_goal_movement_p2, id_group_2)
                time.sleep(0.5)
                print("phase 2")                       


ID_tibia = [1,2,3,4,5,6]
ID_femur = [11,21,31,41,51,61]
ID_coxa = [10,20,30,40,50,60]

hexapod = hexapod_class(ID_tibia, ID_femur, ID_coxa)
"""
input()
goal_tibia = hexapod.tibia_group_current_position
print(goal_tibia)
print(type(goal_tibia[0]))
goal_tibia = [0, 2*math.pi, math.pi, math.pi/2, math.pi*(3/2), math.pi/4]
goal_femur = [0, 0, 0, 0, 0, 0]
goal_coxa = [0, 0, 0, 0, 0, 0]

goal_coxa, goal_femur, goal_tibia = hexapod.convert_radians2binary_data(goal_coxa, goal_femur, goal_tibia)
print(goal_tibia)
goal_tibia = [3100, 3100, 3100, 3100, 3100, 3100]
goal_femur = [1700, 1700, 1700, 1700, 1700, 1700]
goal_coxa = [2200, 2200, 2200, 2200, 2200, 2200]
hexapod.move_joints_hexapod(goal_coxa, goal_femur, goal_tibia)
time.sleep(5)
print("STOP")
exit()
rospy.spin()
exit()
"""

print()
print()
print()
print(hexapod.leg_lenghts)


hexapod.run_debug_tripod_mode()

#pos =hexapod.forward_kinematics(ang[0], ang[1], ang[2])
#print("FINAL POS= ", pos)
