#!/usr/bin/env python3

from ntpath import join
import rospy
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import rospy, tf2_ros
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import numpy as np
import math

from h_robix_control.srv import *
from h_robix_control.msg import *


class hexapod_class:
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        rospy.init_node('remote_hexapod_control')
        self.rate = rospy.Rate(100)# 100hz
        self.robot_description = URDF.from_parameter_server()
        
        self.base_height = 0
        self.dir = [0,0]
        self.T_base2coxa_list = {}
        self.leg_positions = {"coxa":0, "femur": 0, "tibia": 0}
        self.leg_lenghts = {"coxa":0, "femur": 0, "tibia": 0}
        self.off_coxa = [math.pi/4 + math.pi, -math.pi/2, -math.pi/4, -math.pi/4 - math.pi, math.pi/2, math.pi/4]
        self.create_msg_motor_control_debug()
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
        rospy.Subscriber("hexapod_remote_control", Joy, self.update_goal_direction)

    def get_params(self):
        ##########################################################################################################################################
        #### @brief get model data from its URDF, like leg lenths moreover the transformation matrix coxa->base_link using tf2 
        ####        transform quaternion to transformation matrix using the lib scipy.spatial.transform
        #### @use __init__
        ##########################################################################################################################################
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
                break

        for joint in self.robot_description.joints:
            if "foot" in joint.name:
                self.leg_positions["tibia"] = [abs(ele) for ele in joint.origin.xyz]
                break

        for joint in self.robot_description.joints:
            if "base" in joint.name:
                self.base_height = joint.origin.xyz[2]
                break
            
        for joint in self.leg_positions:
            self.leg_lenghts[joint] = math.sqrt(self.leg_positions[joint][0]**2 + self.leg_positions[joint][1]**2 + self.leg_positions[joint][2]**2)

    def create_default_poses(self):
        ##########################################################################################################################################
        #### @brief generate home poses for each joint leg and list of goal poses
        #### @use __init__
        ##########################################################################################################################################

        pos1 = [-0.19*0.85, 0.17*0.85, -0.14, 1]
        pos1 = np.dot(self.T_base2coxa_list["coxa_LB"], pos1)
        pos2 = [0.0, 0.24*0.85, -0.14, 1]
        pos2 = np.dot(self.T_base2coxa_list["coxa_LM"], pos2)
        pos3 = [0.19*0.85, 0.17*0.85, -0.14, 1]
        pos3 = np.dot(self.T_base2coxa_list["coxa_LF"], pos3)
        pos4 = [-0.19*0.85, -0.17*0.85, -0.14, 1]
        pos4 = np.dot(self.T_base2coxa_list["coxa_RB"], pos4)
        pos5 = [0.0, -0.24*0.85, -0.14, 1]
        pos5 = np.dot(self.T_base2coxa_list["coxa_RM"], pos5)
        pos6 = [0.19*0.85, -0.17*0.85, -0.14, 1]
        pos6 = np.dot(self.T_base2coxa_list["coxa_RF"], pos6)

        self.poses_home2 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        """
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
        """
        self.poses_home = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}

        self.poses_home = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_home_movement = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_goal_movement_p1 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_goal_movement_p2 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_goal_movement_p2 = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_to_motors = {"LB":pos1, "LM": pos2, "LF": pos3, "RB":pos4, "RM": pos5, "RF": pos6}
        self.poses_sleep = {}

    def get_current_motor_data(self, data):
        ##########################################################################################################################################
        #### @brief when a new message comes to the node "joint states" this function passed it to the hexapod object self data
        #### @param data -> data of type JointState received at the ROS node 
        #### @use rospy.suscriber 
        ##########################################################################################################################################

        self.coxa_group_current_position = list(data.position[12:18])
        self.femur_group_current_position = list(data.position[6:12])
        self.tibia_group_current_position = list(data.position[0:6])

    def convert_radians2binary_data(self, leg_positions, id):
        ##########################################################################################################################################
        #### @brief convert rotation value from radians to binary data 12 bits (0-4095), depending of home offset of each motor
        #### @param leg_positions -> array of all joints of a leg
        #### @use in create_msg_motor_control_real()  
        ##########################################################################################################################################
        
        if id == 1:
            tibia = abs (int((leg_positions[2]+math.pi)/(2*math.pi/4095)))
            femur = abs (int((leg_positions[1]+math.pi)/(2*math.pi/4095)))
            coxa =  abs (int((-leg_positions[0] -self.off_coxa[0]+math.pi)/(2*math.pi/4095)))
        if id == 2:
            tibia = abs (int((leg_positions[2]+math.pi)/(2*math.pi/4095)))
            femur = abs (int((leg_positions[1]+math.pi)/(2*math.pi/4095)))
            coxa = abs (int((-leg_positions[0] -self.off_coxa[1]+math.pi)/(2*math.pi/4095)))
        if id == 3:
            tibia = abs (int((leg_positions[2]+math.pi)/(2*math.pi/4095)))
            femur = abs (int((leg_positions[1]+math.pi)/(2*math.pi/4095)))
            coxa = abs (int((-leg_positions[0] -self.off_coxa[2]+math.pi)/(2*math.pi/4095)))
        if id == 4:
            tibia = abs (int((leg_positions[2]-math.pi)/(2*math.pi/4095)))
            femur = abs (int((-leg_positions[1] +math.pi)/(2*math.pi/4095)))
            coxa = abs (int((-leg_positions[0] -self.off_coxa[3]+math.pi)/(2*math.pi/4095)))

        if id == 5:
            tibia = abs (int((leg_positions[2]-math.pi)/(2*math.pi/4095)))
            femur = abs (int((-leg_positions[1] +math.pi)/(2*math.pi/4095)))
            coxa = abs (int((-leg_positions[0] -self.off_coxa[4]+math.pi)/(2*math.pi/4095)))

        if id == 6:
            tibia = abs (int((leg_positions[2]-math.pi)/(2*math.pi/4095)))
            femur = abs (int((-leg_positions[1] +math.pi)/(2*math.pi/4095)))
            coxa = abs (int((-leg_positions[0] -self.off_coxa[5]+math.pi)/(2*math.pi/4095)))

        return {"tibia":tibia, "femur":femur, "coxa": coxa, "id": id}

    def create_msg_motor_control_debug(self):
        ##########################################################################################################################################
        #### @brief it creates the ROS msg type Joint state to send positions to Rviz in order to perform simulations of walk modes
        #### @use __init__
        ##########################################################################################################################################

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

    def create_msg_motor_control_real(self, mode, list_legs):
        ##########################################################################################################################################
        #### @brief it creates the ROS msg type SetGroupMotorData defined to send data to the motors
        #### @param mode -> string defining operation modes of motors (position, velocity, pwm)
        #### @param list_legs -> array of legs. The driver implemented recives a ROS message with 6 motor id, in this function we send data from 2 legs, but if
        ####                     for some reason we only need to move 1 leg it will send the same data 2 times  
        #### @use command_position() 
        ##########################################################################################################################################

        #TODO add radiantobinary
        if len (list_legs) < 2:
            set_position_leg_group = SetGroupMotorData()
            set_position_leg_group.data_required = mode

            set_position_leg_group.motor1_id = list_legs[0]["id"]                      # tibia
            set_position_leg_group.motor2_id = int ( str(list_legs[0]["id"]) + "0")    # coxa
            set_position_leg_group.motor3_id = int ( str(list_legs[0]["id"]) + "1")    # femur

            set_position_leg_group.motor4_id = 255
            set_position_leg_group.motor5_id = 255
            set_position_leg_group.motor6_id = 255

            set_position_leg_group.motor1_data = list_legs[0]["tibia"]
            set_position_leg_group.motor2_data = list_legs[0]["coxa"]
            set_position_leg_group.motor3_data = list_legs[0]["femur"]
            set_position_leg_group.motor4_data = 0
            set_position_leg_group.motor5_data = 0
            set_position_leg_group.motor6_data = 0
        else:
            set_position_leg_group = SetGroupMotorData()
            set_position_leg_group.data_required = mode

            set_position_leg_group.motor1_id = list_legs[0]["id"]                      # tibia
            set_position_leg_group.motor2_id = int ( str(list_legs[0]["id"]) + "0")    # coxa
            set_position_leg_group.motor3_id = int ( str(list_legs[0]["id"]) + "1")    # femur

            set_position_leg_group.motor4_id = list_legs[1]["id"]
            set_position_leg_group.motor5_id = int ( str(list_legs[1]["id"]) + "0")
            set_position_leg_group.motor6_id = int ( str(list_legs[1]["id"]) + "1")

            set_position_leg_group.motor1_data = list_legs[0]["tibia"]
            set_position_leg_group.motor2_data = list_legs[0]["coxa"]
            set_position_leg_group.motor3_data = list_legs[0]["femur"]
            set_position_leg_group.motor4_data = list_legs[1]["tibia"]
            set_position_leg_group.motor5_data = list_legs[1]["coxa"]
            set_position_leg_group.motor6_data = list_legs[1]["femur"]

        return set_position_leg_group

    def command_position(self, poses_legs, group_ids, mode="real"):
        ##########################################################################################################################################
        #### @brief depends on the mode it sends (publish) data to the corresponding topic  the goal position of each leg in xyz plane
        #### @param poses_legs -> array of cartesian poses of each leg
        #### @param group_ids -> id of legs to move
        #### @param mode -> mode of running code (sending data to motors o simulation)
        #### @use in gait modes  
        ##########################################################################################################################################

        if mode == "debug":
            i = 0
            for key in poses_legs:
                if i in group_ids: 
                    ang = self.inverse_kinematics(poses_legs[key][0], poses_legs[key][1], poses_legs[key][2],  key)
                    self.message_joint_state.position[i] = ang[2]
                    self.message_joint_state.position[i+6] = -ang[1]
                    self.message_joint_state.position[i+12] = ang[0] + self.off_coxa[i]
                i+=1
            self.message_joint_state.header.stamp = rospy.Time.now()
            self.pub_joint_state.publish(self.message_joint_state)
            self.rate.sleep()
        
        elif mode == "real":
            i = 0
            binary_goal_pos = []
            for key in poses_legs:
                if i in group_ids: 
                    ang_pos = self.inverse_kinematics(poses_legs[key][0], poses_legs[key][1], poses_legs[key][2], key)
                    binary_goal_pos.append( self.convert_radians2binary_data(ang_pos, i+1))
                i +=1

            for i in range(0, len(binary_goal_pos), 2):
                set_position_legs_group = self.create_msg_motor_control_real("position", binary_goal_pos[i:i+2])
                self.pub_to_motor_group.publish(set_position_legs_group)
        else:
            print("MODE BAD DEFINED")

    def inverse_kinematics(self, x, y, z, key_leg):
        ##########################################################################################################################################
        #### @brief perform inverse kinematics for one leg with geometric solution
        #### @param x -> x goal pos
        #### @param y -> y goal pos
        #### @param z -> z goal pos
        #### @param key_leg -> key of leg to move
        #### @use in command_position()  
        #### @return [theta_3 , theta_2, theta_1] -> angle for each joint in radians (thetha N3 -> tibia, 2-> femur, 1-> coxa)
        ##########################################################################################################################################
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
        ##########################################################################################################################################
        #### @brief perform forward kinematics for one leg with geometric solution
        #### @param q_coxa -> goal q coxa
        #### @param q_femur -> goal q femur 
        #### @param q_tibia -> goal z tibia
        #### @use in
        #### @return [x , y, z] -> goal position in cartesian
        ##########################################################################################################################################

        x = math.cos(q_coxa)*self.leg_lenghts["coxa"] + math.cos(q_coxa)*(self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        y = math.sin(q_coxa)*self.leg_lenghts["coxa"] + math.sin(q_coxa)*(self.leg_lenghts["femur"] * math.cos(q_femur) + self.leg_lenghts["tibia"] * math.cos(q_femur + q_tibia))
        z = -self.leg_lenghts["femur"] * math.sin(q_femur) - self.leg_lenghts["tibia"] * math.sin(q_femur + q_tibia)

        return ([x, y, z])

    def calculate_goal_points_movement(self, ang, h_hop, h_leg):
        ##########################################################################################################################################
        #### @brief get two points of trayectory (p1 -> half goal distance elevated point, p2 goal distance and position)
        #### @param ang -> ang of goal movement circumference
        #### @param h_leg -> heigh of the final point of the leg (foot)
        #### @param h_hop -> heigh of the hop of the trayectory (needed in order to do a gait in a rough terrain )
        #### @use in gait modes  
        ##########################################################################################################################################

        for key in self.poses_home:
            #radius of movement circumference (depends on the speed of movement)
            radius = 0.055
            self.poses_home_movement[key][2] = h_leg
            self.poses_goal_movement_p1[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang + math.pi/4)/2,
                                                self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4)/2,
                                                h_leg + h_hop]
            self.poses_goal_movement_p2[key] = [self.poses_home_movement[key][0] + radius*math.cos(ang + math.pi/4),
                                                self.poses_home_movement[key][1] + radius*math.sin(ang + math.pi/4),
                                                h_leg]

    def update_goal_direction(self, data):
        self.dir[0] = data.axes[0]
        self.dir[1] = data.axes[1]

    def run_tripod_mode(self, h_legs = - 0.1, h_hop = 0.05, mode = "real"):
        ##########################################################################################################################################
        #### @brief perform triple gait 
        #### @param h_legs -> heigh of the final point of the legs (foot)
        #### @param h_hop -> heigh of the hop of the trayectory (needed in order to do a gait in a rough terrain )
        #### @param mode -> mode of running code (sending data to motors o simulation)
        #### @use in main()
        ##########################################################################################################################################

        #TODO self.base_height

        if type(self.dir) == list:
            ang = math.atan2(self.dir[1], self.dir[0])
            speed = math.sqrt(math.pow(self.dir[0],2) + math.pow(self.dir[1],2))
        else:
            return 
        
        id_group_1 = [0, 2, 4]
        id_group_2 = [1, 3, 5 ]
        self.calculate_goal_points_movement(ang, h_hop, h_legs)

        try:
            for phase in range(2):
                if phase == 0:
                    #PHASE 1
                    self.command_position(self.poses_goal_movement_p1, id_group_1, mode)
                    self.command_position(self.poses_home_movement, id_group_2, mode)
                    time.sleep(0.5)
                    #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                    self.command_position(self.poses_goal_movement_p2, id_group_1, mode)
                    time.sleep(0.5)
                if phase == 1:   
                    #PHASE 2
                    self.command_position(self.poses_goal_movement_p1, id_group_2, mode)
                    self.command_position(self.poses_home_movement, id_group_1, mode)
                    time.sleep(0.5)
                    #DECREASE VELOCITY IN THIS CASE TO DO 2 MOVEMENTS IN THE SAME TIME
                    self.command_position(self.poses_goal_movement_p2, id_group_2, mode)
                    time.sleep(0.5)
                    
        except:
            exit()
                                  
    def run_wave_mode(self, h_legs = - 0.1, h_hop = 0.05, mode = "real"):
        ##########################################################################################################################################
        #### @brief perform wave gait 
        #### @param h_legs -> heigh of the final point of the legs (foot)
        #### @param h_hop -> heigh of the hop of the trayectory (needed in order to do a gait in a rough terrain )
        #### @param mode -> mode of running code (sending data to motors o simulation)
        #### @use in main()
        ##########################################################################################################################################

        #TODO self.base_height

        if type(dir) == list:
            ang = math.atan2(self.dir[1], self.dir[0])

        self.calculate_goal_points_movement(ang, h_hop, h_legs)
        for leg in range(6):
                self.command_position(self.poses_goal_movement_p1, [leg], mode)
                self.command_position(self.poses_goal_movement_p1, [leg], mode)
                time.sleep(0.5)
                self.command_position(self.poses_goal_movement_p2, [leg], mode)
                self.command_position(self.poses_goal_movement_p2, [leg], mode)
                time.sleep(0.5)
                self.command_position(self.poses_home_movement, [leg], mode)
                self.command_position(self.poses_home_movement, [leg], mode)
                time.sleep(0.5)
                                        
    def run_ripple_mode(self, h_legs = -0.1, h_hop = 0.05, mode = "real"):
        ##########################################################################################################################################
        #### @brief perform triple gait 
        #### @param h_legs -> heigh of the final point of the legs (foot)
        #### @param h_hop -> heigh of the hop of the trayectory (needed in order to do a gait in a rough terrain )
        #### @param mode -> mode of running code (sending data to motors o simulation)
        #### @use in main()
        ##########################################################################################################################################

        #TODO 
        pass                  


def main():

    #TODO read from json conf file 
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]

    hexapod = hexapod_class(ID_tibia, ID_femur, ID_coxa)
    start = time.time()
    try:
        while True:
            if hexapod.dir[0]== 0 and hexapod.dir[1]== 0:
                for leg in range(6):
                        time.sleep(0.1)
                        hexapod.command_position(hexapod.poses_home_movement, [leg], "debug")
            else:
                hexapod.run_tripod_mode(mode="debug")

    except rospy.ROSInterruptException:
            print("FIN")

    
if __name__ == '__main__':
    main()
