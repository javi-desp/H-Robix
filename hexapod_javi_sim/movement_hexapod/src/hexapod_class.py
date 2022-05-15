from ntpath import join
import rospy
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import rospy, tf2_ros, geometry_msgs.msg
import numpy as np
import math


class hexapod:
    def __init__(self):
        self.robot_description = URDF.from_parameter_server()
        self.T_bc_list = {}
        self.leg_positions = {"coxa":0, "femur": 0, "tibia": 0}
        self.leg_lenghts = {"coxa":0, "femur": 0, "tibia": 0}
        self.floor_height = 0
        self.offset_angle_kinematics = {"femur": 0, "tibia": 0}
        self.get_params()

    def get_params(self):
        for joint in self.robot_description.joints:
            if "coxa" in joint.name:
                #print(joint)
                print(joint.origin.rpy[2])
                r = R.from_euler('zyx', [joint.origin.rpy], degrees=False)
                T_bc = np.identity(4)
                T_bc[:3, 3] = joint.origin.xyz
                T_bc[:3, :3] = r.as_matrix()
                print(T_bc)
                self.T_bc_list[joint.name] = T_bc

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



hexapod_inst = hexapod()

for key in hexapod_inst.T_bc_list:
    print(key)
    print(hexapod_inst.T_bc_list[key])
    print()
    print()
    print()

print()
print()
print()
print(hexapod_inst.leg_positions)
print(hexapod_inst.leg_lenghts)
print(hexapod_inst.floor_height)