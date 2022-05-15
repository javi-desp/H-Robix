"""
import math

point = [8, 0, 5]
leg_lenghts = {"coxa": 0, "femur": 2, "tibia": 10}

def inverse_kinematics(final_point, leg_lenghts):
    try:
        theta_1 = math.atan2(final_point[1], final_point[0])
        cos_theta3 = ((final_point[0]**2 + final_point[1]+ (final_point[2] - leg_lenghts["coxa"])**2 - leg_lenghts["femur"]**2 - leg_lenghts["tibia"]**2) / (2 * leg_lenghts["femur"] * leg_lenghts["tibia"]))
        theta_3 = math.atan2( math.sqrt(1 -cos_theta3**2), cos_theta3)
        theta_2 = (math.atan2(final_point[2] - leg_lenghts["coxa"], math.sqrt(final_point[0]**2 + final_point[1]**2)) - math.atan2((leg_lenghts["tibia"] * math.sin(theta_3)) , (leg_lenghts["femur"] + leg_lenghts["tibia"] * math.cos(theta_3))))

        return ([theta_1 , theta_2, theta_3])

    except ValueError:
        return ([0 , 0, 0])

def forward_kinematics(ang, leg_lenghts):
    x = math.cos(ang[0]) * (leg_lenghts["femur"] * math.cos(ang[1]) + leg_lenghts["tibia"] * math.cos(ang[1] + ang[2]))
    y = math.sin(ang[0]) *(leg_lenghts["femur"] * math.cos(ang[1]) + leg_lenghts["tibia"] * math.cos(ang[1] + ang[2]))
    z = leg_lenghts["coxa"] + leg_lenghts["femur"] * math.sin(ang[1]) + leg_lenghts["tibia"] * math.sin(ang[1] + ang[2])

    return ([x, y, z])

print(point)
print()
ang = inverse_kinematics(point, leg_lenghts)
print(ang)
print()
pos = forward_kinematics(ang, leg_lenghts)
print(pos)
"""

import math

point = [8, 0, 5]
leg_lenghts = {"coxa": 0, "femur": 2, "tibia": 10}

def inverse_kinematics(final_point, leg_lenghts):
    try:
        cos_theta3 = ((final_point[0]**2 + final_point[1]+ (final_point[2])**2 - leg_lenghts["femur"]**2 - leg_lenghts["tibia"]**2) / (2 * leg_lenghts["femur"] * leg_lenghts["tibia"]))
        theta_3 = math.atan2( math.sqrt(1 -cos_theta3**2), cos_theta3)
        theta_2 = (math.atan2(final_point[2], math.sqrt(final_point[0]**2 + final_point[1]**2)) - math.atan2((leg_lenghts["tibia"] * math.sin(theta_3)) , (leg_lenghts["femur"] + leg_lenghts["tibia"] * math.cos(theta_3))))

        return ([0, theta_2, theta_3])

    except ValueError:
        return ([0 , 0, 0])

def forward_kinematics(ang, leg_lenghts):
    x = (leg_lenghts["femur"] * math.cos(ang[1]) + leg_lenghts["tibia"] * math.cos(ang[1] + ang[2]))
    z = leg_lenghts["femur"] * math.sin(ang[1]) + leg_lenghts["tibia"] * math.sin(ang[1] + ang[2])

    return ([x, 0,  z])

print(point)
print()
ang = inverse_kinematics(point, leg_lenghts)
print(ang)
print()
pos = forward_kinematics(ang, leg_lenghts)
print(pos)
