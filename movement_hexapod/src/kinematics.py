import math

x = 5
z = 8

p_femur = [x, 0, z]

femur_length = 2
tibia_length = 10

try:
    theta_2 = math.acos((p_femur[0]**2 + p_femur[2]**2 - femur_length**2 - tibia_length**2) / (2 * femur_length * tibia_length))
    theta_1 = -(math.atan2(z, x) + math.atan2((tibia_length * math.sin(theta_2)) , (femur_length + tibia_length * math.cos(theta_2))))
    print([theta_1 , theta_2], "----", True)
except ValueError:
    print([0, 0, 0],"----", False)

ang = [theta_1, theta_2]

try:
    D = (x**2 + z**2 - femur_length**2 - tibia_length**2)/(2*femur_length*tibia_length)
    theta_2_1 =  math.atan2(math.sqrt(1-D**2),D)
    theta_2_2 =  math.atan2(-math.sqrt(1-D**2), D)
    theta_1 = math.atan2(z,x) - math.atan2((tibia_length*math.sin(theta_2_1)), (femur_length*math.cos(theta_2_1 + tibia_length)))
    print([theta_1, theta_2_1, theta_2_2], "----", True)
except ValueError:
    print([0, 0, 0],"----", False)



leg_lenghts = {"femur": 2, "tibia": 10}

x = leg_lenghts["femur"] * math.cos(ang[0]) + leg_lenghts["tibia"] * math.cos(ang[0] + ang[1])
z = leg_lenghts["femur"] * math.sin(ang[0]) + leg_lenghts["tibia"] * math.sin(ang[0] + ang[1])

print()
print(x)
print(z)

ang = [theta_1, theta_2_2]

x = leg_lenghts["femur"] * math.cos(ang[0]) + leg_lenghts["tibia"] * math.cos(ang[0] + ang[1])
z = leg_lenghts["femur"] * math.sin(ang[0]) + leg_lenghts["tibia"] * math.sin(ang[0] + ang[1])

print()
print(x)
print(z)