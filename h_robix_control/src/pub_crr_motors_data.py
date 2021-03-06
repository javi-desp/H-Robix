#!/usr/bin/env python3

import rospy
import time
import math
from dynamixel_sdk import *
from h_robix_control.srv import *
from h_robix_control.msg import *
import numpy as np 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class service_to_joint_state: 
    def __init__(self, array_tibia_ids, array_femur_ids, array_coxa_ids):
        self.array_tibia_ids = array_tibia_ids
        self.array_femur_ids = array_femur_ids
        self.array_coxa_ids = array_coxa_ids

        rospy.init_node('read_motor_data')
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(100) # 100hz

        self.message_joint_state = JointState()
        self.message_joint_state.header = Header()
        self.message_joint_state.name = ['tibia_joint_LB', 'tibia_joint_LM', 'tibia_joint_LF', 
                        'tibia_joint_RB', 'tibia_joint_RM', 'tibia_joint_RF', 
                        'femur_joint_LB', 'femur_joint_LM', 'femur_joint_LF', 
                        'femur_joint_RB', 'femur_joint_RM', 'femur_joint_RF', 
                        'coxa_joint_LB', 'coxa_joint_LM', 'coxa_joint_LF', 
                        'coxa_joint_RB', 'coxa_joint_RM', 'coxa_joint_RF']
        self.message_joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.message_joint_state.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.message_joint_state.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.feedback = {"position": True, "velocity": True, "pwm": True}
    
        rospy.wait_for_service('get_dinamixel_motor_group_data')
        self.motor_group_service = rospy.ServiceProxy('get_dinamixel_motor_group_data', GetGroupMotorData)

    def update_joint_state_message(self):
        if self.feedback["position"]:
            self.message_joint_state.position = [
                    self.tibia_group_current_position.motor1_data*2*(math.pi/4095) - math.pi,
                    self.tibia_group_current_position.motor2_data*2*math.pi/4095 - math.pi,
                    self.tibia_group_current_position.motor3_data*2*math.pi/4095 - math.pi,
                    -self.tibia_group_current_position.motor4_data*2*math.pi/4095 + math.pi,
                    -self.tibia_group_current_position.motor5_data*2*math.pi/4095 + math.pi,
                    -self.tibia_group_current_position.motor6_data*2*math.pi/4095 + math.pi, 

                    -self.femur_group_current_position.motor1_data*2*math.pi/4095 + math.pi,
                    -self.femur_group_current_position.motor2_data*2*math.pi/4095 + math.pi,
                    -self.femur_group_current_position.motor3_data*2*math.pi/4095 + math.pi,
                    self.femur_group_current_position.motor4_data*2*math.pi/4095 - math.pi,
                    self.femur_group_current_position.motor5_data*2*math.pi/4095 - math.pi,
                    self.femur_group_current_position.motor6_data*2*math.pi/4095 - math.pi,  

                    -self.coxa_group_current_position.motor1_data*2*math.pi/4095 + math.pi,
                    -self.coxa_group_current_position.motor2_data*2*math.pi/4095 + math.pi,
                    -self.coxa_group_current_position.motor3_data*2*math.pi/4095 + math.pi,
                    -self.coxa_group_current_position.motor4_data*2*math.pi/4095 + math.pi,
                    -self.coxa_group_current_position.motor5_data*2*math.pi/4095 + math.pi,
                    -self.coxa_group_current_position.motor6_data*2*math.pi/4095 + math.pi
            ]
        if self.feedback["velocity"]:
            self.message_joint_state.velocity = [
                self.tibia_group_current_velocity.motor1_data,
                self.tibia_group_current_velocity.motor2_data,
                self.tibia_group_current_velocity.motor3_data,
                self.tibia_group_current_velocity.motor4_data,
                self.tibia_group_current_velocity.motor5_data,
                self.tibia_group_current_velocity.motor6_data,

                self.femur_group_current_velocity.motor1_data,
                self.femur_group_current_velocity.motor2_data,
                self.femur_group_current_velocity.motor3_data,
                self.femur_group_current_velocity.motor4_data,
                self.femur_group_current_velocity.motor5_data,
                self.femur_group_current_velocity.motor6_data,

                self.coxa_group_current_velocity.motor1_data,
                self.coxa_group_current_velocity.motor2_data,
                self.coxa_group_current_velocity.motor3_data,
                self.coxa_group_current_velocity.motor4_data,
                self.coxa_group_current_velocity.motor5_data,
                self.coxa_group_current_velocity.motor6_data, 
            ]
        if self.feedback["pwm"]:
            self.message_joint_state.effort = [
                np.int16( self.tibia_group_current_pwm.motor1_data),
                np.int16( self.tibia_group_current_pwm.motor2_data),
                np.int16( self.tibia_group_current_pwm.motor3_data),
                np.int16( self.tibia_group_current_pwm.motor4_data),
                np.int16( self.tibia_group_current_pwm.motor5_data),
                np.int16( self.tibia_group_current_pwm.motor6_data),

                np.int16( self.femur_group_current_pwm.motor1_data),
                np.int16( self.femur_group_current_pwm.motor2_data),
                np.int16( self.femur_group_current_pwm.motor3_data),
                np.int16( self.femur_group_current_pwm.motor4_data),
                np.int16( self.femur_group_current_pwm.motor5_data),
                np.int16( self.femur_group_current_pwm.motor6_data),

                np.int16( self.coxa_group_current_pwm.motor1_data),
                np.int16( self.coxa_group_current_pwm.motor2_data),
                np.int16( self.coxa_group_current_pwm.motor3_data),
                np.int16( self.coxa_group_current_pwm.motor4_data),
                np.int16( self.coxa_group_current_pwm.motor5_data),
                np.int16( self.coxa_group_current_pwm.motor6_data),                               
            ]

    def send_data_to_topic(self):
        while not rospy.is_shutdown():
            start = time.time()        
            try:
                if self.feedback["position"]:
                    self.tibia_group_current_position = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                            self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"position")

                    self.femur_group_current_position = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                            self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"position")

                    self.coxa_group_current_position = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                            self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"position")

                if self.feedback["velocity"]:
                    self.tibia_group_current_velocity = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                            self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"velocity")

                    self.femur_group_current_velocity = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                            self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"velocity")

                    self.coxa_group_current_velocity = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                            self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"velocity")
                    
                if self.feedback["pwm"]:
                    self.tibia_group_current_pwm = self.motor_group_service(self.array_tibia_ids[0],self.array_tibia_ids[1],self.array_tibia_ids[2],
                                                            self.array_tibia_ids[3],self.array_tibia_ids[4],self.array_tibia_ids[5],"PWM")

                    self.femur_group_current_pwm = self.motor_group_service(self.array_femur_ids[0],self.array_femur_ids[1],self.array_femur_ids[2],
                                                            self.array_femur_ids[3],self.array_femur_ids[4],self.array_femur_ids[5],"PWM")

                    self.coxa_group_current_pwm = self.motor_group_service(self.array_coxa_ids[0],self.array_coxa_ids[1],self.array_coxa_ids[2],
                                                            self.array_coxa_ids[3],self.array_coxa_ids[4],self.array_coxa_ids[5],"PWM")

            except: 
                pass
            if (self.tibia_group_current_position == False or self.femur_group_current_position == False or self.coxa_group_current_position == False):
                print("couldnt update position!!")
            else:
                self.update_joint_state_message()

                self.message_joint_state.header.stamp = rospy.Time.now()
                self.pub_joint_state.publish(self.message_joint_state)
                self.rate.sleep()

                #print("elapsed time = ", time.time() - start)

#TODO it can be done in server side
#TODO run after conf_motors in a proper way... 
def main():
    time.sleep(3)
    
    #TODO read ID motors from json conf file 
    ID_tibia = [1,2,3,4,5,6]
    ID_femur = [11,21,31,41,51,61]
    ID_coxa = [10,20,30,40,50,60]

    hexapod = service_to_joint_state(ID_tibia, ID_femur, ID_coxa)
    hexapod.send_data_to_topic()

if __name__ == '__main__':
    main()

