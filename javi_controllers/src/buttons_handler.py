#!/usr/bin/env python3

import rospy
from javi_controllers.msg import *

import pigpio
from time import sleep     # this lets us have a time delay (see line 15)  

GPIO = pigpio.pi('hexapod')


GPIO.set_mode(17, pigpio.INPUT)
GPIO.set_pull_up_down(17, pigpio.PUD_UP)
GPIO.set_mode(22, pigpio.INPUT)
GPIO.set_pull_up_down(22, pigpio.PUD_UP)
GPIO.set_mode(27, pigpio.INPUT)
GPIO.set_pull_up_down(27, pigpio.PUD_UP)
GPIO.set_mode(5, pigpio.INPUT) 
GPIO.set_pull_up_down(25, pigpio.PUD_UP)
GPIO.set_mode(6, pigpio.INPUT)
GPIO.set_pull_up_down(6, pigpio.PUD_UP)
GPIO.set_mode(13, pigpio.INPUT) 
GPIO.set_pull_up_down(13, pigpio.PUD_UP)
GPIO.set_mode(26, pigpio.INPUT)
GPIO.set_pull_up_down(26, pigpio.PUD_UP)

rospy.init_node('pub_motor_data')

pub_buttons_state = rospy.Publisher('buttons_state', ButtonData, queue_size=10, latch=True)
rate = rospy.Rate(20) # 50hz
state_buttons = ButtonData()

while True:            # this will carry on until you hit CTRL+C  
    if not GPIO.read(26):   
        state_buttons.left_front_leg = True
    else:
        state_buttons.left_front_leg = False
        
    if not GPIO.read(13):   
       state_buttons.left_middle_leg = True
    else:
        state_buttons.left_middle_leg = False

    if not GPIO.read(17):   
       state_buttons.left_back_leg = True
    else:
        state_buttons.left_back_leg = False
        
    if not GPIO.read(6):   
       state_buttons.right_front_leg = True
    else:
        state_buttons.right_front_leg = False
        
    if not GPIO.read(5):   
       state_buttons.right_middle_leg = True
    else:
        state_buttons.right_middle_leg = False
        
    if not GPIO.read(22):
       state_buttons.right_back_leg = True
    else:
        state_buttons.right_back_leg = False
    
    pub_buttons_state.publish(state_buttons)
    rate.sleep()
