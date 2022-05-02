import rospy
from javi_controllers.msg import *

import RPi.GPIO as GPIO
from time import sleep     # this lets us have a time delay (see line 15)  
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(17, GPIO.IN)   # set GPIO24 as an output (LED)  
GPIO.setup(22, GPIO.IN)    # set GPIO25 as input (button)  
GPIO.setup(27, GPIO.IN)    # set GPIO25 as input (button)  
GPIO.setup(5, GPIO.IN)   # set GPIO24 as an output (LED)  
GPIO.setup(6, GPIO.IN)    # set GPIO25 as input (button)  
GPIO.setup(13, GPIO.IN)   # set GPIO24 as an output (LED)  
GPIO.setup(26, GPIO.IN)

rospy.init_node('pub_motor_data')

pub_buttons_state = rospy.Publisher('buttons_state', ButtonData, queue_size=10, latch=True)
rate = rospy.Rate(20) # 50hz
state_buttons = ButtonData()

while True:            # this will carry on until you hit CTRL+C  
    if not GPIO.input(26):   
        print ("right-front leg")
        state_buttons.left_front_leg = True
    else:
        state_buttons.left_front_leg = False
        
    if not GPIO.input(13):   
       state_buttons.left_middle_leg = True
    else:
        state_buttons.left_middle_leg = False

    if not GPIO.input(17):   
       state_buttons.left_back_leg = True
    else:
        state_buttons.left_back_leg = False
        
    if not GPIO.input(6):   
       state_buttons.right_front_leg = True
    else:
        state_buttons.right_front_leg = False
        
    if not GPIO.input(5):   
       state_buttons.right_middle_leg = True
    else:
        state_buttons.right_middle_leg = False
        
    if not GPIO.input(22):
       state_buttons.right_back_leg = True
    else:
        state_buttons.right_back_leg = False
    
    pub_buttons_state.publish(state_buttons)
    rate.sleep()