#!/usr/bin/env python


import rospy
from test_kalman_command.msg import custom_cmd_motors

import arduino_driver as ardudrv


def callback_cmd(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f %f", data.u1, data.u2)
    
    u1, u2 = data.u1, data.u2
    # ardudrv.send_arduino_cmd_motor(serial_arduino, u1, u2)
    
def listener():
    rospy.init_node('Driver_cmd_motors', anonymous=True)
    rospy.Subscriber("u", custom_cmd_motors, callback_cmd)
    rospy.spin()
    

if __name__ == '__main__':
    serial_arduino, data_arduino = ardudrv.init_arduino_line()
    listener()
    

            
        
        
