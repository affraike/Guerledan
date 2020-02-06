#!/usr/bin/env python
# -*-coding:utf-8 -*
import rospy
from test_kalman_command.msg import custom_cmd_motors
from encoders_boat.msg import Message_encoders

import sys
import time

import arduino_driver as ardudrv

should_stop = False
u1, u2 = 0, 0
            
def calcul_encoders():
    global posLeft0, posRight0, posRight1, posLeft1

    dOdoL = abs(delta_odo(posLeft1,posLeft0))
    dOdoR = abs(delta_odo(posRight1,posRight0))
    
    if(u1 == 0 and u2 == 0):

        Ecmdl = 0
        Ecmdr = 0
        
    elif(u1 >= u2):
        #on va diminuer u1 de façon a garder le rapport u1 sur u2
        #avec vitesse u2 aka uR aka vitesse droite constante
        #on a dOdoL/dOdoR = u1/u2
        
        errL = dOdoR*u1/u2-dOdoL
        errR = 0
        
        kp = 0.1

        Ecmdl = kp*errL
        Ecmdr = 0
            
    elif(u1 < u2):
        #on va diminuer u2 de façon à garder le rapport u1/u2
        #on a dOdoL/dOdoR = u1/u2
        
        errL = 0
        errR = dOdoL*u2/u1-dOdoR
        
        kp = 0.1

        Ecmdl = 0
        Ecmdr = kp*errR

    posLeft0 = posLeft1
    posRight0 = posRight1

    return (Ecmdl, Ecmdr)

def callback_cmd(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f %f", data.u1, data.u2)
    
    u1, u2 = data.u1, data.u2

    Ecmdl, Ecmdr = calcul_encoders()
    
    cmdl = 25.5 *(u1-Ecmdl)
    cmdr = 25.5 * (u2-Ecmdr)
    
    if(cmdl > 255):
        cmdl = 255
    elif(cmdl < 0):
    	cmdl = 0
        
    if(cmdr > 255):
        cmdr = 255
    elif(cmdr < 0):
    	cmdr = 0
    
    # ardudrv.send_arduino_cmd_motor(serial_arduino, cmdl, cmdr)

def callback_encoders(data):
    t = data.time
    global posLeft1, posRight1
    posLeft1 = data.enc_left
    posRight1 = data.enc_right

def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Driver_cmd_motors_encoders', anonymous=True)

    rospy.Subscriber("u", custom_cmd_motors, callback_cmd)
    rospy.Subscriber("encoders", Message_encoders, callback_encoders)

    global posLeft0, posRight0, posRight1, posLeft1
    posLeft0, posRight0, posRight1, posLeft1 = 0, 0, 0, 0

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    print("fin ")


if __name__ == '__main__':
    serial_arduino, data_arduino = ardudrv.init_arduino_line()
    listener()
    

