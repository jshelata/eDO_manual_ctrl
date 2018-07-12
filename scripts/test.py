#!/usr/bin/env python

import socket
import time
import os
import rospy

PACKAGE='edo_core_pkg'
import roslib
roslib.load_manifest(PACKAGE)

#from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import std_msgs.msg

from edo_core_msgs.msg import JointControl
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointControlArray
from edo_core_msgs.msg import MovementFeedback
from edo_core_msgs.msg import CartesianPose
from edo_core_msgs.msg import Point
from edo_core_msgs.msg import Frame

from edo_core_msgs.srv import ControlSwitch

from parse import parse

algo_jnt_ctrl = rospy.Publisher('algo_jnt_ctrl', JointControlArray, queue_size=1)
machine_move = rospy.Publisher('machine_move', MovementCommand, queue_size=1)

waitme = 0
def callback_algo_movement_ack(ack):
    global waitme

    #print("callback algo movement ack")
    if ack.type == 2:
        waitme = 1

startc5g = 0
def callback_bridge_c5g(val):
    global startc5g

    #print("callback bridge c5g")
    #print(val)
    if val.data == True:
        startc5g = 1
    else:
        startc5g = 0


rospy.init_node('c5g', anonymous=True)
rate = rospy.Rate(100) # 100hz

rospy.wait_for_service('algo_control_switch_srv')
control_switch = rospy.ServiceProxy('algo_control_switch_srv', ControlSwitch)

algo_movement_ack = rospy.Subscriber('algo_movement_ack', MovementFeedback, callback_algo_movement_ack)
bridge_c5g = rospy.Subscriber('bridge_c5g', std_msgs.msg.Bool, callback_bridge_c5g)

if __name__ == '__main__':

    print("ENDURANCE TEST")
    
    while True:
        
        print("Move 1")

        time.sleep(5)
    
        joints = [
                float(0),
                float(0),
                float(0),
                float(0),
                float(0),
                float(0),
                float(5),
                ]

        
        ros_cartesian_pose = CartesianPose(0.0,0.0,0.0,0.0,0.0,0.0,'')
        ros_point =  Point(74, ros_cartesian_pose, 127, joints)
        ros_frame =  Frame(0.0,0.0,0.0,0.0,0.0,0.0)
        mmmsg = MovementCommand(77, 74, 100, 0, 0, 0.0, ros_point, ros_point, ros_frame, ros_frame)

        # publish move
        machine_move.publish(mmmsg)
        print("Move 2")
        
        time.sleep(5)
        
        joints = [
                float(0),
                float(45),
                float(45),
                float(90),
                float(0),
                float(0),
                float(50),
                ]

        
        ros_cartesian_pose = CartesianPose(0.0,0.0,0.0,0.0,0.0,0.0,'')
        ros_point =  Point(74, ros_cartesian_pose, 127, joints)
        ros_frame =  Frame(0.0,0.0,0.0,0.0,0.0,0.0)
        mmmsg = MovementCommand(77, 74, 100, 0, 0, 0.0, ros_point, ros_point, ros_frame, ros_frame)
        # publish move
        machine_move.publish(mmmsg)


