'''
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''
import sys
import rospy
from qt_gesture_controller.srv import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=1)   
#Functions to be used in Scripts
def rightArm (shoulderPitch, shoulderRoll, elbowRoll):
    ref = Float64MultiArray()
    ref.data = [shoulderPitch, shoulderRoll, elbowRoll]
    right_pub.publish(ref)

def leftArm (shoulderPitch, shoulderRoll, elbowRoll):
    ref = Float64MultiArray()
    ref.data = [shoulderPitch, shoulderRoll, elbowRoll]
    left_pub.publish(ref)
    
def start():
    gesturePlay("QT/touch-head", 0)
    rightArm(-90,-60,-35)
    leftArm(90,-60,-35)

if __name__ == '__main__':
    rospy.init_node('qt_motor_command')

    # create a publisher
    
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    #home_pos = rospy.ServiceProxy('/qt_robot/motors/home', home_pos)
    
    rospy.wait_for_service('/qt_robot/gesture/play')
    
    start()
