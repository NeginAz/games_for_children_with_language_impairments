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

def joint_states_callback(msg):
    strmsg = ""
    for i, joint_name in enumerate(msg.name):
        strmsg += "%s: %.2f, " % (joint_name, msg.position[i])
    rospy.loginfo(strmsg)



if __name__ == '__main__':
    rospy.init_node('qt_motor_command')

    # create a publisher
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    gesturePlay("QT/touch-head", 0)
    
    # wait for publisher/subscriber connections
    wtime_begin = rospy.get_time()
    while (right_pub.get_num_connections() == 0) :
        rospy.loginfo("waiting for subscriber connections...")
        if rospy.get_time() - wtime_begin > 10.0:
            rospy.logerr("Timeout while waiting for subscribers connection!")
            sys.exit()
        rospy.sleep(1)
        


    rospy.loginfo("publishing motor commnad...")
    
    try:
        ref = Float64MultiArray()
        RightShoulderPitch = -90
        RightShoulderRoll = 100
        RightElbowRoll =15
        ref.data = [RightShoulderPitch, RightShoulderRoll, RightElbowRoll]
        right_pub.publish(ref)
        
        ref = Float64MultiArray()
        RightShoulderPitch = -90
        RightShoulderRoll = 100
        RightElbowRoll = 0
        ref.data = [RightShoulderPitch, RightShoulderRoll, RightElbowRoll]
        right_pub.publish(ref)
    except rospy.ROSInterruptException:
        rospy.logerr("could not publish motor commnad!")

    rospy.loginfo("motor commnad published")
