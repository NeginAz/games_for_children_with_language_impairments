import numpy as np
from flask import Flask, render_template, redirect, url_for, session, request
from flask_socketio import SocketIO, emit, join_room
#from flask_cors import CORS
import random
import time
import threading
import rospy
from std_msgs.msg import String
from anytree import Node, RenderTree
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *

def publishAllCuncurent():
    audioPlay_pub.publish("Qt2.wav")
    speechSay_pub.publish("Hello! This is QT talking using text to speech")
    gesturePlay_pub.publish("QT/happy")
    emotionShow_pub.publish("QT/happy")
    behaviorTalkText_pub.publish("I am QT robot! ")
    behaviorTalkAudio_pub.publish("Qt3.wav")

# the following activities will run in sequence on the robot
# one after another
def callAllSequence():
    audioPlay("Qt2.wav", "")
    speechSay("Hello! This is QT talking using text to speech")
    gesturePlay("QT/happy", 0)
    emotionShow("QT/happy")
    behaviorTalkText("I am QT robot!")
    behaviorTalkAudio("Qt3.wav", "");
    
def mine1():
    mine2()
    gesturePlay("QT/happy", 1)
    emotionShow_pub.publish("QT/happy")
    
def mine2():
    rospy.sleep(6)
    speechSay_pub.publish("Good job!")


if __name__ == '__main__':
    rospy.init_node('python_qt_example')

    # create a publisher
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    audioPlay_pub = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
    behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
    behaviorTalkAudio_pub = rospy.Publisher('/qt_robot/behavior/talkAudio', String, queue_size=10)

    # wait for publisher/subscriber connections
    wtime_begin = rospy.get_time()
    while (speechSay_pub.get_num_connections() == 0 or
           audioPlay_pub.get_num_connections() == 0 or
           emotionShow_pub.get_num_connections() == 0 or
           gesturePlay_pub.get_num_connections() == 0 or
           behaviorTalkText_pub.get_num_connections() == 0 or
           behaviorTalkAudio_pub.get_num_connections() == 0 ) :

        rospy.loginfo("waiting for subscriber connections")
        if rospy.get_time() - wtime_begin > 5.0:
            rospy.logerr("Timeout while waiting for subscribers connection!")
            sys.exit()
        rospy.sleep(1)

    # create some service clients
    audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    behaviorTalkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
    behaviorTalkAudio = rospy.ServiceProxy('/qt_robot/behavior/talkAudio', behavior_talk_audio)

    # wait for some services and connection to subscribers
    rospy.loginfo("waiting for services connections")
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/qt_robot/emotion/show')
    rospy.wait_for_service('/qt_robot/audio/play')
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/behavior/talkText')
    rospy.wait_for_service('/qt_robot/behavior/talkAudio')

    rospy.loginfo("ready...")
    try:
        mine1()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass

