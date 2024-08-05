#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from time import time, ctime
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import random
import requests
import threading


ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


#for enabling command line arguments
ap = argparse.ArgumentParser()

ap.add_argument("-i", "--image", required=False, help="path to input image containing ArUCo tag")

#make sure that --type is the SAME as the type used when the tag was generated
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")

args = vars(ap.parse_args())

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()
# detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

object_action_dictionary= {
    0: "eat", 1: "play" , 2: "wash", 3: "walk", 4: "draw",
    5: "drink", 6: "sleep" , 7: "brush", 8: "bike", 9: "tidy"
}  

def send_post_request(action):
    r = requests.post('http://192.168.100.2:5000/request', data={'action': action})


def object_card(id):
    rospy.sleep(1.0)
    action = str(object_action_dictionary[id])
    talktext_pub.publish(action)
    send_post_request(action)
    
   
def img_callback(img):
    convertedImage = CvBridge().imgmsg_to_cv2(img, "bgr8")
    frame = imutils.resize(convertedImage, width=1920)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
    parameters=arucoParams)
    if corners:
        global t1
        cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        print(ids)
        t2 = time.time()
        if((t2- t1) >  4 and ids[0] < 10):
            # object_card(ids[0][0])
            send_post_request(str(object_action_dictionary[ids[0][0]]))
            t1 = time.time()
        # print("rejected" , rejected)
    #cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        return  
# def signal_handler(signal, frame):
#   sys.exit(0)

def closing(sth):
    # print(sth)
    return


def exit_main_action():
    # rospy.Subscriber('/usb_cam/image_raw/', Image, closing)
    global sub
    sub.unregister()
    cv2.destroyAllWindows()
    exit()


def main_action():
    # rospy.init_node('my_tutorial_node')
    threading.Thread(target=lambda:rospy.init_node('node4', disable_signals=True)).start() 
    rospy.loginfo("my_tutorial_node started!")
    global t1 
    t1 = time.time()
    global talktext_pub
    global speechSay_pub
    global emotionShow_pub
    global gesturePlay_pub
    global sub
    # creating a ros publisher
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText',String,queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play',String,queue_size=10)    
    sub = rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)




# if __name__ == '__main__':
#     rospy.init_node('my_tutorial_node')
#     rospy.loginfo("my_tutorial_node started!")
#     global t1 
#     t1 = time.time()
#    # creating a ros publisher
#     speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
#     emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
#     talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText',String,queue_size=10)
#     rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)
    
   # publish a text message to TTS
   

    # try:
    #     rospy.spin()
        
    # except KeyboardInterrupt:
    #     pass

    # rospy.loginfo("finsihed!")
