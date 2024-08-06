#!/usr/bin/env python
import argparse
import random
import requests
import threading
import time

import cv2
import imutils
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

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

OBJECT_ACTION_DICTIONARY = {
    0: "eat", 1: "play", 2: "wash", 3: "walk", 4: "draw",
    5: "drink", 6: "sleep", 7: "brush", 8: "bike", 9: "tidy"
}

def send_post_request(action):
    requests.post('http://192.168.100.2:5000/request', data={'action': action})

def object_card(id):
    rospy.sleep(1.0)
    action = str(OBJECT_ACTION_DICTIONARY[id])
    talktext_pub.publish(action)
    send_post_request(action)

def img_callback(img):
    converted_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
    frame = imutils.resize(converted_image, width=1920)
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    
    if corners:
        global t1
        cv2.aruco.drawDetectedMarkers(frame, corners)
        print(ids)
        t2 = time.time()
        if (t2 - t1) > 4 and ids[0] < 10:
            send_post_request(str(OBJECT_ACTION_DICTIONARY[ids[0][0]]))
            t1 = time.time()
    
    if cv2.waitKey(1) == ord('q'):
        return

def exit_main_action():
    global sub
    sub.unregister()
    cv2.destroyAllWindows()
    exit()

def main_action():
    threading.Thread(target=lambda: rospy.init_node('node4', disable_signals=True)).start() 
    rospy.loginfo("my_tutorial_node started!")
    
    global t1 
    t1 = time.time()
    global talktext_pub
    global speechSay_pub
    global emotionShow_pub
    global gesturePlay_pub
    global sub
    
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
    sub = rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=False, help="path to input image containing ArUCo tag")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    print("[INFO] detecting '{}' tags...".format(args["type"]))
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    aruco_params = cv2.aruco.DetectorParameters()

    main_action()
