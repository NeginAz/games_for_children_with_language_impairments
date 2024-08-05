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
import rospy
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

emotion_dictionary= {0: "angry", 1: "happy" , 2: "excited", 3: "sad", 4: "scared", 5: "shy"}

happy = [ "When I'm happy, I smile!" , "When I'm happy, I feel like i have, a lot of energy" , "When I'm happy, I want to jump of joy!" , "Playing with my friends makes me happy!" , "Having my favorite snack makes me happy!" , "Staying with my family makes me feel happy." ]
sad = ["When im sad, my smile disappears." , 
    "When im sad, I want to cry.", 

    "My favorite toy is broken, it makes me feel sad." ,   

    "My friend is sad, it makes me feel sad too.", 

    "When im feeling sad, I can talk to my friends.",  

    "When im feeling sad, I can listen to my favorite music."]

angry = ["When I feel angry, I feel like I want to stomp my feet!" , "When I feel angry, I want to scream and shout", "when I feel angry, I feel like I am going to explode!", "when someone makes fun of me, it makes me feel angry!" , "when someone ruins my painting, it makes me feel angry" , "When i am blamed for something, I did n't do, it makes me feel angry!"]
excited = ["When I feel excited, I can not be calm.", 

    "When I feel excited, I feel like my heaty is racing.", 

    "When I feel excited, I can not stop laughing.",  

    "Travelling with my family, makes me feel excited.",  

    "My birthday is coming, it makes me feel excited.",  

    "Winning a trophy makes me feel excited."]

jealous = ["When I feel jealous, I feel like no one cares about me.", 

    "When I feel jealous, I become rudeness.", 

    "When someone has a new toy and I don’t, it makes me feel jealous." ,

    "When others gets more attention than me, it makes me feel jealous.",  

    "When others win in the race, it makes me feel jealous."]

disappointed = ["When I feel disappointed, my smile disappears.", 
    "When I feel disappointed, I want to stay alone.",   
    "When in feel disappointed, I am in bed mood.",   
    "When I cannot go to park as planned, I feel disappointed.",  
    "When my favorite flavor is sold out in the shop, I feel disappointed.", 
    "When what I want to have happen, doesn’t happen, I feel disappointed." ]




scared = ["When I feel scared, my heart beats so loud.",  

    "When I feel scared u want to hide in a place that makes me feel safe."  , 
    
    "When I feel scared, my legs shake.", 

    "When I get lost in a crowd, it makes me feel scared." , 

    "When I watch a spooky movie, It makes me feel scared.",  

    "When I have a bad dream, It makes me feel scared."]


shy = ["When I feel shy, I feel like my heart is racing." , "When I feel shy, I get red in my face."  , "When I feel shy, I want to hide behind my parents.", "Going to a new school, makes me feel shy." , "Speaking in front of class, makes me feel shy." ,  "Meeting mom and dad’s friends makes me feel shy."]
    
def send_post_request(emotion):
    r = requests.post('http://192.168.100.2:5000/request', data={'emotion': emotion})

def emotion_card(id):
    if id == 0:
        rospy.sleep(1.0)
        talktext_pub.publish("angry!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/angry")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/angry")
        r1 = random.randint(0,len(angry)-1)
        print("r1" , r1 , "length" , len(angry))        
        # talktext_pub.publish(angry[r1]) 
    elif id == 1: 
        rospy.sleep(1.0)
        talktext_pub.publish("happy!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/happy")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/happy")    
        r2 = random.randint(0,len(happy)-1)
        print("r2" , r2 , "length" , len(happy))        
        # talktext_pub.publish(happy[r2]) 
    elif id == 2:
        rospy.sleep(1.0)
        talktext_pub.publish("excited!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/happy_blinking")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/hoora")
        r3 = random.randint(0,len(excited)-1)
        print("r3" , r3 , "length" , len(excited))        
        # talktext_pub.publish(excited[r3]) 
    elif id == 3:
        rospy.sleep(1.0)
        talktext_pub.publish("sad!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/sad") 
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/sad")        
        r4 = random.randint(0,len(sad)-1)
        print("r4" , r4 , "length" , len(sad))        
        # talktext_pub.publish(sad[r4])      
    elif id == 4:
        rospy.sleep(1.0)
        talktext_pub.publish("scared!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/afraid")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/afraid")        
        r5 = random.randint(0,len(scared)-1)
        print("r5" , r5 , "length" , len(scared))        
        # talktext_pub.publish(scared[r5])
    elif id == 5:
        rospy.sleep(1.0)
        talktext_pub.publish("shy!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/shy")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/shy")
        r6 = random.randint(0,len(shy)-1)
        print("r6" , r6 , "length" , len(shy))
        # talktext_pub.publish(shy[r6]) 
    if (id < 6 and id > -1):
        talktext_pub.publish("which one is " + emotion_dictionary[id] ) 
        send_post_request(emotion_dictionary[id])

def img_callback(img):
    convertedImage = CvBridge().imgmsg_to_cv2(img, "bgr8")
    
    frame = imutils.resize(convertedImage, width=1920)

    # (corners, ids, rejected) = detector.detectMarkers(frame)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict,
    parameters=arucoParams)
    if corners:
        global t1
        cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        # print(corners)
        print(ids)
        t2 = time.time()
        if((t2- t1) >  6 and ids[0] < 6):
            emotion_card(ids[0][0])
            t1 = time.time()
        # print("rejected" , rejected)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        return  


def main_emotion_game2():
    # rospy.init_node('my_tutorial_node')
    threading.Thread(target=lambda:rospy.init_node('node2', disable_signals=True)).start() 
    rospy.loginfo("my_tutorial_node started!")
    global t1 
    t1 = time.time()
    global talktext_pub
    global speechSay_pub
    global emotionShow_pub
    global gesturePlay_pub
    # creating a ros publisher
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText',String,queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play',String,queue_size=10)    
    rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)



# if __name__ == '__main__':
#     rospy.init_node('my_tutorial_node')
#     rospy.loginfo("my_tutorial_node started!")
#     global t1 
#     t1 = time.time()
#    # creating a ros publisher
#     speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
#     emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
#     talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText',String,queue_size=10)
#     gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play',String,queue_size=10)    
#     rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)
    
#    # publish a text message to TTS
   

#     try:
#         rospy.spin()
        
#     except KeyboardInterrupt:
#         pass

#     rospy.loginfo("finsihed!")
