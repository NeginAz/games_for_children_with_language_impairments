
from flask import Flask, render_template , request , redirect, url_for , session
from flask_socketio import SocketIO, emit , join_room
import random
import numpy as np
from random import seed
import sys
import sys
import rospy
from std_msgs.msg import String
import threading
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
import time


threading.Thread(target=lambda:rospy.init_node('app', disable_signals=True)).start() 
pub_speech = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
pub_audio = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)
pub_gesture = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
pub_emotion =  rospy.Publisher('/qt_robot/emotion/show',String,queue_size=10)
pub_talktext = rospy.Publisher('/qt_robot/behavior/talkText',String,queue_size=10)

speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play',gesture_play)
audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText' , behavior_talk_text)


# Flask constructor takes the name of 
# current module (__name__) as argument.
app = Flask(__name__)
app.config['SECRET_KEY'] = 'vnkdjnfjknfl1232#'
socketio = SocketIO(app)
# The route() function of the Flask class is a decorator, 
# which tells the application which URL should call 
# the associated function.
path = "/static/images"
seed(1)

global speech_flag
speech_flag = False #just to ensure that the student isn't spamming the button and the speech commands aren't being queued 

@app.route('/')
def hello_world():
    return render_template('start_game.html')
 
@app.route('/request',  methods = ['POST'])
def request_callback():
    data = request.form['action']
    # socketio.emit('redirect', {'url': url_for('first_view')})
    # rospy.sleep(3)
    global action
    action = data
    global speech_flag
    # speech_flag = False
    # print(data, "action")
    if(not speech_flag):
        socketio.emit('update image', {'path': [path + "/action_cards/" + action + ".png"]}, broadcast=True)      

def random_image_selector_1():
    l = []
    global index_two
    global index_one 
    global ind_img_one
    global ind_img_two
    global mdl1
    global mdl2
    #x = random.sample(range(11, 100), 10)
    x = random.sample(range(0, 12), 10)
    y = random.sample(range(1,5), 1) #categories
    ind_image = np.random.randint(6,9,2) #selects from the set that are in the middle
    print("ind image: " , ind_image)
    mdl1 = ind_image[0] +2
    mdl2 = ind_image[1] +2
    index = np.random.randint(0,4,2)
    elem1 = x[ind_image[0]]
    elem2 = x[ind_image[1]]
    x.insert(index[0] , elem1)
    x.insert(index[1]+4 , elem2)
    index_one = index[0]
    index_two = index[1]+4
    ind_img_one = ind_image[0]
    ind_img_two = ind_image[1]
    for i in range(0,12):
        name = 'v'+str(i)
        dic[name] = path + str(x[i])+".jpg"
        l.append(path + cat[y[0]] + "/" + str(x[i])+".jpg")
    return l



@socketio.on('next button')
def next_button(message):
    global speech_flag
    if(speech_flag == True):
        speech_flag = False 
        socketio.emit('update image', {'path': [path + "/emotions/ask.jpg"]}, broadcast=True)

def random_image_selector(id):
    l = []
    # x = random.sample(range(1,3), 1)
    # x = np.random.randint(0,3)
    x = id
    y = 5 - x
    print(emotions[x])
    l.append(path + "emotions/" + emotions[x] + ".jpg")
    l.append(path + "emotions/" + emotions[y] + ".jpg")
    print(l)
    return l

@socketio.on('selected')
def image_selected(message):
    global speech_flag
    if not speech_flag:
        speech_flag = True
        pub_speech.publish(action)    
        # time.sleep(1)
        # socketio.emit('update image', {'path': [path + "/emotions/ask.jpg"]}, broadcast=True)

        # speech_flag = False
        

@app.route('/first_view')
def first_view():
    return render_template('action_game.html')


@app.route('/default_view')
def default_view():
    return render_template('default.html')    


@socketio.on('start game')
def start_game(message):
    global speech_flag
    speech_flag = False
    if(message['who'] == 'start_game'):
        print("clicked")
    socketio.emit('redirect', {'url': url_for('first_view')})
        




if __name__ == '__main__':
    socketio.run(app,host='0.0.0.0',  debug = True )
