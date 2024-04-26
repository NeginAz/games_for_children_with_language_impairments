
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
path = "/static/images/"
dic = {'v0': " ", 'v1':" " , 'v2':" " ,'v3':" " , 'v4':" " , 'v5':" " ,'v6':" " , 'v7':" " , 'v8':" " , 'v9': " " , 'v10': " " , 'v11': " "}

img = {0:"dog",1:"deer" , 2:"giraffe",3:"lion", 4:"rabbit", 5:"panda", 6:"horses", 7:"cat",8:"tiger", 9:"whale", 10:"rhinoceros", 11:"dog",12:"fox", 13:"mouse" ,14:"cow",15:"bird", 16:"parrot", 17:"bird", 33:"flower", 50:"leave" , 71:"mountain" , 84:"sky"}

seed(1)




#pictures for spot it 
pic_name  = ['Birds' , 'Flowers' ,  'Leaves' ,  'Mountains' ,  'Sky']

cat = ["cat1" , "cat2" , "cat3" , "cat4" , "cat5"]
mem_cat = ["Fruits","Stationary","Vehicles","Acessories" ]

acs = {"0.jpg" : "Boots" , "1.jpg" : "Belt" , "2.jpg" : "Purse" , "3.jpg" :"Sun glasses" , "4.jpg" : "wallet" , 
"5.jpg" : "Shirt" , "6.jpg" : "Watch" , "7.jpg" : "Cap" ,"8.jpg" : "Sneakers", "9.jpg" : "Tie" }
fruits = {"0.jpg" : "Strawberries", "1.jpg" : "Watermelon" , "2.jpg" : "Cheries" , "3.jpg":"Mangos" ,
"4.jpg":"Blueberries" , "5.jpg" : "Tangerine" , "6.jpg" :"Avacado" ,"7.jpg" : "Pear" ,
"8.jpg" : "Kiwi" , "9.jpg" : "Banana" }

statioanry = {"0.jpg": "Crayons" , "1.jpg" : "notebook" , "2.jpg" : "ruler" , "3.jpg":"tape" ,
"4.jpg" : "Sharpner" , "5.jpg" : "Eraser" , "6.jpg" : "book" , 
"7.jpg" : "Scissors" , "8.jpg" : "Highlighter" , "9.jpg" : "pen" }
vehicles={"0.jpg" : "motor cycle" , "1.jpg" : "car" , "2.jpg" : "bus" , 
"3.jpg" : "train" , "4.jpg" : "bike" , "5.jpg": "air plane" , 
"6.jpg" : "ship" , "7.jpg" : "scooter" , "8.jpg" :"truck" }

@app.route('/')
def hello_world():
    return render_template('start_game.html')
 
@app.route('/request',  methods = ['POST'])
def request_callback():
    data = request.form['emotion']
    # socketio.emit('redirect', {'url': url_for('first_view')})
    # rospy.sleep(3)
    global selected
    
    id = list(emotions.keys())[list(emotions.values()).index(data)]
    # socketio.emit('update image', {'path': random_image(id)}, broadcast=True)  
    if(selected == 0):    
        socketio.emit('update image', {'path': random_image(id)}, broadcast=True)    


# def random_image_selector_1():
#     l = []
#     global index_two
#     global index_one 
#     global ind_img_one
#     global ind_img_two
#     global mdl1
#     global mdl2
#     #x = random.sample(range(11, 100), 10)
#     x = random.sample(range(0, 12), 10)
#     y = random.sample(range(1,5), 1) #categories
#     ind_image = np.random.randint(6,9,2) #selects from the set that are in the middle
#     print("ind image: " , ind_image)
#     mdl1 = ind_image[0] +2
#     mdl2 = ind_image[1] +2
#     index = np.random.randint(0,4,2)
#     elem1 = x[ind_image[0]]
#     elem2 = x[ind_image[1]]
#     x.insert(index[0] , elem1)
#     x.insert(index[1]+4 , elem2)
#     index_one = index[0]
#     index_two = index[1]+4
#     ind_img_one = ind_image[0]
#     ind_img_two = ind_image[1]
#     for i in range(0,12):
#         name = 'v'+str(i)
#         dic[name] = path + str(x[i])+".jpg"
#         l.append(path + cat[y[0]] + "/" + str(x[i])+".jpg")
#     return l


emotions = {0: "happy" , 1: "angry" , 2: "scared" ,  3: "excited"   , 4: "shy" , 5: "sad" }

happy = {'ice': 'Having ice cream makes me feel happy!' , 'smile':'when I am happy, I smile!' , 'energy': 'When I am happy, I feel like I have a lot of energy!' , 'jump': 'When I am happy, I want to jump for joy!'}
sad = {'sad': 'When I am sad, my smile disappears' , 'toy': 'My favorite toy is broken, it makes me feel sad!' , 'friend_sad': 'My friend is sad, it makes me feel sad too!'}
angry = {'scream': 'When I feel angry, I want to scream and yell!'}

# def random_image_selector(id):
#     l = []
#     # x = random.sample(range(1,3), 1)
#     # x = np.random.randint(0,3)
#     x = id
#     y = 5 - x
#     print(emotions[x])
#     l.append(path + "emotions/" + emotions[x] + ".jpg")
#     l.append(path + "emotions/" + emotions[y] + ".jpg")
#     print(l)
#     return l



def random_image(id):
    global var
    if id == 0:
        i = random.randint(0, len(happy)-1)
        em = list(happy.keys())[i]
        var = happy[em]
    elif id ==1:
        i = random.randint(0, len(angry)-1)
        em = list(angry.keys())[i]
        var = angry[em]
    elif id ==2:
        i = random.randint(0, len(scared)-1)
        em = list(scared.keys())[i]
        var = scared[em]
    elif id ==3:
        i = random.randint(0, len(excited)-1)
        em = list(excited.keys())[i]
        var = excited[em]
    elif id == 4:
        i = random.randint(0, len(shy)-1)
        em = list(shy.keys())[i]
        var = shy[em]
    elif id == 5:
        i = random.randint(0, len(sad)-1)
        em = list(sad.keys())[i]
        var = sad[em]
    l = []
    l.append(path + "emotion_game_2/" + em+ "1.png")
    l.append(path + "emotion_game_2/" + em + "2.png")  
    print("list is :" , l)
    return l  

@socketio.on('next button')
def next_button(message):
    global selected
    if(selected == 1):
        selected = 0 
        socketio.emit('update image', {'path': [path + "emotions/ask.jpg" , path + "emotions/ask.jpg"]}, broadcast=True)

@socketio.on('selected')
def image_selected(message):
    global selected
    global var
    if(selected == 0):
        if(message['who'] == "img00"):
            socketio.emit('highlight',{},  broadcast = True)
            # print("correct", file=sys.stderr)
            pub_speech.publish(var)
            # rospy.sleep(4)
            # time.sleep(2)
            # socketio.emit('update image', {'path': [path + "emotions/ask.jpg" , path + "emotions/ask.jpg"]}, broadcast=True)
            selected = 1
            # socketio.emit('redirect', {'url': url_for('default_view')})
        else:
            pub_speech.publish("Please try again!")
            # print("please try again", file=sys.stderr)

@app.route('/first_view')
def first_view():
    return render_template('emotion_game1.html')


@app.route('/default_view')
def default_view():
    return render_template('default.html')    


@socketio.on('start game')
def start_game(message):
    if(message['who'] == 'start_game'):
        print("clicked")
        global selected
        selected = 0
    socketio.emit('redirect', {'url': url_for('first_view')})
        




if __name__ == '__main__':
    socketio.run(app,host='0.0.0.0',  debug = True )