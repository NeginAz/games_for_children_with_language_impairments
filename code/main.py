import numpy as np
from flask import Flask, render_template, redirect, url_for, session, request
from flask_socketio import SocketIO, emit, join_room
import random
import time
import rospy
from std_msgs.msg import String, Float64MultiArray
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
from emotion_card import *
from object_action_card import *
from emotion_card2 import *
from emotion_card3 import *

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

path = "/static/images/"

# Initialize variables
arr_visit = [False, False, False]
praise_order = [0, 1, 2, 3]
encourage_order = [0, 1, 2]
random.shuffle(praise_order)
random.shuffle(encourage_order)
random_praise = 0
random_encouragement = 0

emotion_dictionary = {0: "angry", 1: "happy", 2: "excited", 3: "sad", 4: "scared", 5: "shy"}
emotions = {0: "happy", 1: "angry", 2: "scared", 3: "excited", 4: "shy", 5: "sad"}

happy = {'ice': 'Having ice cream makes me feel happy!', 'smile': 'When I am happy, I smile!',
         'energy': 'When I am happy, I feel like I have a lot of energy!', 'jump': 'When I am happy, I want to jump for joy!'}
sad = {'sad': 'When I am sad, my smile disappears', 'toy': 'My favorite toy is broken, it makes me feel sad!',
       'friend_sad': 'My friend is sad, it makes me feel sad too!'}
angry = {'scream': 'When I feel angry, I want to scream and yell!'}
shy = {'shy': 'When I feel shy, I get red in my face'}
excited = {'travel': 'Travelling with my family makes me feel excited'}
scared = {'scared': 'When I feel scared, my legs shake'}

@app.route('/')
def login():
    return render_template('login.html')

@socketio.on('login')
def logged_in(message):
    name = message['name']
    session_no = message["session_no"]
    age = message["age"]
    global file_name
    file_name = f"{name}_{session_no}.txt"
    with open(file_name, "w") as f:
        f.write(f"Name: {name}\nAge: {age}\nSession: {session_no}\n")
    socketio.emit('redirect', {'url': url_for('main_page')})

@app.route('/main')
def main_page():
    return render_template('main.html')

@socketio.on('click_main')
def main_menu(message):
    global game, file_name
    with open(file_name, 'a+') as f:
        f.write(f"{message['who']}\nTime: {time.ctime()}\n")
    game_map = {
        'instructions_game': 'taking_instruction',
        'emotion_game_1': ('main', 'emotion_games_start', 'emotion_game1'),
        'action_game': ('main_action', 'emotion_games_start', 'action_game'),
        'emotion_game2': ('main', 'emotion_games_start', 'emotion_game2'),
        'emotion_game3': ('main_emotion_game3', 'emotion_games_start', 'emotion_game3')
    }
    if message['who'] in game_map:
        if isinstance(game_map[message['who']], tuple):
            globals()[game_map[message['who']][0]]()
            game = game_map[message['who']][2]
            socketio.emit('redirect', {'url': url_for(game_map[message['who']][1])})
        else:
            socketio.emit('redirect', {'url': url_for(game_map[message['who']])})

@app.route('/taking_instruction_main')
def taking_instruction():
    return render_template('Taking_Instructions_main.html')

@app.route('/first_page')
def taking_instruction1():
    rospy.sleep(1.0)
    return render_template('index_taking_instruction.html')

@app.route('/emotion_games')
def emotion_games_start():
    return render_template('start_game.html')

@app.route('/second_page')
def taking_instruction2():
    rospy.sleep(1.0)
    return render_template('index_taking_instruction_page2.html')

@app.route('/third_page')
def taking_instruction3():
    rospy.sleep(1.0)
    return render_template('index_taking_instruction_page3.html')

@socketio.on('connect event')
def test_connect(message):
    print(message)

@socketio.on('disconnect')
def test_disconnect():
    print("disconnected")

@socketio.on('category_talk')
def first_talk_robot():
    print(arr_visit)
    rospy.sleep(2.0)
    socketio.emit('number', arr_visit, broadcast=True)
    if not arr_visit[0]:
        talktext_pub.publish("Let's go shopping for fruits, foods and school items!")
        rospy.sleep(3.0)
        talktext_pub.publish("Touch the correct one on the tablet!")
        arr_visit[0] = True
    elif not arr_visit[1]:
        talktext_pub.publish("Let's see what items are in a hospital!")
        rospy.sleep(2.5)
        talktext_pub.publish("Touch the correct one on the tablet!")
        arr_visit[1] = True
    elif not arr_visit[2]:
        talktext_pub.publish("Let's go to see plants and animals in the park!")
        rospy.sleep(3.0)
        talktext_pub.publish("Touch the correct one on the tablet!")
        arr_visit[2] = True
    else:
        talktext_pub.publish("All done! Let's go back!")

@socketio.on('init_after_category')
def init_interaction_robot(msg):
    print("message: ", msg)
    talktext_pub.publish(msg)

@socketio.on('first_talk')
def first_talk_robot(msg):
    print("message: ", msg)
    rospy.sleep(2.0)
    talktext_pub.publish(msg)

@socketio.on('giveme_talk')
def giveme_talk_robot(msg):
    print("message: ", msg)
    rospy.sleep(4.0)
    talktext_pub.publish(msg)

@socketio.on('object_list')
def correct_answer(obj):
    print(str(obj))
    global file_name
    with open(file_name, 'a+') as f:
        f.write(obj["data"] + " selected\n")

@socketio.on('correct')
def correct_answer():
    global random_praise, emotionShow_pub, gesturePlay_servc
    emotionShow_pub.publish("QT/happy")
    if random_praise == 4:
        random_praise = 0
        random.shuffle(praise_order)
    if praise_order[random_praise] == 0:
        gesturePlay_servc("QT/happy", 2)
        talktext_pub.publish("Good job!")
    elif praise_order[random_praise] == 1:
        gesturePlay_servc("QT/happy", 2)
        talktext_pub.publish("Well done!")
    elif praise_order[random_praise] == 2:
        gesturePlay_servc("QT/emotions/hoora", 2)
        talktext_pub.publish("Amazing!")
    else:
        gesturePlay_servc("QT/emotions/hoora", 2)
        talktext_pub.publish("Great job!")
    random_praise += 1

@socketio.on('wrong')
def score_handle_from_html():
    global random_encouragement, emotionShow_pub, gesturePlay_servc
    ref_r = Float64MultiArray()
    ref_l = Float64MultiArray()
    emotionShow_pub.publish("QT/sad")
    gesturePlay_servc("QT/sad", 1)
    ref_r.data = [-85, -65, -20]
    ref_l.data = [88, -71, -23]
    if random_encouragement == 3:
        random_encouragement = 0
        random.shuffle(encourage_order)
    if encourage_order[random_encouragement] == 0:
        talktext_pub.publish("Try again!")
    elif encourage_order[random_encouragement] == 1:
        talktext_pub.publish("Do it again!")
    else:
        talktext_pub.publish("Choose another one!")
    random_encouragement += 1
    right_pub.publish(ref_r)
    left_pub.publish(ref_l)

@socketio.on('wrong_repeat')
def speak_repeat(msg):
    emotionShow_pub.publish("QT/sad")
    rospy.sleep(9.0)
    talktext_pub.publish(str(msg))

@socketio.on('block_page')
def block_page_redirect(msg):
    rospy.sleep(3.0)
    talktext_pub.publish(str(msg))

@socketio.on('next_page')
def block_page_redirect():
    rospy.sleep(6.0)
    talktext_pub.publish("Let's go to the next page!")

@socketio.on('end')
def first_talk_robot():
    gesturePlay_servc("QT/happy", 2)
    rospy.sleep(1.0)
    talktext_pub.publish("Let's play another game!")

@socketio.on('client_disconnecting')
def disconnect_details(data):
    global file_name
    with open(file_name, 'a+') as f:
        f.write(data['data'] + " closed " + time.ctime() + "\n")
    if data['data'] in ["emotion_game1", "emotion_game
