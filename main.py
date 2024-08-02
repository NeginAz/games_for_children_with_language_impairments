import numpy as np
from flask import Flask, render_template, redirect, url_for, session, request
from flask_socketio import SocketIO, emit, join_room
import random
import time
import threading
import rospy
from std_msgs.msg import String
from anytree import Node, RenderTree
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
from std_msgs.msg import Float64MultiArray
from emotion_card import *
from object_action_card import *
from emotion_card2 import *
from emotion_card3 import *



app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


path = "/static/images/"

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
def test_connect():
    print("disconnected")









@socketio.on('category_talk')
def first_talk_robot():
    print(arr_visit)
    rospy.sleep(2.0)
    socketio.emit('number', arr_visit, broadcast=True)
    if arr_visit[0] == False:
        talktext_pub.publish("Let's go shopping for fruits, foods and school items!")
        rospy.sleep(3.0)
        talktext_pub.publish("Touch the correct one on the tablet!")  # Instruction
        arr_visit[0] = True
    elif arr_visit[0] == True and arr_visit[1] == False:
        # audioPlay_pub.publish(" ") #hospital audio
        talktext_pub.publish("Let's see what items are in a hospital!")
        rospy.sleep(2.5)
        talktext_pub.publish("Touch the correct one on the tablet!")  # Instruction
        arr_visit[1] = True
    elif arr_visit[0] == True and arr_visit[1] == True and arr_visit[2] == False:
        talktext_pub.publish("Let's go to see plants and animals in the park!")  # park
        rospy.sleep(3.0)
        talktext_pub.publish("Touch the correct one on the tablet!")  # Instruction
        arr_visit[2] = True
    else:
        talktext_pub.publish("All done! Let's go back!")  # park


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
    with open(file_name,'a+') as f:
        f.write(obj["data"] +" selected " "\n")



@socketio.on('correct')
def correct_answer():
    global random_praise
    global emotionShow_pub
    global gesturePlay_servc
    emotionShow_pub.publish("QT/happy") 
    
    if random_praise == 4:
        random_praise = 0
        random.shuffle(praise_order)  # shuffle again
    if praise_order[random_praise] == 0:
        gesturePlay_servc("QT/happy", 2)
        talktext_pub.publish("Good job!")
        random_praise += 1
        # audioPlay_pub.publish("QT/good_job")
    elif praise_order[random_praise] == 1:
        gesturePlay_servc("QT/happy", 2)
        talktext_pub.publish("Well done!")
        random_praise += 1
        # audioPlay_pub.publish("QT/well_done")
    elif praise_order[random_praise] == 2:
        gesturePlay_servc("QT/emotions/hoora", 2)
        talktext_pub.publish("Amazing!")
        random_praise += 1
        # audioPlay_pub.publish("QT/amazing")
    else:
        gesturePlay_servc("QT/emotions/hoora", 2)
        talktext_pub.publish("Great job!")
        random_praise += 1
        # audioPlay_pub.publish("QT/amazing")


@socketio.on('wrong')
def score_handle_from_html():
    global random_encouragement
    global emotionShow_pub
    global gesturePlay_servc
    ref_r = Float64MultiArray()
    ref_l = Float64MultiArray()
    emotionShow_pub.publish("QT/sad") 
    gesturePlay_servc("QT/sad", 1) # it needs to down both hands after performing the gestures
    
    
    #[shoulderPitch, shoulderRoll, elbowRoll]
    ref_r.data = [-85,-65, -20]
    ref_l.data = [88, -71, -23]

    
    
    if random_encouragement == 3:
        random_encouragement = 0
        random.shuffle(encourage_order)  # shuffle again

    if encourage_order[random_encouragement] == 0:
        talktext_pub.publish("Try again!")
        random_encouragement += 1

    elif encourage_order[random_encouragement] == 1:
        talktext_pub.publish("Do it again!")
        random_encouragement += 1

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


@socketio.on('connect event')
def test_connect(message):
    print(message)


@socketio.on('disconnect')
def test_connect():
    print("disconnected")




@socketio.on('click_main')
def main_menu(message):
    global game
    global f
    global file_name
    game = ""
    with open(file_name,'a+') as f:
        start = time.ctime()
        f.write(message["who"] + "\n")
        f.write("Time: " + time.ctime() + "\n")
    if (message["who"] == 'instructions_game'):
        socketio.emit('redirect', {'url': url_for('taking_instruction')})
    elif (message["who"] == 'emotion_game_1'):
        main()
        game = "emotion_game1"
        socketio.emit('redirect', {'url': url_for('emotion_games_start')})
    elif(message["who"] == 'action_game'):
        main_action()
        game = "action_game"
        socketio.emit('redirect', {'url': url_for('emotion_games_start')})
    elif(message["who"] == 'emotion_game2'):
        game = "emotion_game2"
        main()
        socketio.emit('redirect', {'url': url_for('emotion_games_start')})
    elif(message["who"] == 'emotion_game3'):
        main_emotion_game3()
        game = "emotion_game3"
        socketio.emit('redirect', {'url': url_for('emotion_games_start')})


@app.route('/main')
def main_page():
    return render_template('main.html')

@socketio.on('client_disconnecting')
def disconnect_details(data):
    global f 
    global file_name
    with open(file_name,'a+') as f:
    # f.open(file_name , "a")
    # print("time.ctime(): ",time.ctime())
        f.write(data['data'] + " closed " + " " + time.ctime() + "\n")
    f.close()
    if(data['data'] == "emotion_game1" or data['data'] == "emotion_game2"):
        exit_main()
    elif(data['data'] == "action_game"):
        exit_main_action()
    elif(data['data'] == "emotion_game3"):
        exit_main_game()

@app.route('/taking_instruction_main')
def taking_instruction():
    return render_template('Taking_Instructions_main.html')


@app.route('/first_page')
def taking_instruction1():
    rospy.sleep(1.0)
    # talktext_pub.publish("I want fruits!")
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


@app.route('/hospital_first')
def hospital1():
    rospy.sleep(1.0)
    return render_template('hospital1_instruction.html')


@app.route('/hospital_second')
def hospital2():
    rospy.sleep(1.0)
    return render_template('hospital2_instruction.html')


@app.route('/park_first')
def park1():
    rospy.sleep(1.0)
    return render_template('park1_instruction.html')


@app.route('/park_second')
def park2():
    rospy.sleep(1.0)
    return render_template('park2_instruction.html')


######################################################################################### Negin ############################################################################################
# Emotion game1

def emotion_card(id):
    global talktext_pub
    global speechSay_pub
    global emotionShow_pub
    global gesturePlay_pub
    if id == 0:
        rospy.sleep(1.0)
        talktext_pub.publish("angry!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/angry")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/angry")
        # r1 = random.randint(0,len(angry)-1)
        # talktext_pub.publish(angry[r1]) 
    elif id == 1: 
        rospy.sleep(1.0)
        talktext_pub.publish("happy!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/happy")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/happy")    
        # r2 = random.randint(0,len(happy)-1)
        # talktext_pub.publish(happy[r2]) 
    elif id == 2:
        rospy.sleep(1.0)
        talktext_pub.publish("excited!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/happy_blinking")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/hoora")
        # r3 = random.randint(0,len(excited)-1)
        # talktext_pub.publish(excited[r3]) 
    elif id == 3:
        rospy.sleep(1.0)
        talktext_pub.publish("sad!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/sad") 
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/sad")        
        # r4 = random.randint(0,len(sad)-1)
        # talktext_pub.publish(sad[r4])      
    elif id == 4:
        rospy.sleep(1.0)
        talktext_pub.publish("scared!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/afraid")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/afraid")        
        # r5 = random.randint(0,len(scared)-1)
        # talktext_pub.publish(scared[r5])
    elif id == 5:
        rospy.sleep(1.0)
        talktext_pub.publish("shy!")
        rospy.sleep(1.0)
        emotionShow_pub.publish("QT/shy")
        rospy.sleep(5.0)
        gesturePlay_pub.publish("/QT/emotions/shy")
        # r6 = random.randint(0,len(shy)-1)
        # talktext_pub.publish(shy[r6]) 
    if (id < 6 and id > -1):
        rospy.sleep(8)
        talktext_pub.publish("Look at the tablet, which one is " + " " + emotion_dictionary[id]) 
        


emotion_dictionary= {0: "angry", 1: "happy" , 2: "excited", 3: "sad", 4: "scared", 5: "shy"}

def object_card(id):
    talktext_pub.publish(action)
    rospy.sleep(4)
    talktext_pub.publish("Look at the tablet, click on the picture") 

@app.route('/request', methods=['POST'])
def request_callback():
    global game
    global selected
    if (game == "emotion_game1"):
        id = request.form['emotion']
        data = emotion_dictionary[int(id)]
        emotion = list(emotions.keys())[list(emotions.values()).index(data)]
        if selected == 0 and int(id) < 7:
            emotion_card(int(id))
            socketio.emit('update image', {'path': random_image_selector(emotion)}, broadcast=True)
    elif (game == "emotion_game2"):
        # data = request.form['emotion']
        # id = list(emotions.keys())[list(emotions.values()).index(data)]
        id = request.form['emotion']
        data = emotion_dictionary[int(id)]
        emotion = list(emotions.keys())[list(emotions.values()).index(data)]        
        if selected == 0: 
            emotion_card(int(id))       
            socketio.emit('update image', {'path': random_image(emotion)}, broadcast=True)
    elif (game == "action_game"):
        global action
        action = request.form['action']
        global speech_flag
        if (not speech_flag):
            socketio.emit('update image', {'path': [path + "/action_cards/" + action + ".png"]}, broadcast=True)
            object_card(action)
    return "pass"

emotions = {0: "happy", 1: "angry", 2: "scared", 3: "excited", 4: "shy", 5: "sad"}



happy = {'ice': 'Having ice cream makes me feel happy!', 'smile': 'when I am happy, I smile!',
         'energy': 'When I am happy, I feel like I have a lot of energy!',
         'jump': 'When I am happy, I want to jump for joy!'}
sad = {'sad': 'When I am sad, my smile disappears', 'toy': 'My favorite toy is broken, it makes me feel sad!',
       'friend_sad': 'My friend is sad, it makes me feel sad too!'}
angry = {'scream': 'When I feel angry, I want to scream and yell!'}

shy ={'shy': 'When I feel Shy, I get red in my face'}


excited= {'travel':'Travelling with my family, makes me feel excited'}

scared = {'scared': 'When I feel scared, my legs shake'}



def random_image_selector(id):
    l = []
    x = id
    y = 5 - x
    print(emotions[x])
    l.append(path + "emotions/" + emotions[x] + ".jpg")
    l.append(path + "emotions/" + emotions[y] + ".jpg")
    return l

def random_image(id):
    global var
    if id == 0:
        i = random.randint(0, len(happy) - 1)
        em = list(happy.keys())[i]
        var = happy[em]
    elif id == 1:
        i = random.randint(0, len(angry) - 1)
        em = list(angry.keys())[i]
        var = angry[em]
    elif id == 2:
        i = random.randint(0, len(scared) - 1)
        em = list(scared.keys())[i]
        var = scared[em]
    elif id == 3:
        i = random.randint(0, len(excited) - 1)
        em = list(excited.keys())[i]
        var = excited[em]
    elif id == 4:
        i = random.randint(0, len(shy) - 1)
        em = list(shy.keys())[i]
        var = shy[em]
    elif id == 5:
        i = random.randint(0, len(sad) - 1)
        em = list(sad.keys())[i]
        var = sad[em]
    l = []
    l.append(path + "emotion_game_2/" + em + "1.png")
    l.append(path + "emotion_game_2/" + em + "2.png")
    print("list is :", l)
    return l


@socketio.on('start game')
def start_game(message):
    global selected
    global speech_flag
    global game
    selected = 0
    speech_flag = False
    if (message['who'] == 'start_game'):
        print("clicked")
        socketio.emit('redirect', {'url': url_for('first_view')})
        if(game == "emotion_game1" or game == "emotion_game2"):
            talktext_pub.publish("Let's play a game, show me an emotion card!")
        elif(game == "emotion_game3"):
            talktext_pub.publish("Let's play a game, show me two emotion cards")
        elif(game == "action_game"):
            talktext_pub.publish("Let's play a game, show me an action card!")
        rospy.sleep(1)



@socketio.on('selected')
def image_selected(message):
    global game
    global selected
    if(game == "emotion_game1"):
        if(selected == 0):
            if(message['who'] == "img00"):
                socketio.emit('highlight',{},  broadcast = True)
                speechSay_pub.publish("Good job!")
                selected = 1
                rospy.sleep(2)
                talktext_pub.publish("You can click next.")
            else:
                speechSay_pub.publish("Please try again!")
    elif(game == "emotion_game2"):
        global var
        if(selected == 0):
            if(message['who'] == "img00"):
                socketio.emit('highlight',{},  broadcast = True)
                speechSay_pub.publish(var)
                rospy.sleep(1)
                talktext_pub.publish("You can click next.")
                selected = 1
            else:
                speechSay_pub.publish("Please try again!")        
    elif(game == "action_game"):
        global speech_flag
        if not speech_flag:
            speech_flag = True
            speechSay_pub.publish(action)
            rospy.sleep(1)
            talktext_pub.publish("You can click next.")   
    #rospy.sleep(1)

@socketio.on('next button')
def next_button(message):
    global game
    global selected
    global speech_flag
    if(game == "emotion_game1" or game  == "emotion_game2"):
        rospy.sleep(2)
        talktext_pub.publish("Show me another emotion card!")
        rospy.sleep(1.5)
        if(selected == 1):
            selected = 0 
            socketio.emit('update image', {'path': [path + "emotions/ask.jpg" , path + "emotions/ask.jpg"]}, broadcast=True)
    elif(game == "action_game"):
        rospy.sleep(2)
        talktext_pub.publish("Show me another action card!")
        global speech_flag
        if(speech_flag == True):
            speech_flag = False 
            socketio.emit('update image', {'path': [path + "emotions/ask.jpg"]}, broadcast=True)

@app.route('/first_view')
def first_view():
    # # time.sleep(10)
    # print("game")
    # return render_template('emotion_game1.html')
    global game
    if(game == "emotion_game1"):
        return render_template('emotion_game1.html')
    elif(game == "emotion_game2"):
        return render_template('emotion_game1.html')
    elif(game == "action_game"):
        return render_template('action_game.html')
    elif(game == "emotion_game3"):
        return render_template('emotion_game3.html')



######################################################################################### Main ############################################################################################

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('app', disable_signals=True)).start()  # it helps to start the rospy and ends terminal
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    talktext_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    audioPlay_pub = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)
    
    gesturePlay_servc = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    rospy.wait_for_service('/qt_robot/gesture/play')
    
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
    left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=1)   
    global f
    time_start = time.ctime()
    socketio.run(app, host='0.0.0.0', debug=True)  # connect to 192.168.100.2:5000 in web
    # record end time
    end = time.ctime()
    # f.write(str((end - time_start) / 60))
    # f.close()
