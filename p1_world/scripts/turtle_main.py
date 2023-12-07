#!/usr/bin/env python
import threading
#from speechy_NoAI import listen
import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty
import random
import numpy as np
import traceback
from sound_play.libsoundplay import SoundClient
from sound_play.msg import sound_play
import speech_recognition as sr
import wave
from audio_common_msgs.msg import AudioData

#tou imports
import itertools
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import subprocess
import os


python3_path = 'python3.7'
file = "ai.py"
file_path = os.path.join(os.path.dirname(__file__), file)

command = "Halt"
currentNode = "home"

nodesDict = {'home': (.254, -.287), 'apples': (-.362, 3.8), 'basketballs': (.911, 4.6), 'cats': (-.383, 5.56), 'dogs': (1.5, 5.71)}

soundhandle = SoundClient()


# parts integrated portion from group provided by Josiah Dadulo, Kevin Nguyen, Karan Peo, and Andrew Graham
def manhattan_distance(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    return abs(x1 - x2) + abs(y1 - y2)

# parts integrated portion from group provided by Josiah Dadulo, Kevin Nguyen, Karan Peo, and Andrew Graham
def total_distance(path, nodes):
    distance = 0
    for i in range(len(path) - 1):
        distance += manhattan_distance(nodes[path[i]], nodes[path[i + 1]])
    distance += manhattan_distance(nodes[path[-1]], nodes[path[0]])
    return distance

# parts integrated portion from group provided by Josiah Dadulo, Kevin Nguyen, Karan Peo, and Andrew Graham
def tsp_manhattan(nodes):
    num_nodes = len(nodes)
    if num_nodes <= 2:
        return nodes

    node_labels = [chr(ord('A') + i) for i in range(num_nodes)]  # Create labels

    # Create a list of node labels (excluding the starting node, which is assumed to be the first one, 'A')
    node_indices = node_labels[1:]

    shortest_path = None
    shortest_distance = float('inf')

    for perm in itertools.permutations(node_indices):
        path = ['A'] + list(perm) + ['A']  # Start and end at 'A'
        distance = total_distance(path, nodes)
        if distance < shortest_distance:
            shortest_path = path
            shortest_distance = distance
    print(shortest_path)
    path = []
    for node in shortest_path:
        path.append(nodes[node])
    
    print(path)

    return path

#TODO
#def navigate_path(path):

# parts integrated portion from group provided by Josiah Dadulo, Kevin Nguyen, Karan Peo, and Andrew Graham
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def navigate_to_multiple_points(self, points):
        for point in points:
            position = {'x': point[0], 'y': point[1]}
            quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = self.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")
            rospy.sleep(1)

    def erase_highlight(self, node_name):
        try:
            with open("highlights.txt", 'r') as highlights_file:
                lines = highlights_file.readlines()

            with open("highlights.txt", 'w') as highlights_file:
                for line in lines:
                    name, _, _ = line.strip().split(',')
                    if name.lower() != node_name.lower():
                        highlights_file.write(line)

            print("Highlight erased for {}".format(node_name))
        except IOError as e:
            print("highlights.txt file not found")
    def check_highlight(self, node_name):
        node_name = node_name.lower()
        not_found = True
        try:
            highlights = open("highlights.txt", 'r')
            lines = highlights.readlines()
            print("Checking " + node_name)

            for line in lines:
                name, x, y = line.strip().split(',')
                print(name + " = " + node_name + "?")
                name = name.lower()
                if name == node_name:
                    not_found = False
                    print("Highlight found!")
                    print("{} at ({}, {})".format(name, x, y))
                    print("Would you like to use this highlight?")
                    user_response = raw_input("'Y' = Yes, 'N' = No, erase existing and create new.")
                    if (user_response == 'Y' or 'y'):
                        return float(x), float(y)
                    else:
                        self.erase_highlight(node_name)
                
            # Highlight not found or deleted, create new highlight
            if (not_found):
                print("Highlight not found. Creating new highlight for " + node_name + ".")
            
            x = raw_input("Please enter the x coordinate: ")
            y = raw_input("Please enter the y coordinate: ")
            highlights.close()
            highlights = open("highlights.txt", 'a')
            new_line = node_name + ',' + x + ',' + y
            highlights.write(new_line + '\n')
            highlights.close()
            print("Highlight created for " + node_name + ".")
            return float(x), float(y)
        except IOError as e:
            print("Highlights file not found.")



# Save the terminal settings
settings = termios.tcgetattr(sys.stdin)



#class SounderPlayer:
#    def __init__ (self):
#        self.player = rospy.Subscriber("/sound_play", sound_play, self.sound_player)
#
#    def sound_player(self, player):
#        sound_play.
        




class AudioRecorder:
    def __init__ (self, sample_rate=16000, channels=1, duration=4, wake_word = False, halt_word = False):
        self.sample_rate = sample_rate
        self.channels = channels
        self.duration = duration
        self.audio_buffer = np.array([], dtype=np.int16)
        self.data = rospy.Subscriber("/audio/audio", AudioData, self.audio_callback)
        self.user_question = ""
        self.wake_word = wake_word
        self.halt_word = halt_word
        print("inititalize audio")

    def audio_callback(self, data):
        #print("adding audio")
        # Append new audio data to the buffer
        new_data = np.frombuffer(data.data, dtype=np.int16)
        self.audio_buffer = np.concatenate((self.audio_buffer, new_data))

        # Check if buffer has 2 seconds of audio
        if len(self.audio_buffer) >= self.sample_rate * self.duration:
            self.save_to_wav()
            self.audio_buffer = np.array([], dtype=np.int16)  # Clear the buffer

    def save_to_wav(self):
        # Save buffer to a WAV file
        wav_file = wave.open('voice_in.wav', 'wb')
        wav_file.setnchannels(self.channels)

        wav_file.setsampwidth(2)  # 16-bit samples
        wav_file.setframerate(self.sample_rate)
        wav_file.writeframes(self.audio_buffer.tobytes())
        wav_file.close()

        print("saving wave")
        threading.Thread(target=self.listen())
        #self.listen()
        

    
    def listen(self):
        global command
        # Initialize the recognizer
        r = sr.Recognizer()
        
        with sr.WavFile("voice_in.wav") as source:                # use the default microphone as the audio source
            audio = r.listen(source) 

            try:
                list = r.recognize(audio,True) # generate a list of possible transcriptions
                print(self.wake_word)          
                print("Possible transcriptions:")
                for prediction in list:
                    print(" " + prediction["text"] + " (" + str(prediction["confidence"]*100) + "%)")
                    self.user_question = prediction["text"]
                    if ("green") in str(self.user_question).lower():
                        print("sent to openai")
                        try:
                            subprocess.call([python3_path, file_path, self.user_question])
                        except Exception as e:
                            print(e)
                        print("openai finished")

                    elif ("oliver") in str(self.user_question).lower():
                        print("works")
                        self.wake_word = True
                        
                        print("Check the user instructions in the list, then reset the wake word and duration")
                        # input dictionary
                        dictionary = {
                            "automode": ["auto mode", "auto mode", "auto", "auto mode", "auto mode", "auto"],
                            "halt": ["halt", "stop", "explode"],
                            "To_Home": ["go to home", "to home"],
                            "To_A": ["go to apples", "to apples"],
                            "To_B": ["go to basketballs", "to basketballs"],
                            "To_C": ["go to cats", "to cats"],
                            "To_D": ["go to dogs", "to dogs", "to dawgz", "to dogz," "to dog"],
                            "where": ["where am I", "where"],
                            "here_apples": ["at apples", "you are at apples"],
                            "here_basketballs": ["at basketballs", "you are at basketballs"],
                            "here_cats": ["at cats", "you are at cats"],
                            "here_dogs": ["at dogs", "you are at dogs"],
                            "tour": ["start tour"]
                        }
                        
                        for key, value in dictionary.items():
                            for val in value:
                                if val in str(self.user_question).lower():
                                    command = key
                                    print(command)
                                    break

                        self.wake_word = False                       
                        break
                    elif ("halt") in str(self.user_question).lower():
                        print("Halted")
                        command = "Halt"
                        self.halt_word = True
                        break
                    else:
                        print("Word not detected")
            except Exception as e:
                print(e) 

     
    
        print(self.user_question)
        return str(self.user_question)

def audio_print(audio):
    rospy.loginfo("audio receeved")
    print("logged")

def goToNode(node):
    global currentNode
    global command
    global soundhandle

    if currentNode is node:
        soundhandle.say("Already at node " + str(node))
    else: 
        soundhandle.say("Currently at " + str(currentNode) + " Going to " + str(node) + "Hello Hello Hello Hello Hello"  + "I am Oliver. Oliver is me. Oliver Oliver Oliver Oliver" + "Hello I am Oliver and Oliver is me and I am travelling because thats what Oliver does. He moves and moves and moves. Yiiiiippppeeeeee"+ "Nyyyyyyyyoooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooom")
        mode = 'p'
        print("traveling to " + str(node) + "in mode: " + str(mode))
        mainNav(mode = mode, node = node)

        ### Stuff to path to ndoe while command is not halt

    rospy.sleep(1)
    ######
    currentNode = node
    command = ""

def whereAmI():
    global soundhandle
    global currentNode
    global command
    say = "We are currently as node " + str(currentNode) + " This node very cool"
    stringLen = len(say)

    soundhandle.say(say)

    rospy.sleep(0.1 * stringLen)
    command = ""

def youAreHere(node):
    global soundhandle
    global currentNode
    global command

    say = "I guess I am at node" + node
    stringLen = len(say)

    soundhandle.say(say)

    rospy.sleep(0.1 * stringLen)
    currentNode = node
    command = ""

def tour():
    global soundhandle
 
    global command

    say = "Starting the tour"
    stringLen = len(say)

    soundhandle.say(say)

    rospy.sleep(0.1 * stringLen)

    command = ""

# parts integrated portion from group provided by Josiah Dadulo, Kevin Nguyen, Karan Peo, and Andrew Graham
def mainNav(mode, node):
    global nodesDict
    user_input = mode
    done = False
    global command
    try:
        navigator = GoToPose()
        print("Weclome to robot tour guide!")
        while(not done and command is not "halt"):
            print('===================================================')
            print("Would you like a point-to-point tour or tour guide?")
            
            print(user_input)
            if user_input == 'p':
                print("Point-to-point mode activated")
                x = nodesDict[node][0]
                y = nodesDict[node][1]
                position = {'x': float(x), 'y' : float(y)}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                success = navigator.goto(position, quaternion)

                if success:
                    print("success")
                    rospy.loginfo("Hooray, reached the desired pose")
                    done = True
                    currentNode = node
                    soundhandle.say("We made it to " + str(currentNode) + ". What now?")
                else:
                    done = True
                    soundhandle.say("We did not make it to node " + str(node))
                    currentNode = "notmade"
                    rospy.loginfo("The base failed to reach the desired pose")

                # Sleep to give the last log messages time to be sent
                rospy.sleep(1)
            elif user_input == 't':
                print("Tour guide mode activated!!")
                nodes = {'A': (-.362, 3.8), 'B': (.911, 4.6), 'C': (-.383, 5.56), 'D': (1.5, 5.71)}
                user_input = raw_input("Would you like to do a tour of all four levels or a custom tour? 1 = tour of all four levels, 2 = custom tour")
                if user_input == '1':
                    path_one = {'A': (.254, -.287), 'B': (-.665, 1.87)} 
                    shortest_path = tsp_manhattan(path_one)
                    navigator.navigate_to_multiple_points(path_one[node] for node in shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    print("Changing levels")
                    path_two = {'A': (0.0701, 3.34), 'B': (0.0701, 3.34), 'C': (1.08, 4.84)}
                    shortest_path = tsp_manhattan(path_two)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    
                    print("Changing levels")
                    path_three = {'A': (0.0701, 3.34), 'B': (1.86, 5.3), 'C': (1.88, 8.16), 'D': (0.2, 7.59), 'E': (-.272, 5.69), 'F': (1.08, 4.84), 'G': (1.08, 4.84)}
                    shortest_path = tsp_manhattan(path_three)
                    navigator.navigate_to_multiple_points(shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    
                    print("Changing levels")
                    path_four = {'A': (-.0306, 5.83), 'B': (0.55, 9.48)}
                    shortest_path = tsp_manhattan(path_four)
                    navigator.navigate_to_multiple_points(path_one[node] for node in shortest_path)
                    # Sleep to give the last log messages time to be sent
                    rospy.sleep(1)
                    print("Tour has ended")
                else:
                    points = {}
                    while(1):
                        user_input = raw_input("Would you like to add a point? 1 = Yes, 2 = No\n")
                        if user_input == '1':
                            node_name = raw_input("Please enter highlight name:\n")
                            points[node_name] = navigator.check_highlight(node_name)
                        else:
                            break
                    if len(points) == 2:
                        shortest_path = tsp_manhattan(points)
                        navigator.navigate_to_multiple_points([points[node] for node in shortest_path])
                        rospy.sleep(1)
                    else:
                        path = tsp_manhattan(points)
                        navigator.navigate_to_multiple_points(path)
                        rospy.sleep(1)
                    print("Custom tour has ended")
            elif user_input == 'r':
                print("Going back to recharging station")
                position = {'x': .254, 'y' : -.287}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                success = navigator.goto(position, quaternion)

                if success:
                    rospy.loginfo("Hooray, reached the desired pose")
                else:
                    rospy.loginfo("The base failed to reach the desired pose")

                # Sleep to give the last log messages time to be sent
            elif user_input== 'q':
                print("Exiting tour robot mode......")
                print("Have a great day!!!")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


def main():
    global command
    rospy.init_node('audio_recorder_node', anonymous=True)
    recorder = AudioRecorder()
    # Initialize the teleop and obstacle avoidance classes


    rospy.sleep(1)

    soundhandle.say("Hello say oliver followed by a command")

    # print the default teleop message


    threshold = 0.6
    # while(1):
    #     #listened = listen()
    #     print("Listening: " +  str(recorder.user_question))
    #     if ("start" or "Start") in str(recorder.user_question):
    #         print("works")
    #         soundhandle.say("Starting Automode")
    #         break

    # print("You said start")
    
    global currentNode
    try:
        while(1):
            if command is "halt":
                soundhandle.say("We halted. Listening for new command")
                while(command is "halt"):   
                    rospy.sleep(0.1)

            elif command is "To_A":
                print("Go to apples")
                goToNode("apples")
            elif command is "To_Home":
                print("Go to home")
                goToNode("home")
            elif command is "To_B":
                print("Go to basketballs")
                goToNode("basketballs")
            elif command is "To_C":
                print("Go to cats")
                goToNode("cats")
            elif command is "To_D":
                print("Go to dogs")
                goToNode("dogs")
            elif command is "where":
                print("Where")
                whereAmI()
            elif command is "here_apples":
                print("here apples")
                youAreHere("apples")
            elif command is "here_basketballs":
                print("here basketballs")
                youAreHere("basketballs")
            elif command is "here_catss":
                print("here cats")
                youAreHere("cats")
            elif command is "here_dogs":
                print("here dogs")
                youAreHere("dogs")
            elif command is ("tour"):
                print("Starting tour")
                tour()

    except Exception as e:
        print(e)

if __name__=="__main__":
    main()

    
