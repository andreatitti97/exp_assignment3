#!/usr/bin/env python
## @file commandManager.py
# Inside this script is handled all the logic of the program, in fact implement the Finite State Machine 
# for switching between the states: NORMAL, SLEEP, PLAY, FIND, TRACK.
# Is the core of the architecture, receive information coming from the human interface and from the detection node 
# ( subscribe to the topics /inteface_chatter and /new_room_found), and also publish on /room_detection topic whenever start
# or stop the detection. Finally it also communicate with the move_base action server for moving the robot.

# Python and Ros Libraries 
from __future__ import print_function
import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
# Ros msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from knowledgeRep import Rooms
import smach_msgs.msg 
# Action Server 
import actionlib
import actionlib.msg
from exp_assignment3.msg import ballTrackingGoal, ballTrackingAction

## Initialized the client to the move_base action server for moving the robot.
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
## Initialized publisher of the detection state which is a boolean (active or not).
RD_pub = rospy.Publisher('detection_state', Bool, queue_size=10) 
## Initialized object of the class Rooms() for use the knowledge representation defined in knowledgeRep.py
rooms = Rooms()
## Dictionary that store important control variables and flags.
# - PLAY: Flag which notify if there is a PLAY request
# - TARGET_ROOM: Control variable which contain the desired room given by the user
# - NEW_COLOR: Control variable which contatin a color corresponding to the las detected ball.
# - FIND: Flag which notify if we are in FIND state or not. 
ctrl_var = {"PLAY" : False, "TARGET_ROOM" : "None", "NEW_COLOR" : "None", "FIND" : False}

## Callback of the ball_detection_subscriber, is called any time a new colored ball is found, precisely a new color is given in input
# and then the function check if the color was already detected, if not the move_base client is stopped for start the tracking.
# @param color is the color detected (string) by the detection algorithm.
def detection_routine(color):
    global ctrl_var, client, rooms, RD_pub
    if not rooms.room_check(color.data):
        rospy.loginfo("[cmdManager]: NEW BALL DETECTED: %s COLOR", color.data)
        RD_pub.publish(False)
        ctrl_var["NEW_COLOR"]= color.data
        client.cancel_all_goals()
## Callback of the human interface, each time a user command is received the function change the PLAY flag and the message is parsed.
# @param data is the message (string).
def UIcallback(data):
    global ctrl_var, client, rooms, RD_pub
    if data.data == "PLAY" :
        rospy.loginfo("[cmdManager]: RECEIVED PLAY COMMAND!")
        ctrl_var["PLAY"] = True
        client.cancel_all_goals() #cancel all move_base goals
        RD_pub.publish(False)
        time.sleep(3)
    elif data.data.startswith("GoTo"):
        ctrl_var["TARGET_ROOM"] = data.data
        rospy.loginfo("[cmdManager]: DESIRED ROOM RECEIVED")
    else:
        rospy.logerr("[cmdMAnager]: WRONG MESSAGE FORMAT")

## Function which implement the navigation to given input position using the move_base action server.
# @param x X goal position.
# @param y Y goal position.
def go_to(x, y):
    global client, ctrl_var
    ## init move_base goal 
    goal = MoveBaseGoal()
    ## set the goal as a random position 
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 2.0

    rospy.loginfo("[cmdManager]: MOVING TO POS: x = %d y = %d", x, y)

    client.send_goal(goal)
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available")
        rospy.signal_shutdown("Action server not available")
    else:
        name = rooms.room_name(x, y)
        if ctrl_var["PLAY"] or ctrl_var["NEW_COLOR"] != "None":
            rospy.loginfo("[cmdManager]: MOVEBASE MISSION ABORTED")
        elif not name:
            rospy.loginfo("[cmdManager]:REACHED POSITION (%d,%d). WAIT...", x, y)
        else: 
            rospy.loginfo("[cmdManager]: REACHED THE %s . WAIT ...", name)            
        time.sleep(5)
 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In this state random positions are sended to the robot while the PLAY flag
    is monitored since if we receive a PLAY command from the user we have to switch state. Moreover it's checked each iteration if 
    a new color is detected for switching to TRACK state and also after a predefined number of iteration switch to SLEEP state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToSleep','goToPlay','goToTrack'], input_keys=['foo_counter_in'], output_keys=['foo_counter_out'])
        
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	    ## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
        global ctrl_var, client, rooms, RD_pub
        rospy.loginfo("***************** NORMAL STATE **************")   
        RD_pub.publish(True)
        while not rospy.is_shutdown():  
            time.sleep(2)
            if ctrl_var["PLAY"] == True:
                return 'goToPlay'
            elif self.counter == 5:
                rospy.loginfo("[cmdManager--NORMAL]: MAX NUMBER OF ITERATION going TO SLEEP")
                return 'goToSleep' 
            elif ctrl_var["NEW_COLOR"] != "None":
                return 'goToTrack'
            else:
                # move to rand position
                rospy.loginfo("[cmdManager--NORMAL]: GENERATING A NEW RANDOM GOAL POSITION")
                pos = rooms.random_pos()                
                go_to(pos[0], pos[1])
                self.rate.sleep()
                self.counter += 1
        
    


class Sleep(smach.State):
    '''Class which define the SLEEP state of the FSM. In this state the robot go to the home position and sleep randomly, after that
    the FSM return to NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToSleep']) 

        self.rate = rospy.Rate(200)  # Loop at 200 Hz
        
    def execute(self, userdata): 
        global rooms
        rospy.loginfo("***************** SLEEP STATE **************")
        # comeback to home      
        position = rooms.room_position("Home")
        go_to(position[0], position[1])

        rospy.loginfo("[cmdManager--SLEEP]: REACHED HOME - SLEEPING")
        time.sleep(random.randint(3,6))

        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state of the FSM. After receiving a PLAY we entered in this state where the robot first goes to the user,
    the wait for a target room to reach. If the target room was already discoverd the position associated in the knowledege representation
    is reachd, else the robot switch to FIND state. After a while (we count the iterations inside this state) the robot return to NORMAL state.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToPlay','goToFind'])
        self.rate = rospy.Rate(1)
        self.counter = 0


    def execute(self, userdata):
        global ctrl_var, rooms
        rospy.loginfo("***************** PLAY STATE **************")
        time.sleep(1)        
        ctrl_var["PLAY"] = False
        position = rooms.room_position("Home")
        go_to(position[0], position[1])	

        while not rospy.is_shutdown():
            if self.counter <= 5:
                if ctrl_var["TARGET_ROOM"] != "None":
                    if ctrl_var["TARGET_ROOM"].startswith("GoTo"):
                        ctrl_var["TARGET_ROOM"] = ctrl_var["TARGET_ROOM"].strip("GoTo ")
                        position = rooms.room_position(ctrl_var["TARGET_ROOM"])
                        if not position:
                            rospy.loginfo("[cmdManager--PLAY]: ROOM NOT VISITED YET")                   
                            return 'goToFind'
                        else:
                            rospy.loginfo("[cmdanager--PLAY]: ROOM ALREADY VISITED")
                            go_to(position[0], position[1])
                    else: 
                        rospy.logerr("NO GoTo <room_name> COMMAND TYPED")

                    # wait a few seconds 
                    time.sleep(1)
                    rospy.loginfo("[cmdManager--PLAY] GOING BACK TO HOME")
                    # comebak to the person 
                    position = rooms.room_position("Home")
                    go_to(position[0], position[1])
                    rospy.loginfo("[cmdManager--PLAY] HOME REACHED")
                    ctrl_var["TARGET_ROOM"] = "None"

                # wait a few seconds 
                time.sleep(3)
                rospy.loginfo("[cmdManager--PlAY]: WAIT NEW TARGET LOCATION")
                self.counter += 1

            else:
                self.counter = 0
                return 'goToNormal'

class Find(smach.State):
    '''Class which define the FIND State of the FSM. In this state the robot explore the area for the requested unknown location during PLAY state.
    If a new ball is found then switch to TRACK state, if the ball correspond to the requested room in PLAY state is notified to the user, otherwise 
    the new room detected is added in the knowledge. After a while return to PLAY state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToPlay','goToTrack','goToFind'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0

    def execute(self, userdata):
        global rooms, ctrl_var
        rospy.loginfo("***************** FIND STATE **************")
        ctrl_var["FIND"] = True
        RD_pub.publish(True)
        time.sleep(4)
        
        while not rospy.is_shutdown():  

            if ctrl_var["NEW_COLOR"] != "None":
                return "goToTrack"
            elif ctrl_var["PLAY"] == True:
                ctrl_var["FIND"] = False
                RD_pub.publish(False)
                self.counter = 0
                return "goToPlay"
            elif self.counter == 5:
                rospy.loginfo("[cmdManager--FIND]: MAXNUMBER OF FIND MODE ITERATIONs")
                ctrl_var["FIND"] = False
                RD_pub.publish(False)
                self.counter = 0
                return 'goToPlay' 
            else:
                rospy.loginfo("[cmdManager--FIND]: EXPLORATION")
                pos = rooms.room_explore()
                go_to(pos[0], pos[1])
                self.rate.sleep()
                self.counter += 1


class Track(smach.State):
    '''Class which define the TRACK stat of the FSM. Inside this state is initialized the client to the tracking action server
    (implemented in trackingBall.py) which make the request of track the previuosly detected ball.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToTrack','goToFind','goToPlay'])

    def execute(self, userdata):
        global rooms, ctrl_var
        rospy.loginfo("***************** TRACK STATE **************")
        goal = ballTrackingGoal()
        goal.color = ctrl_var["NEW_COLOR"]

        trackClient = actionlib.SimpleActionClient('Tracking',ballTrackingAction)
        trackClient.wait_for_server()
        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            if ctrl_var["FIND"] == True:
                return "goToFind"
            return 'goToNormal'

        else:

            result = trackClient.get_result()
            if result.x != 0 and result.y != 0:
                rooms.new_room(ctrl_var["NEW_COLOR"], result.x, result.y)
                if ctrl_var["FIND"] == True:
                    if ctrl_var["NEW_COLOR"] == rooms.room_color(ctrl_var["TARGET_ROOM"]):
                        ctrl_var["FIND"] = False
                        rospy.loginfo("[cmdManager--TRACK]: DISCOVERED DESIRED ROOM")
                        ctrl_var["NEW_COLOR"] = "None"
                        return "goToPlay"
                    else:
                        rospy.loginfo("[cmdManager--TRACK]: NOT FOUND THE DESIRED ROOM")
                        ctrl_var["NEW_COLOR"] = "None"
                        return "goToFind"
            else: 
                rospy.loginfo("[cmdManager--TRACK]: NOT FOUND THE PREVIOUSLY DETECT ROOM")
                if ctrl_var["FIND"]:
                    ctrl_var["NEW_COLOR"] = "None"
                    return "goToFind"
            ctrl_var["NEW_COLOR"] = "None"
            return "goToNormal"

def main():

    try:
        rospy.init_node('Cmd_Manager_Node')
        # Init move_base client
        client.wait_for_server()
        ## Subscriber to the human interface topic, receive user commands.
        UIsubscriber = rospy.Subscriber("Interface_chatter", String, UIcallback) 
        time.sleep(2)
        ## Subscriber to the /color_found topic, in which a detected color (string) is sended to the detection routine.
        ball_detection_subscriber = rospy.Subscriber("color_found", String, detection_routine)
        ## Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['init'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('NORMAL', Normal(), transitions={'goToSleep':'SLEEP','goToPlay':'PLAY','goToTrack' : 'TRACK','goToNormal':'NORMAL'}, remapping={'NORMAL_counter_in':'normal_counter','NORMAL_counter_out':'normal_counter'})
            smach.StateMachine.add('SLEEP', Sleep(), transitions={'goToSleep':'SLEEP','goToNormal':'NORMAL'})
            smach.StateMachine.add('PLAY', Play(), transitions={'goToNormal':'NORMAL','goToPlay':'PLAY','goToFind':'FIND'})
            smach.StateMachine.add('TRACK', Track(), transitions={'goToNormal':'NORMAL','goToTrack':'TRACK','goToPlay':'PLAY','goToFind':'FIND'})
            smach.StateMachine.add('FIND', Find(), transitions={'goToTrack':'TRACK','goToPlay':'PLAY','goToFind':'FIND'})

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    main()
