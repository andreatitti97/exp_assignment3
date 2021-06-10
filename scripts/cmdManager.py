#!/usr/bin/env python

## @file commandManager.py
#  This node includes the subsription to State and GetPosition publishers,
#  And implement a finite state machine 
#  which manages the information coming from the two publisher and changes the state of the system in according to them.
# \see getPosition.cpp
# \see Navigation.cpp
# \see State.cpp
 

from __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
import random 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from knowledgeRep import Rooms

import smach_msgs.msg 


# Action Server 
import actionlib
import actionlib.msg
from exp_assignment3.msg import ballTrackingGoal, ballTrackingAction

# Flag which notify when the user types 'play'
PLAY = False

# Contains the desired room entered by the user 
TARGET_ROOM = "None"

# Flag which notify when the command manager recives a new target room
NEW_TR = False

NEW_ROOM = False

RANDOM =  True

COLOR_ROOM = "None"

FIND_MODE = False
## init move_base client 
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

# init the environment 
rooms = Rooms()



def UIcallback(data):
    global PLAY, TARGET_ROOM, NEW_TR, client, rooms
    if data.data == "PLAY" :
        rospy.loginfo("[cmdManager]: RECEIVED PLAY COMMAND!")
        client.cancel_all_goals() #cancel all move_base goals
        time.sleep(3)
        PLAY = True

    elif data.data.startswith("GoTo"):
        NEW_TR = True
        TARGET_ROOM = data.data
        rospy.loginfo("[cmdManager]: DESIRED ROOM TO GO:] %s", TARGET_ROOM)
    else:
        rospy.logerr("[cmdMAnager]: WRONG MESSAGE FORMAT")


def newRoomDetected(color):
    global NEW_ROOM, COLOR_ROOM, client, rooms
    if not rooms.room_check(color.data):
        rospy.loginfo("[cmdManager]: NEW BALL DETECTED: %s COLOR", color.data)
        NEW_ROOM = True
        COLOR_ROOM = color.data
        client.cancel_all_goals()

def move_base_go_to(x, y):
    global client
    ## init move_base goal 
    goal = MoveBaseGoal()
    ## set the goal as a random position 
    goal.target_pose.header.frame_id = "odom"
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
        name = rooms.get_name_position(x, y)
        if not name:
            rospy.loginfo("[cmdManager]:REACHED POSITION (%d,%d). WAIT...", x, y)
        else: 
            rospy.loginfo("[cmdManager]: REACHED THE %s . WAIT ...", name)            
        time.sleep(3)
 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToSleep','goToPlay','goToTrack'])
        
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	    ## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
        global PLAY, client, NEW_ROOM, rooms
        rospy.loginfo("***************** NORMAL STATE **************")
        time.sleep(4)
        self.counter = 0
        while not rospy.is_shutdown():  

            if PLAY == True:
                return 'goToPlay'
            elif self.counter == 4:
                return 'goToSleep' 
            elif NEW_ROOM == True:
                return 'goToTrack'
            else:
                # move to rand position
                rospy.loginfo("[cmdManager]: GENERATING A NEW RANDOM GOAL POSITION")
                pos = rooms.random_pos()                
                move_base_go_to(pos[0], pos[1])
                self.rate.sleep()

                self.counter += 1
                #print(counter)
                
        return 'goToSleep' 
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state which sleeps for a random period of time.
    Then it makes a request to the Navigation service to go to the home location.
    Finally it returns in the NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToSleep', 'goToFind']) 

        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata): 
        global rooms
        rospy.loginfo("***************** SLEEP STATE **************")
        # comeback to home      
        position = rooms.get_room_position("Home")
        move_base_go_to(position[0], position[1])

        rospy.loginfo("[cmdManager]: REACHED HOME")
        # sleep for a random time period
        rospy.loginfo("[CommandManager] SLEEP ... ")
        time.sleep(random.randint(3,6))

        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state. 
    It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToPlay'])
        self.rate = rospy.Rate(200)
        self.counter = 0


    def execute(self, userdata):

        rospy.loginfo("***************** PLAY STATE **************")
        global PLAY, rooms
        # Go to the user
        position = rooms.get_room_position("Home")
        move_base_go_to(position[0], position[1])	

        while not rospy.is_shutdown():
            # we need to update them at each iteration 
            global TARGET_ROOM, NEW_TR

            if self.counter <= 5:
                if NEW_TR == True:
                    if TARGET_ROOM.startswith("GoTo"):
                        TARGET_ROOM = TARGET_ROOM.strip("GoTo ")
                        position = rooms.get_room_position(TARGET_ROOM)
                        if not position:
                            rospy.loginfo("[cmdManager]: ROOM NOT VISITED YET")                   
                            return 'goToFind'
                        else:
                            move_base_go_to(position[0], position[1])
                    else: 
                        rospy.logerr("NO GoTo command typed")

                    # wait a few seconds 
                    time.sleep(1)
                    rospy.loginfo("[cmdManager] GOING BACK TO HOME")
                    # comebak to the person 
                    position = rooms.get_room_position("Home")
                    move_base_go_to(position[0], position[1])
                    rospy.loginfo("[cmdManager] HOME REACHED")
                    NEW_TR = False

                # wait a few seconds 
                time.sleep(3)
                rospy.loginfo("[cmdManager]: WAIT NEW LOCATION ...")
                self.counter += 1

            else:
                self.counter = 0
                PLAY = False
                return 'goToNormal'

class Track(smach.State):
    '''Class that defines the PLAY state. 
    It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToTrack','goToFind','goToPlay'])

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM, NEW_TR, FIND_MODE
        rospy.loginfo("***************** TRACK STATE **************")
        goal = ballTrackingGoal()
        goal.color = COLOR_ROOM

        trackClient = actionlib.SimpleActionClient('trackAction',ballTrackingAction)
        trackClient.wait_for_server()
        rospy.loginfo("[cmdManager]: CLIENT FOR TRACKING CREATED")

        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            NEW_ROOM = False
            return 'goToNormal'

        else:

            result = trackClient.get_result()
            if result.x != 0 and result.y != 0:
                rospy.loginfo("[cmdManager]: NEW ROOM DISCOVERED")
                rospy.loginfo("[cmdManager] I'M IN POSITION (%d,%d)",result.x,result.y)
                rooms.add_new_room(COLOR_ROOM, result.x, result.y)
                if FIND_MODE == True:
                    if COLOR_ROOM == NEW_TR:
                        FIND_MODE = False
                        #return 'goToPlay'
                        'goToPlay'
                    else:
                        #return 'goToFind'
                        'goToFind'
            else: 
                rospy.loginfo("[cmdManager] NOT ABLE TO FIND PREVIOUS ROOM DETECTED")
            ###### add the new room to the list ######
            NEW_ROOM = False
            return "goToNormal"

class Find(smach.State):
    '''Class that defines the PLAY state. 
    It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToPlay','goToTrack','goToFind'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM, FIND_MODE
        rospy.loginfo("***************** TRACK STATE **************")
        FIND_MODE = True
        time.sleep(4)
        self.counter = 0
        
        while not rospy.is_shutdown():  

            if self.counter == 4:
                FIND_MODE = False
                return 'goToPlay' 
            elif NEW_ROOM == True:
                return 'goToTrack'
            else:
                #move_base_go_to(-2, 8)

                # move in a random position using move_base
                rospy.loginfo("[cmdManager] EXPLORATION")
                #move_base_go_to(random.randint(-5,5), random.randint(-5,5))
                pos = rooms.explore()
                move_base_go_to(pos[0], pos[1])
                #rospy.loginfo("[CommandManager] position:( %d,%d) reached !!!", pos[0], pos[1])
                self.rate.sleep()

                self.counter += 1

def main():
    
    #global client
    try:
        rospy.init_node('cmd_manager')
        # move_base client
        client.wait_for_server()
        # Subscriber to the UIchatter topic
        UIsubscriber = rospy.Subscriber("Interface_chatter", String, UIcallback)
        # Subscriber to the newRoom topic 
        newRoomSub = rospy.Subscriber("new_room_found", String, newRoomDetected)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['init'])
    

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('NORMAL', Normal(), transitions={'goToSleep':'SLEEP','goToPlay':'PLAY','goToTrack' : 'TRACK','goToNormal':'NORMAL'})
            smach.StateMachine.add('SLEEP', Sleep(), transitions={'goToSleep':'SLEEP','goToNormal':'NORMAL'})
            smach.StateMachine.add('PLAY', Play(), transitions={'goToNormal':'NORMAL','goToPlay':'PLAY'})
            smach.StateMachine.add('TRACK', Track(), transitions={'goToNormal':'NORMAL','goToTrack':'TRACK'})
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
