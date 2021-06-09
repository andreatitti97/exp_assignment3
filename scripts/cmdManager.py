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

## init move_base client 
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

# init the environment 
rooms = Rooms()



def UIcallback(data):
    global PLAY, TARGET_ROOM, NEW_TR, client, rooms
    if data.data == "PLAY" :
        rospy.loginfo("Recived a 'play' request...stop every move_base goal!!")
        client.cancel_all_goals()
        time.sleep(3)
        PLAY = True

    elif data.data.startswith("GoTo"):
        NEW_TR = True
        TARGET_ROOM = data.data
        rospy.loginfo("I recived the desired room: %s", TARGET_ROOM)
    else:
        rospy.logerr("[Syntax Error] the sent msg is wrong")


def newRoomDetected(color):
    global NEW_ROOM, COLOR_ROOM, client
    rospy.loginfo("[CmdMg] reach a new ball, start track it !")
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

    rospy.loginfo("[CommandManager] I'm going to position x = %d y = %d", x, y)

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("[CommandManager] [Move Base] Action server closed. wait")
        time.sleep(3)

def random_pos():
    while True:
        tmpX = random.randint(-5,5)
        tmpY = random.randint(-5,5)
    if not (tmpX > 0 and tmpY > 3):
        return [tmpX, tmpY]

 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay','goToTrack'])

        ## init move_base goal 
        #self.goal = MoveBaseGoal()
        #self.goal.target_pose.header.frame_id = "map"
        #self.goal.target_pose.header.stamp = rospy.Time.now()

        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
        global PLAY, client, NEW_ROOM
        rospy.loginfo("NORMAL STATE")
        time.sleep(4)
        self.counter = 0
        while not rospy.is_shutdown():  

            if PLAY == True:
                return 'goToPlay'
            elif self.counter == 4:
                return 'goToSleep' 
            elif NEW_ROOM == True:
                print("prova")
                return 'goToTrack'
            else:
                # move to rand position
                rospy.loginfo("[CommandManager] generate a new random goal position")
                pos = random_pos()                
                move_base_go_to(pos[0], pos[1])
                self.rate.sleep()

                self.counter += 1
                print(counter)
                
            return 'goToSleep' 
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state which sleeps for a random period of time.
    Then it makes a request to the Navigation service to go to the home location.
    Finally it returns in the NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep']) 
                             
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata): 
        global rooms
	rospy.loginfo("***********************************")
        rospy.loginfo("[CommandManager] I m in SLEEP mode")
        # comeback to home      
        position = rooms.room_check("Home")
	move_base_go_to(position[0], position[1])
     
	rospy.loginfo("[CommandManager] Home reached")
        # sleep for a random time period
	rospy.loginfo("[CommandManager] Sleeping...")
	time.sleep(random.randint(3,6))

        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)
        self.counter = 0
	    

    def execute(self, userdata):

        rospy.loginfo("I m in PLAY mode")
	global PLAY, rooms

        

        # go to the person
        #userdata.rooms_in.go_to_room("Home")
        position = rooms.room_check("Home")
        move_base_go_to(position[0], position[1])	
        rospy.loginfo("Home Reached!!!!")

        while not rospy.is_shutdown():
	    # we need to update them at each iteration 
            global TARGET_ROOM, NEW_TR

            if self.counter <= 5:
		if NEW_TR == True:
                                        
                    if TARGET_ROOM.startswith("GoTo"):
                        TARGET_ROOM = TARGET_ROOM.strip("GoTo ")
                        position = rooms.room_check(TARGET_ROOM)
                        if not position:
                            rospy.loginfo("That room has not yet been visited")                   
                        else:
                            move_base_go_to(position[0], position[1])
                    else: 
                    	rospy.logerr("NO GoTo command typed")
    
                    # wait a few seconds 
                    time.sleep(1)
                    rospy.loginfo("homecoming...")
                    # comebak to the person 
                    position = rooms.room_check("Home")
                    rospy.loginfo("Home Reached!!!!")
                    NEW_TR = False
                    
                # wait a few seconds 
                time.sleep(3)
                rospy.loginfo("wait-..........")
                self.counter += 1
                    
            else:
                self.counter = 0
                PLAY = False
                return 'goToNormal'

class Track(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToTrack'])

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM
	rospy.loginfo("***********************************")
	rospy.loginfo("[CommandManager] I'm in TRACK state")
        goal = ballTrackingGoal()
        goal.color = COLOR_ROOM

        trackClient = actionlib.SimpleActionClient('trackAction',trackBallAction)
        trackClient.wait_for_server()
        rospy.loginfo("Track client creato")

        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            NEW_ROOM = False
            return 'goToNormal'

        else:
            rospy.loginfo("[CommandManager] New Room reached!")
            result = trackClient.get_result()
            rospy.loginfo("[CommandManager] la posizione attuale e':")
            rospy.loginfo(result.x)
            rospy.loginfo(result.y)
            rooms.add_new_room(COLOR_ROOM, result.x, result.y)
	    ###### add the new room to the list ######
            NEW_ROOM = False
            return "goToNormal"

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
            smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToPlay':'PLAY',
						                        'goToTrack' : 'TRACK',
                                                'goToNormal':'NORMAL'})
            smach.StateMachine.add('SLEEP', Sleep(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToNormal':'NORMAL'})
            smach.StateMachine.add('PLAY', Play(), 
                                transitions={'goToNormal':'NORMAL',
                                                'goToPlay':'PLAY'})
            smach.StateMachine.add('TRACK', Track(), 
                                transitions={'goToNormal':'NORMAL',
                                                'goToTrack':'TRACK'})


        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    main()
