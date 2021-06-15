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

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from knowledgeRep import Rooms

import smach_msgs.msg 


# Action Server 
import actionlib
import actionlib.msg
from exp_assignment3.msg import ballTrackingGoal, ballTrackingAction

ctrl_var = {"PLAY" : False, "TARGET_ROOM" : "None", "NEW_ROOM_COLOR" : "None", "FIND_MODE" : False}

## init move_base client 
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
RD_pub = rospy.Publisher('room_detection', Bool, queue_size=10)
# init the environment 
rooms = Rooms()



def UIcallback(data):
    global ctrl_var, client, rooms, RD_pub
    if data.data == "PLAY" :
        rospy.loginfo("[cmdManager]: RECEIVED PLAY COMMAND!")
        ctrl_var["PLAY"] = True
        client.cancel_all_goals() #cancel all move_base goals
        RD_pub.publish(False)
        time.sleep(3)
    elif data.data == "list":
        rospy.loginfo("[cmdManager]: DISCOVERED ROOMS:")
        v_rooms = rooms.visited()
        for room in v_rooms:
            print(room)
    elif data.data.startswith("GoTo"):
        ctrl_var["TARGET_ROOM"] = data.data
        rospy.loginfo("[cmdManager]: DESIRED ROOM RECEIVED")
    else:
        rospy.logerr("[cmdMAnager]: WRONG MESSAGE FORMAT")


def newRoomDetected(color):
    global ctrl_var, client, rooms, RD_pub
    if not rooms.room_check(color.data):
        rospy.loginfo("[cmdManager]: NEW BALL DETECTED: %s COLOR", color.data)
        RD_pub.publish(False)
        ctrl_var["NEW_ROOM_COLOR"]= color.data
        client.cancel_all_goals()

def move_base_go_to(x, y):
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
        name = rooms.get_name_position(x, y)
        if ctrl_var["PLAY"] or ctrl_var["NEW_ROOM_COLOR"]:
            rospy.loginfo("[cmdManager]: MOVEBASE MISSION ABORTED")
        elif not name:
            rospy.loginfo("[cmdManager]:REACHED POSITION (%d,%d). WAIT...", x, y)
        else: 
            rospy.loginfo("[cmdManager]: REACHED THE %s . WAIT ...", name)            
        time.sleep(5)
 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
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
                rospy.loginfo("[cmdManager]: MAX NUMBER OF ITERATION going TO SLEEP")
                return 'goToSleep' 
            elif ctrl_var["NEW_ROOM_COLOR"] != "None":
                return 'goToTrack'
            else:
                # move to rand position
                rospy.loginfo("[cmdManager]: GENERATING A NEW RANDOM GOAL POSITION")
                pos = rooms.random_pos()                
                move_base_go_to(pos[0], pos[1])
                self.rate.sleep()
                self.counter += 1
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state which sleeps for a random period of time.
    Then it makes a request to the Navigation service to go to the home location.
    Finally it returns in the NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToSleep']) 

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
        smach.State.__init__(self, outcomes=['goToNormal','goToPlay','goToFind'])
        self.rate = rospy.Rate(1)
        self.counter = 0


    def execute(self, userdata):

        rospy.loginfo("***************** PLAY STATE **************")
        global ctrl_var, rooms
        time.sleep(1)
        ctrl_var["PLAY"] = False
        # Go to the user
        position = rooms.get_room_position("Home")
        move_base_go_to(position[0], position[1])	

        while not rospy.is_shutdown():
            if self.counter <= 5:
                if ctrl_var["TARGET_ROOM"] != "None":

                    if ctrl_var["TARGET_ROOM"].startswith("GoTo"):
                        ctrl_var["TARGET_ROOM"] = ctrl_var["TARGET_ROOM"].strip("GoTo ")
                        position = rooms.get_room_position(ctrl_var["TARGET_ROOM"])
                        if not position:
                            rospy.loginfo("[cmdManager]: ROOM NOT VISITED YET")                   
                            return 'goToFind'
                        else:
                            rospy.loginfo("[cmdanager]: ROOM ALREADY VISITED")
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
                    ctrl_var["TARGET_ROOM"] = "None"

                # wait a few seconds 
                time.sleep(3)
                rospy.loginfo("[cmdManager]: WAIT NEW LOCATION ...")
                self.counter += 1

            else:
                self.counter = 0
                return 'goToNormal'

class Track(smach.State):
    '''Class that defines the PLAY state. 
    It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToNormal','goToTrack','goToFind','goToPlay'])

    def execute(self, userdata):
        global rooms, ctrl_var
        rospy.loginfo("***************** TRACK STATE **************")
        goal = ballTrackingGoal()
        goal.color = ctrl_var["NEW_ROOM_COLOR"]

        trackClient = actionlib.SimpleActionClient('TrackAction',ballTrackingAction)
        trackClient.wait_for_server()
        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            if ctrl_var["FIND_MODE"] == True:
                return "goToFind"
            return 'goToNormal'

        else:

            result = trackClient.get_result()
            if result.x != 0 and result.y != 0:
                #rospy.loginfo("[cmdManager]: NEW ROOM DISCOVERED")
                #rospy.loginfo("[cmdManager] I'M IN POSITION (%d,%d)",result.x,result.y)
                rooms.add_new_room(ctrl_var["NEW_ROOM_COLOR"], result.x, result.y)
                if ctrl_var["FIND_MODE"] == True:
                    if ctrl_var["NEW_ROOM_COLOR"] == rooms.get_color_room(ctrl_var["TARGET_ROOM"]):
                        ctrl_var["FIND_MODE"] = False
                        rospy.loginfo("[cmdManager]: DISCOVERED DESIRED ROOM")
                        ctrl_var["NEW_ROOM_COLOR"] = "None"
                        return "goToPlay"
                    else:
                        rospy.loginfo("[cmdManager]: FOUND NOT THE DESIRED ROOM")
                        ctrl_var["NEW_ROOM_COLOR"] = "None"
                        return "goToFind"
            else: 
                rospy.loginfo("[cmdManager] NOT ABLE TO FIND PREVIOUS ROOM DETECTED")
                if ctrl_var["FIND_MODE"]:
                    ctrl_var["NEW_ROOM_COLOR"] = "None"
                    return "goToFind"
            ctrl_var["NEW_ROOM_COLOR"] = "None"
            return "goToNormal"

class Find(smach.State):
    '''Class that defines the PLAY state. 
    It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['goToPlay','goToTrack','goToFind'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0

    def execute(self, userdata):
        global rooms, ctrl_var
        rospy.loginfo("***************** FIND STATE **************")
        ctrl_var["FIND_MODE"] = True
        RD_pub.publish(True)
        time.sleep(4)
        
        while not rospy.is_shutdown():  

            if ctrl_var["NEW_ROOM_COLOR"] != "None":
                #rooms.cancel_room()
                return "goToTrack"
	    elif ctrl_var["PLAY"] == True:
		ctrl_var["FIND_MODE"] = False
                RD_pub.publish(False)
                self.counter = 0
		return "goToPlay"
            elif self.counter == 5:
                rospy.loginfo("[cmdManager]: MAXNUMBER OF FIND MODE ITERATIONs")
                ctrl_var["FIND_MODE"] = False
                RD_pub.publish(False)
                self.counter = 0
                return 'goToPlay' 
            else:

                rospy.loginfo("[cmdManager] EXPLORATION")
                pos = rooms.explore()
                move_base_go_to(pos[0], pos[1])
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
        time.sleep(2)
        newRoomSub = rospy.Subscriber("new_room_found", String, newRoomDetected)

        # Create a SMACH state machine
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
