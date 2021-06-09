#!/usr/bin/env python
import actionlib
import rospy 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Rooms():

    def __init__(self):
        self.ROOMS = [ 
        {'name':"Entrance",'colour': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'colour': "red", "x":0, "y":0, 'detected':False},
        {'name':"LeavingRoom",'colour': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'colour': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'colour': "orange", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'colour':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'colour':"","x":-5,"y":7, 'detected':True}		           
        ]

    
# check if the room contained in msg is already visited, if so then the robot will move to that position 
    def room_check(self, target_room):
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
                    print("ROOM ALREADY VISITED")
		    return [room["x"], room["y"]]
        return False
        
    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['colour']:
                room['detected'] = True
                room['x'] = x
                room['y'] = y
		print("[ROOMS] discovered room:",room['name'])

    def move_to(self, x, y):
    	goal = MoveBaseGoal()
    	goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
    	goal.target_pose.position.x = x
	goal.target_pose.position.y = y
	goal.target_pose.pose.orientation.w = 1.0
	self.client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
        	rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
		return False
	else:
		rospy.loginfo("ROOM REACHED")
		return True                    
    def stop_moving(self):
	self.client.cancel_all_goals()   	
