#!/usr/bin/env python  
## @file humanInterface.py
# This script implement the user interface for interact with the robot entering the PLAY state, where we can send a target room to the robot to reach.
import rospy 
from std_msgs.msg import String
import time

## Definition of the human interface. 
def humanInterface():
    ## Publisher to /interface_chatter topic, where string containing the user digit are sended to the command manager.
    publisher = rospy.Publisher('Interface_chatter',String,queue_size=10)
    rate = rospy.Rate(10)
    print("WELCOME TO THE USER INTERFACE OF THE ROBOT \n Digit 'PLAY' for enter the INTERACTION MODE \n Digit 'GoTo <room_name>' for MOVE THE ROBOT TO THE SELECTED ROOM or FIND THE SELECTED ROOM \n KNOWLEDGE REPRESENTATION: Entrance -> BLUE, Closet -> RED, LeavingRoom -> GREEN, Kitchen -> YELLOW, BedRoom -> BLACK \n")

    while not rospy.is_shutdown():
        msg = raw_input("User:")
        publisher.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('Human_Interface', anonymous=True)
    try:
        humanInterface()
    except rospy.ROSInterruptException:
        rospy.loginfo("HUMAN INTERFACE CLOSED")
    
