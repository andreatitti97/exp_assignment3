#!/usr/bin/env python  
import rospy 
from std_msgs.msg import String
import time

def humanInterface():
    rospy.init_node('Human Interface', anonymous=True)
    publisher = rospy.Publisher('Interface_chatter',String,queue_size=10)
    rate = rospy.Rate(10)
    print("Welcome !!!! \n The user can digit the keyword: \n - 'play' -> to switch in play mode \n - 'GoTo roomName' -> to reach that room or to start looking for it (if it hasn't yet been discovered) \n The rooms present are: Entrance(blue), Closet(red), LeavingRoom(green), Kitchen(yellow), BathRoom(orange), BedRoom(black)\n")

    while not rospy.is_shutdown():
	    msg = raw_input("User:")
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        humanInterface()
    except rospy.ROSInterruptException:
        rospy.loginfo("Human Interface terminated")
    
