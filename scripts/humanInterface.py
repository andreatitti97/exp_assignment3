#!/usr/bin/env python  
import rospy 
from std_msgs.msg import String
import time

def humanInterface():
    rospy.init_node('Human Interface', anonymous=True)
    publisher = rospy.Publisher('Interface_chatter',String,queue_size=20)
    rate = rospy.Rate(10)
    print("Interface for the robot \n\n Digit PLAY (switch play mode) \n followed by GoTo <room_name>")

    while not rospy is_shutdown():
        msg = raw_input("")
        publisher.pub(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        humanInterface()
    except rospy.ROSInterruptException:
        rospy.loginfo("Human Interface terminated")
    