#!/usr/bin/env python  
import rospy 
from std_msgs.msg import String
import time

def humanInterface():
    rospy.init_node('Human Interface', anonymous=True)
    publisher = rospy.Publisher('Interface_chatter',String,queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = raw_input("Type PLAY")
        publisher.publish(msg)
        rate.sleep()
	msg = raw_input("Type GoTo <room_name>")
	publisher.publish(msg)
	time.sleep(2)

if __name__ == "__main__":
    try:
        humanInterface()
    except rospy.ROSInterruptException:
        rospy.loginfo("Human Interface terminated")
    
