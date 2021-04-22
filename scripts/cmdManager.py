#!/usr/bin/env python
import actionlib
import rospy 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from knowledgeRep import Rooms

# initialize the environment 
rooms = Rooms()
rooms.ROOMS[1]['detected'] = True
   

def HIcallback(data):
    global rooms
    msg = data.data
    print("command received", msg)

    if msg.startswith("GoTo"):
         target_room = msg.strip("GoTo ")
         if not rooms.check_visited(target_room):
             print("ROOM not VISITED")

    else:
         rospy.logerr("Parsing Error - No GoTo command ")
          

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        human_inteface = rospy.Subscriber("Interface_chatter", String, HIcallback)
    	rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
