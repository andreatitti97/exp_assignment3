#!/usr/bin/env python
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point, Pose

# Action Server 
import actionlib
import actionlib.msg
import exp_assignment3.msg


def main():
        rospy.init_node('action_server')

        goal = exp_assignment3.msg.ballTrackingGoal()

        goal.color = "blue"
        client = actionlib.SimpleActionClient('reaching_ball', exp_assignment3.msg.ballTrackingAction)
        rospy.loginfo("START the CLIENT")
        
	client.send_goal(goal)
        rospy.loginfo("GOAL SENDED")
        client.wait_for_result()
        rospy.loginfo("GOAL ACHIVED")
        result = client.get_result()
        rospy.loginfo("robot_position:")
        rospy.loginfo(result.x)
        rospy.loginfo(result.y)

if __name__ == "__main__":
    main()
