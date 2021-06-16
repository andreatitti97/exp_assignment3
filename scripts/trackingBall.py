#!/usr/bin/env python
## @file trackingBall.py
# This script is used for tracking the balls after the detection, and is responsable of the movement towards the ball of the robot.
# Implements an action server which is requested by the Cmd_Manager_Node anytime a new ball is detected

# Python Libraries 
import sys
import time
import imutils
import roslib
import rospy
import numpy as np
from scipy.ndimage import filters
# OpenCV
import cv2
# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# Action Server 
import actionlib
import actionlib.msg
import exp_assignment3.msg


# COLORS RANGES FOR APPLY THE MASK
blackLower = (0, 0, 0)
blackUpper = (5,50,50)
redLower = (0, 50, 50)
redUpper = (5, 255, 255)
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (70, 255, 255)
blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)


## Class used for tracking the balls detected by ballDetect.py. After receiving the request from the client (Cmd_Manager_Node) the script 
# start to track the ball of the color sended by the client. It also save the robot position subscribing to /odom topic for store the room position.
# Sice during the tracking the move_base action is aborted, it's also implemented a simple obstacle avoidance using the "bug algorithm" for
# prevent the robot collide with obstacle during the approaching to the ball.
class Tracking(object):

    def __init__(self, name):
        ## NAME OF THE SERVER
        self.action_name = name
        self.act_s = actionlib.SimpleActionServer('Tracking', exp_assignment3.msg.ballTrackingAction, self.track, auto_start=False)
        self.act_s.start()
        self.feedback = exp_assignment3.msg.ballTrackingFeedback()
        self.result = exp_assignment3.msg.ballTrackingResult()
        ## DICT FOR OBSTACLE AVOIDANCE ALGORITHM (store the regions of the lidar for be able to localize the obstacle w.r.t. the robot)
        self.lidar_regions = {'r':0,'fr':0, 'f':0,'fl':0,'l':0,}
        ## BOOLEAN OF ACKNOWLEDGMENT POSITIVO IF THE MISSION IS FINISH CORRECTLY
        self.ACK_POSITIVO = False
        ## COUNTER FOR MONITOR THE TRACKING WHILE THE BALL IS LOST
        self.lost_ball_counter = 0
        # BOOLEAN OF ACKNOWLEDGMENT NEGATIVO FOR ABORT MISSION IN SOME DANGEROUS SITUATION
        self.ACK_NEGATIVO = False
        ## RADIUS OF THE BALL IN THE IMAGE
        self.radius = 0
        ## INITIALIZATION OF THE VELOCITY PUBLISHER TO THE WHEELS
        self.vel_publisher = 0
        ## INIT ROBOT POSITION
        self.position = Point()
        ## INIT ROBOT POSE
        self.pose = Pose()
        ## PARAMETER FOR CORRECT LINEAR VELOCITY DURING TRACKING (for smooth movements)
        self.sigmaLinear = 0.001
        ## PARAMETER FOR CORRECT ANGULAR VELOCITY DURING TRACKING (for smooth movements)
        self.sigmaAngular = 0.002
    
    ## Callbak to the /odom subscriber (update pose and position of the robot)
    def odom_clbk(self, msg):
        self.pose = msg.pose.pose
        self.position = msg.pose.pose.position

    ## Callback of the "camera1/image_raw/compressed" subscriber, allows the robot to move towards the ball.
    def go_to_ball(self, ros_image):
        ## @param image_np (decompresed image and converted to CV2)
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  
        ## Reduce noise
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        ## Conversion to hsv    
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Apply the proper color mask
        if self.color == "black":
            mask = cv2.inRange(hsv, blackLower, blackUpper)
        elif self.color == "red":
            mask = cv2.inRange(hsv, redLower, redUpper)
        elif self.color == "yellow":
            mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        elif self.color == "green":
            mask = cv2.inRange(hsv, greenLower, greenUpper)
        elif self.color == "blue":
            mask = cv2.inRange(hsv, blueLower, blueUpper)
        '''elif self.color == "magenta":
            mask = cv2.inRange(hsv, magentaLower, magentaUpper)'''

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if self.radius > 10:
                cv2.circle(image_np, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                # SETTING VELOCITIES TO APPLIED TO THE ROBOT
                vel = Twist()
                # ANGULAR VELOCITY IS COMPUTED FROM THE MISALIGNMENT BETWEEN THE CENTER OF THE BALL AND THE CENTER OF THE IMAGE 
                vel.angular.z = -self.sigmaAngular*(center[0]-400)
                # LINEAR VELOCITY IS COMPUTED FROM THE DIMENSION OF THE RADIUS OF THE BALL IN THE IMAGE 
                vel.linear.x = -self.sigmaLinear*(self.radius-130)
                self.vel_publisher.publish(vel)
                rospy.loginfo("[trackingBall]: TRACKING ")
                # THRESHOLD FOR CONSIDERING THE BALL AS REACHED
                if (self.radius>=120) and (abs(center[0]-400)<5): 
                    rospy.loginfo("ballDetection --> BALL REACHED")
                    self.result.x = self.position.x
                    self.result.y = self.position.y
                    # SEND ACK_POSITIVO OF MISSION ACCOMPLISHED
                    self.ACK_POSITIVO = True

        else:
            # Routine that happend if the ball is lost during tracking, simplty rotate the robot to find the lost ball. After a while abort mission.
            rospy.loginfo("[trackingBall]: BALL NOT FOUND")
            vel = Twist()
            if self.lost_ball_counter <= 10:
                rospy.loginfo("[trackingBall]: TURN RIGHT SEARCHING THE BALL")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            elif self.lost_ball_counter < 20:
                rospy.loginfo("[trackingBall]: TURN LEFT SEARCHING THE BALL")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.lost_ball_counter == 20:
                rospy.loginfo("[trackingBall]: UNABLE TO FIND BALL")
                self.lost_ball_counter = 0
                # SEND ACK_NEGATIVO OF MISSION ABORTED DUE TO BALL LOST.
                self.ACK_NEGATIVO = True
            self.lost_ball_counter += 1

    ## Callback of the subscriber to the laser scan, is used for implement a simple bug algorithm for obstacle avoidance during tracking.
    # @param msg LaserScan message --> distance from the obstacle
    def avoid_obstacle(self, msg):
        vel = Twist()
        self.lidar_regions = {
            'r': min(min(msg.ranges[0:143]),10),
            'fr': min(min(msg.ranges[144:287]),10),
            'f': min(min(msg.ranges[288:431]),10),
            'fl': min(min(msg.ranges[432:575]),10),
            'l': min(min(msg.ranges[576:713]),10),
        }
        thresh = 0.6
        thresh2 = 0.9
        if (self.lidar_regions['f'] > 0) and (self.lidar_regions['f'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.lidar_regions['fr'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.lidar_regions['fl'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: STOPPED OBS IN FRONT")
                self.ACK_NEGATIVO = True
        elif (self.lidar_regions['fr'] > 0) and (self.lidar_regions['fr'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.lidar_regions['f'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            elif self.lidar_regions['fl'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: STOPPED OBS IN FRONT")
                self.ACK_NEGATIVO = True
        elif (self.lidar_regions['fl'] > 0) and (self.lidar_regions['fl'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.lidar_regions['f'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.lidar_regions['fl'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: STOPPED OBS IN FRONT")
                self.ACK_NEGATIVO = True
    
    ## ACTION SERVER FUNCTION.
    # @param goal contain the color of the ball to track.
    def track(self, goal):
        rospy.loginfo("[trackingBall]: ACTIVATED")
        self.color = goal.color
        ## Subscriber to the odometry of the robot for update pose and position.
        sub_odom = rospy.Subscriber('odom', Odometry, self.odom_clbk)
        ## Publisher of the velocity commands for the robot.
        self.vel_publisher = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        ## Subscriber to camera topic, after receiving image start the tracking.
        camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.go_to_ball, queue_size=1)
        ## Subscriber to Laser Scan topic, for implement ostacle avoidance.
        laser_sub = rospy.Subscriber('/scan', LaserScan, self.avoid_obstacle)
        while not self.ACK_POSITIVO:
            if self.act_s.is_preempt_requested():
                rospy.loginfo('[trackingBall]: GOAL WAS PREEMPTED')
                self.act_s.set_preempted()
                break
            elif self.ACK_NEGATIVO == True:
                rospy.loginfo("[trackingBall]: MISSION STOPPED")
                vel = Twist()
                vel.linear.x = 0
                self.vel_publisher.publish(vel)
                break
            else:
                self.feedback.state = "REACHING THE BALL"
                self.act_s.publish_feedback(self.feedback)
        camera_sub.unregister()
        sub_odom.unregister()
        laser_sub.unregister()

        if not self.ACK_NEGATIVO == True:
            self.act_s.set_succeeded(self.result)
        else:
            self.act_s.set_preempted()
        self.ACK_NEGATIVO = False
        self.ACK_POSITIVO = False
        rospy.loginfo("[trackingBall] ACTION SERVER CLOSED")		


def main():
    try:
        # INIT ROS NODE
        rospy.init_node('Track_Node')
        # INIT ACTION SERVER
        track = Tracking('Tracking')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        print("SHUT DOWN TRACK SERVER")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


