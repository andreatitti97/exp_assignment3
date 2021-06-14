#!/usr/bin/env python

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


# Colors
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

class TrackAction(object):

    ## Publisher to move the robot 

    def __init__(self, name):
        self.action_name = name
        self.act_s = actionlib.SimpleActionServer('TrackAction', exp_assignment3.msg.ballTrackingAction, self.track, auto_start=False)
        self.act_s.start()
        self.feedback = exp_assignment3.msg.ballTrackingFeedback()
        self.result = exp_assignment3.msg.ballTrackingResult()
        self.regions = {'right':0,'fright':0, 'front':0,'fleft':0,'left':0,}
        #self.vel_publisher = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        self.success = False
        self.unfound_ball_counter = 0
        self.abort = False
        self.radius = 0
        self.vel_publisher = 0
        self.position = Point()
        self.pose = Pose()
    
    def odom_clbk(self, msg):
        self.pose = msg.pose.pose
        self.position = msg.pose.pose.position

    def go_to_ball(self, ros_image):
        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
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
        elif self.color == "magenta":
            mask = cv2.inRange(hsv, magentaLower, magentaUpper)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        center = None
        # only proceed if at least one contour was found

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if self.radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                
                # Setting the velocities to be applied to the robot
                vel = Twist()
                # 400 is the center of the image 
                vel.angular.z = -0.002*(center[0]-400)
                # 150 is the radius that we want see in the image, which represent the desired disatance from the object 
                vel.linear.x = -0.007*(self.radius-150)
                self.vel_publisher.publish(vel)
                rospy.loginfo("[trackingBall]: TRACKING ")
                if (self.radius>=143) and abs(center[0]-400)<5: #Condition for considering the ball as reached
                    rospy.loginfo("ballDetection --> BALL REACHED")
                    self.result.x = self.position.x
                    self.result.y = self.position.y
                    self.success = True

        else:
            rospy.loginfo("[trackingBall]: BALL NOT FOUND")
            vel = Twist()
            if self.unfound_ball_counter <= 8:
                rospy.loginfo("[trackingBall]: TURN RIGHT SEARCHING THE BALL")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            elif self.unfound_ball_counter < 17:
                rospy.loginfo("[trackingBall]: TURN LEFT SEARCHING THE BALL")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.unfound_ball_counter == 17:
                rospy.loginfo("[trackingBall]: UNABLE TO FIND BALL")
                self.unfound_ball_counter = 0
                #self.act_s.set_preempted()
                self.abort = True
            self.unfound_ball_counter += 1

    def avoid_obstacle(self, msg):
        vel = Twist()
        self.regions = {
            'right': min(min(msg.ranges[0:143]),10),
            'fright': min(min(msg.ranges[144:287]),10),
            'front': min(min(msg.ranges[288:431]),10),
            'fleft': min(min(msg.ranges[432:575]),10),
            'left': min(min(msg.ranges[576:713]),10),
        }
        thresh = 0.6
        thresh2 = 0.9
        if (self.regions['front'] > 0) and (self.regions['front'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.regions['fright'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.regions['fleft'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: ABORTED OBS IN FRONT")
                self.abort = True
        elif (self.regions['fright'] > 0) and (self.regions['fright'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.regions['front'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            elif self.regions['fleft'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = 0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: ABORTED OBS IN FRONT")
                self.abort = True
        elif (self.regions['fleft'] > 0) and (self.regions['fleft'] <= thresh) and (self.radius < 110):
            rospy.loginfo("[trackingBall]: DETECTED OBS")
            if self.regions['front'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN RIGHT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            elif self.regions['fleft'] > thresh2:
                rospy.loginfo("[trackingBall]: TURN LEFT")
                vel.angular.z = -0.5
                self.vel_publisher.publish(vel)
            else:
                rospy.loginfo("[trackingBall]: ABORTED OBS IN FRONT")
                self.abort = True
    
    
    def track(self, goal):
	rospy.loginfo("[trackingBall]: Activated")
        self.color = goal.color
        # crate the subscriber to camera1 in order to recive and handle the images
        
        sub_odom = rospy.Subscriber('odom', Odometry, self.odom_clbk)
        self.vel_publisher = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.go_to_ball, queue_size=1)
        laser_sub = rospy.Subscriber('/scan', LaserScan, self.avoid_obstacle)
        while not self.success:
            if self.act_s.is_preempt_requested():
                rospy.loginfo('[trackingBall]: Goal was preempted')
                self.act_s.set_preempted()
                break
            elif self.abort == True:
                rospy.loginfo("[trackingBall]: MISSION ABORTED")
                vel = Twist()
                vel.linear.x = 0
                self.vel_publisher.publish(vel)
                break
            else:
                self.feedback.state = "Reaching the ball..."
                self.act_s.publish_feedback(self.feedback)
        camera_sub.unregister() #unregister from camera topic
        sub_odom.unregister()
        laser_sub.unregister()
        self.vel_publisher.unregister()

        if not self.abort == True:
            self.act_s.set_succeeded(self.result)
        else:
            self.act_s.set_preempted()
        self.abort == False
        self.success = False
        rospy.loginfo("[trackingBall] ACTION SERVER CLOSED")		


def main():
    try:
        rospy.init_node('Track')
        track = TrackAction('TrackAction')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        print("SHUT DOWN TRACK SERVER")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


