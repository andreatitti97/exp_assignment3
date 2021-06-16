#!/usr/bin/env python
## @file ballDetector.py
# This script is able to detect colored balls, thought the subscription to "camera1/image_raw/compressed" is able to  
# detect balls of 5 different colors ( black, yellow, red, blue, green) related to the knowledge representation.

# Python Libraries
import sys
import time 
import imutils
import numpy as np
from scipy.ndimage import filters
# OpenCV
import cv2
# Ros libraries
import roslib
import rospy
# Ros msgs
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import Bool, String

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

## Class which define the color detection algorithm used which run in a node on his own. Receive in input compressed images from the camera
# and then apply a mask of different colors to detect if at least one colored ball is in the scene. It's important to highlight that colors can
# be detected only once, so after the first detection of the black ball the algorithm is not able to detect other black balls.
class ballDetector():
    def __init__(self):
        ''' INITIALIZE ROS NODE and PUBLISHER '''
        rospy.init_node('Ball_Detection_Node', anonymous=True)
        ## List for store already detected colors 
        self.detectedBalls = 'None'
        ## The publisher send strings for transfer the new color founded (associated to a room).
        self.ball_detection_pub = rospy.Publisher('color_found', String, queue_size=10)
        self.rate = rospy.Rate(1)
        self.ballDetected = False


    ## Simple color detection, this function return true when a contour of the given input color is founded in the input image. 
    # @param hsv_min minimum threshold of the input color expressed in hsv scale.
    # @param hsv_max maximum threshold of the input color expressed in hsv scale.
    # @param image_np image stored in a numpy array. 
    def color_detection(self, hsv_min, hsv_max, image_np):
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0.5:
            return True

    ## This function implement the ball detection algorithm, given an input image different mask are applied for detect the color of our knowledge representation.
    # It's the callback of the subscriber to the camera topic, so each time a new image is available is called.
    # @param image_np is the decompressed image and converted in OpenCv
    def ball_detection(self, ros_image):
        ## @param image_np is the decompressed image and converted in OpenCv
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        self.ballDetected = False
        # Color tests
        self.ballDetected = self.color_detection(blackLower, blackUpper, image_np)
        if self.ballDetected == True:
            if "black" != self.detectedBalls:
                    self.detectedBalls = "black"
                    self.ball_detection_pub.publish("black")

        self.ballDetected = self.color_detection(redLower, redUpper, image_np)
        if self.ballDetected == True:
            if "red" != self.detectedBalls:
                    self.detectedBalls = "red"
                    self.ball_detection_pub.publish("red")

        '''self.ballDetected = self.color_detection(magentaLower, magentaUpper, image_np)
        if self.ballDetected == True:
            if "magenta" != self.detectedBalls:
                    self.detectedBalls = "magenta"
                    self.ball_detection_pub.publish("magenta")'''

        self.ballDetected = self.color_detection(greenLower, greenUpper, image_np)
        if self.ballDetected == True:
            if "green" != self.detectedBalls:
                    self.detectedBalls = "green"
                    self.ball_detection_pub.publish("green")

        self.ballDetected = self.color_detection(blueLower, blueUpper, image_np)
        if self.ballDetected == True:
            if "blue" != self.detectedBalls:
                    self.detectedBalls = "blue"
                    self.ball_detection_pub.publish("blue")

        self.ballDetected = self.color_detection(yellowLower, yellowUpper, image_np)
        if self.ballDetected == True:
            if "yellow" != self.detectedBalls:
                    self.detectedBalls = "yellow"
                    self.ball_detection_pub.publish("yellow")
    ## Function that start the detection subscribing to the camera topic.
    def startDetection(self):
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.ball_detection, queue_size=1)
    ## Function that stop the detection unregistering the camera topic.
    def stopDetection(self):
        self.camera_sub.unregister()
## This is the callback function of the subscriber to \detection_state topic, it receive a boolen which notify to start the detection or stop it.
def detectionState(state, rd):
    if state.data:
        rd.startDetection()
    else:
        rd.stopDetection()

def main(args):
    
    try:
        ball_detect = ballDetector()
        ## Subscriber to the \detection_state topic which notify with a boolean to start or stop the detection.
        detection_state_sub = rospy.Subscriber("detection_state", Bool, detectionState, ball_detect)
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("ROS image module terminated")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
