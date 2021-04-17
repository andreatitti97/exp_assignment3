#!/usr/bin/env python
import sys
import time 

import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

from sensor_msgs.msg import CompressedImage 

class ballDetector():
    def __init__(self):
	''' INITIALIZE ROS NODE '''
        rospy.init_node('ball_detection', anonymous=True)

        self.detectedRooms = ["Red", "Room2"]
        # Initialize the subscriber to the camera topic with the raw image
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.find_ball, queue_size=1)


    def color_detection(self, hsv_min, hsv_max, image_np):
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:
            return True


    def find_ball(self, ros_image):
        #### Detect colors (balls) in the image using OpenCV ####
        ## @param image_np is the decompressed image and converted in OpenCv
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

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

	x = self.color_detection(blackLower, blackUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED BLACK BALL")
        
	x = self.color_detection(redLower, redUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED RED BALL")
        
	x = self.color_detection(magentaLower, magentaUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED MAGENTA BALL")
	x = self.color_detection(greenLower, greenUpper, image_np)
	
        if x == True:
            rospy.loginfo("DETECTED GREEN BALL")
        
	x = self.color_detection(blueLower, blueUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED BLUE BALL")
        
	x = self.color_detection(yellowLower, yellowUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED YELLOW BALL")

def main(args):
    rd = ballDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

Toggle all file notes
Toggle all file annotations
