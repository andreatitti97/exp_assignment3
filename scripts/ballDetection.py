#!/usr/bin/env python
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
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import Bool, String

class ballDetector():
    def __init__(self):
	''' INITIALIZE ROS NODE '''
        rospy.init_node('ball_detection', anonymous=True)

        self.detectedBalls = []
        # Initialize the subscriber to the camera topic with the raw image
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.ball_detection, queue_size=1)
        self.room_pub = rospy.Publisher('new_room_found', String, queue_size=10)
        self.rate = rospy.Rate(1)

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


    def ball_detection(self, ros_image):
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
        # Color tests
        x = self.color_detection(blackLower, blackUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED BLACK BALL")
            if "black" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("black")
		self.room_pub.publish("black")

	    x = self.color_detection(redLower, redUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED RED BALL")
            if "red" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("red")
		self.room_pub.publish("red")

	    x = self.color_detection(magentaLower, magentaUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED MAGENTA BALL")
            if "magenta" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("magenta")
		self.room_pub.publish("magenta")

	    x = self.color_detection(greenLower, greenUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED GREEN BALL")
            if "green" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("green")
		self.room_pub.publish("green")
	    x = self.color_detection(blueLower, blueUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED BLUE BALL")
            if "blue" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("blue")
		self.room_pub.publish("blue")
	    x = self.color_detection(yellowLower, yellowUpper, image_np)
        if x == True:
            rospy.loginfo("DETECTED YELLOW BALL")
            if "yellow" not in self.detectedBalls:
                print("New Room detected")
                self.detectedBalls.append("yellow")
		self.room_pub.publish("yellow")
	self.rate.sleep()
def main(args):
    
    try:
	time.sleep(15)#wait for cmdManager.py
	ball_detect = ballDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print ("ROS image module terminated")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
