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
        self.detectedBalls = 'None'
        # Initialize the subscriber to the camera topic with the raw image
        #self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.ball_detection, queue_size=1)
        self.room_pub = rospy.Publisher('new_room_found', String, queue_size=10)
        self.rate = rospy.Rate(1)
        self.ballDetected = False

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
        self.ballDetected = False
        # Color tests
        self.ballDetected = self.color_detection(blackLower, blackUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED BLACK BALL")
            if "black" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "black"
                    self.room_pub.publish("black")

        self.ballDetected = self.color_detection(redLower, redUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED RED BALL")
            if "red" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "red"
                    self.room_pub.publish("red")

        '''self.ballDetected = self.color_detection(magentaLower, magentaUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED MAGENTA BALL")
            if "magenta" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "magenta"
                    self.room_pub.publish("magenta")'''

        self.ballDetected = self.color_detection(greenLower, greenUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED GREEN BALL")
            if "green" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "green"
                    self.room_pub.publish("green")

        self.ballDetected = self.color_detection(blueLower, blueUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED BLUE BALL")
            if "blue" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "blue"
                    self.room_pub.publish("blue")

        self.ballDetected = self.color_detection(yellowLower, yellowUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[ballDetection]: DETECTED YELLOW BALL")
            if "yellow" != self.detectedBalls:
                    print("[ballDetection]: NEW ROOM DETECTED")
                    self.detectedBalls = "yellow"
                    self.room_pub.publish("yellow")
        #cv2.imshow('window',image_np)
        #cv2.waitkey(2)
    def startDetection(self):
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.ball_detection, queue_size=1)
        rospy.loginfo("[ballDetection]: CAMERA STARTED")

    def stopDetection(self):
        self.camera_sub.unregister()

def detectionState(state, rd):
    if state.data:
        rd.startDetection()
        rospy.loginfo("[ballDetection]: START DETECTING ")
    else:
        rd.stopDetection()
        rospy.loginfo("[ballDetection]: STOP DETECTING ")

def main(args):
    
    try:
        ball_detect = ballDetector()
        cmdSub = rospy.Subscriber("room_detection", Bool, detectionState, ball_detect)
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("ROS image module terminated")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
