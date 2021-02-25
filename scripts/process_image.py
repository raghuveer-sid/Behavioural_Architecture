#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_msgs.msg import Float64
import math

VERBOSE = False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        self.camera_pub = rospy.Publisher("joint1_position_controller/command", 
                                          Float64, queue_size=1)
        self.flag_arrive = False
        self.counter = 0
        self.ball_lost_pub = rospy.Publisher("/lost_ball",Bool, queue_size=1)
        self.subBall = rospy.Subscriber('/found_ball',Bool,self.callbackFoundBall, queue_size=1)
        self.ball_found = False
    def callbackFoundBall(self,data):
        self.ball_found = data.data
        if self.ball_found == True:
            print("Robot_following node starting" , self.ball_found)
            self.ball_lost_pub.publish(False)
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        
        # only proceed if at least one contour was found
        if len(cnts) > 0 and self.ball_found==True:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            # self.flag_arrive = False
            self.ball_lost_pub.publish(False)
            self.counter = 0
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = 0.005*(center[0]-400)
                vel.linear.x = -0.01*(radius-200)
                self.camera_pub.publish(0)
                self.vel_pub.publish(vel)
                self.counter = 0
                #Rotate head +45 and -45 degrees 
                ##
                if self.flag_arrive == False and vel.linear.x < 0.05 and vel.angular.z < 0.05:
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    cam_angle = Float64()
                    cam_angle.data = -math.pi/4
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    # cv2.waitKey(3)
                    time.sleep(1)
                    cam_angle.data = math.pi/4
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    
                    time.sleep(1)
                    cam_angle.data = 0.0
                    self.camera_pub.publish(cam_angle)
                    cv2.imshow('window',image_np)
                    
                    time.sleep(1)
                    self.flag_arrive = True
                    self.counter = 0
          
            else:
                vel = Twist()
                self.camera_pub.publish(0)
                vel.linear.x = 0.5
                self.vel_pub.publish(vel)
                self.flag_arrive = False
                self.counter = 0
        elif self.ball_found==True:
            self.counter = self.counter + 1
            vel = Twist()
            vel.angular.z = -math.pi
            self.vel_pub.publish(vel)
            self.flag_arrive = False
            if self.counter == 150:                
                self.ball_lost_pub.publish(True)
                print('Cannot find ball')
                self.ball_found = False
                #self.subscriber.unregister()
                # self.counter = 0
                return None
        cv2.imshow('window', image_np)
        cv2.waitKey(2)
        # self.subscriber.unregister()

def main(args):
    
    '''Initializes and cleanup ros node'''
  
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)