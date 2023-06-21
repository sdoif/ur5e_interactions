#! /usr/bin/env python3          

import rospy 
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from datetime import datetime 
import os

class DigitNode(object):
    def __init__(self, dsn):

        self.digit = Digit(dsn)
        self.digit_meta = DigitHandler.find_digit(dsn)
        self.fps = self.digit.fps
        self.res = self.digit.resolution
        

        self.frame = None
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.digit_meta["dev_name"])

        rospy.init_node(dsn, anonymous=True)
        print(self.fps)
        self.loop_rate = rospy.Rate(10) 
        # self.loop_rate = rospy.Rate(self.fps) 
        self.pub = rospy.Publisher("DigitFrames", Image, queue_size=10)
    
    def start(self):

        rospy.loginfo("Start publishing of "+self.digit_meta["dev_name"]+" Frames")

        while not rospy.is_shutdown():
            _, self.frame = self.cap.read()
            rospy.loginfo("info message")
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.frame))
            self.loop_rate.sleep()
        
    def start1(self):

        self.digit.connect()

        rospy.loginfo("Start publishing of "+self.digit_meta["dev_name"]+" Frames")

        while not rospy.is_shutdown():
            self.frame = self.digit.get_frame()
            rospy.rospy.loginfo("info message")
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.frame))
            self.loop_rate.sleep()
        
        self.digit.disconnect()

if __name__ == '__main__':

    digit_node = DigitNode('D20508')
    digit_node.start()

            

