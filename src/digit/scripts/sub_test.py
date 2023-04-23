#! /usr/bin/env python3          

import rospy 
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime 
import os

path = datetime.now().strftime("%d-%m-%Y--%H:%M:%S")
path = '/home/mtl/Desktop/salman_fyp/digit/Frames/'+path
os.mkdir(path)
i = 0

def callback(data):
    print('got something!!')
    bridge = CvBridge()
    # print the actual message in its raw format
    frame = bridge.imgmsg_to_cv2(data.data)

    fileName = path+'/img_'+ datetime.now().strftime("%H:%M:%S") + '.png'
    cv2.imwrite(fileName,frame)
    
    i = i+1
      
    # otherwise simply print a convenient message on the terminal
    print('Data from /topic_name received')
  
  
def main():
      
    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/DigitFrames", Image, callback)
      
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass