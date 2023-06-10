#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerRequest
from rospy.rostime import Duration
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel

from datetime import datetime 
import os
import pickle
import matplotlib.pyplot as plt
import math

from geometry_msgs.msg import WrenchStamped

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

import time
import numpy as np
import math

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def zeroFT():
    try:
        rospy.wait_for_service("/ur_hardware_interface/zero_ftsensor", timeout=2)
        zero_ftsensor = rospy.ServiceProxy("/ur_hardware_interface/zero_ftsensor", Trigger)
        zero_ftsensor(TriggerRequest())
        print("FT sensor zeroed")
    except:
        print("Service call failed")


class Experiment(object):

    def __init__(self, ts):
        super(Experiment, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("data_logger", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)


        ### Subscriber for wrench data
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_cb)
        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]
        self.frame = 0
        
        move_group = self.move_group
        self.home =  move_group.get_current_pose().pose
        self.actual_home = move_group.get_current_pose().pose
        self.aactual_home = self.actual_home

        self.data = {'force':[],
                     'position':[],
                     'tactile': [],
                     'time':[]}

        ## Subscribe to Digit topic to obtain tactile data
        rospy.Subscriber("/DigitFrames", Image, self.digit_cb)
        self.bridge = CvBridge()
        self.digit = None

        self.approaching = 0
        ## Subscribe to Interrupt topic to obtain interrupt signal AND UPDATE self.test_interrupt
        rospy.Subscriber("/Approaching", Int16, self.approaching_cb)


        self.tactile_sensor = ts

        self.count = 0

    def approaching_cb(self, msg):
        print(msg.data)
        self.approaching = msg.data

    def wrench_cb(self, data):
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z] 
        self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    
    def logSelfData(self):
        current_position =  self.move_group.get_current_pose().pose
        self.data['force'].append(self.force)
        self.data['position'].append(current_position)
        self.data['tactile'].append(self.frame)
        self.data['time'].append(datetime.now().strftime("%d-%m-%Y-%H-%M-%S-%f"))

    def digit_cb(self, data):
        
        ## convert data.data using cvbridge
        self.frame = self.bridge.imgmsg_to_cv2(data)        
        # obtain cartesian coordinates at current position
        # store current force, coordinates, frames, and time in self.data dictionary
        # format time as day-month-year-hour-minute-second-millisecond
        if self.approaching:
            self.logSelfData()

    def LogAndLoop(self):
        # loop and print current position until enter is pressed
        while True:
            if self.approaching>0:
                self.logSelfData()
            elif self.approaching<0:
                break

    def saveData(self):
        # currently we are saving the data in a pickle file
        # the data it saves is timestamped force, pose and tactile sensor data

        # create unique filename using time, data, and type of tactile sensor
        filename = datetime.now().strftime("%d-%m-%Y-%H-%M-%S-%f") + ".pkl"

        # Get the absolute path of the directory containing the script
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Construct the absolute path of the data file
        data_dir = os.path.join(script_dir, '..', 'data', self.tactile_sensor)
        location = os.path.join(data_dir, filename)

        # Create the directory if it doesn't exist
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        
        with open(location, 'wb') as handle:
            pickle.dump(self.data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        
    def publishPose(self):
        # Create a publisher for the robot pose
        pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

        rate = rospy.Rate(60)  # Publish rate of 1 Hz (adjust as needed)

        move_group = self.move_group

        while not rospy.is_shutdown():
            # Get the current pose of the end-effector
            current_pose = move_group.get_current_pose().pose

            # Create a PoseStamped message to publish the pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = move_group.get_planning_frame()
            pose_msg.pose = current_pose

            # Publish the pose message
            pose_pub.publish(pose_msg)

            rate.sleep()
    


import csv
          
tactile_sensor = "Digit"

mp = Experiment(tactile_sensor)

mp.LogAndLoop()

# mp.publishPose()

# Data Saving ###
try:
    mp.saveData()
    print("Data saved")
except:
    print("Data not saved")
