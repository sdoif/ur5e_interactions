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


import math


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
    
          
tactile_sensor = "Digit"

mp = Experiment(tactile_sensor)

mp.publishPose()

