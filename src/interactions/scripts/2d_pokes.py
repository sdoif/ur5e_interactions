#!/usr/bin/env python

import rospy 

# ur5e libraries and motion planning
import moveit_commander
import moveit_msgs.msg 
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# digit and dataset libraries 
from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
import cv2
from datetime import datetime 
import os

######################

class UR5eDigitControl:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5e_moveit_interface", anonymous=True)

        rospy.loginfo("Starting UR5eDigitControl as ur5e_moveit_interface...")

        # Robot kinematic model and current joint states
        self.robot = moveit_commander.RobotCommander(robot_description= )

        # Later??? 
        # scene = moveit_commander.PlanningSceneInterface()

        # planning joints motion for ur5e
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        #
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        rospy.Subscriber('/wrench')



