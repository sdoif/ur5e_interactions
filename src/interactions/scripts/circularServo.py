#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerRequest
from rospy.rostime import Duration
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from datetime import datetime 
import os
import pickle

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


class Experiment(object):

    def __init__(self, ts):
        super(Experiment, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## call service ur_hardware_interface/zero_ftsensor
        ## service has type std_srvs/Trigger
        ## exception for error handling, print out error message

        try:
            rospy.wait_for_service("/ur_hardware_interface/zero_ftsensor", timeout=3)
            zero_ftsensor = rospy.ServiceProxy("/ur_hardware_interface/zero_ftsensor", Trigger)
            zero_ftsensor(TriggerRequest())
            print("FT sensor zeroed")
        except:
            print("Service call failed")

        ### Subscriber for wrench data
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_cb)
        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
        move_group = self.move_group
        self.home =  move_group.get_current_pose().pose
        self.actual_home = move_group.get_current_pose().pose
        self.aactual_home = self.actual_home
        self.aactual_home.position.x = 0
        self.aactual_home.position.y = 0.38
        self.aactual_home.position.y = 0.13

        self.data = {'force':[],
                     'position':[],
                     'tactile': [],
                     'time':[]}

        ## Subscribe to Digit topic to obtain tactile data
        #rospy.Subscriber("/DigitFrames", Image, self.digit_cb)
        self.bridge = CvBridge()
        self.digit = None

        self.test_interrupt = 0
        ## Subscribe to Interrupt topic to obtain interrupt signal AND UPDATE self.test_interrupt
        rospy.Subscriber("/Interrupt", Int16, self.interrupt_cb)


        self.tactile_sensor = ts

    def interrupt_cb(self, msg):
        self.test_interrupt = msg.data

    def probe_object(self, n):

        move_group = self.move_group
        center = [0, 0.5, 0]
        radius = center[1] - move_group.get_current_pose().pose.position.y
        points = self.circle_points(center, radius, n)
        
        for point in points:
            self.move_to_position(point)
            self.orient_wrist(center)
            #self.move_toward_center(center)
            time.sleep(1)
            
    def move_to_position(self, position):

        move_group = self.move_group
        pose_goal = move_group.get_current_pose().pose

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        #pose_goal.position.z = position[2]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def orient_wrist(self, center):
        move_group = self.move_group
        pose_goal = move_group.get_current_pose().pose

        dx = center[0] - pose_goal.position.x
        dy = center[1] - pose_goal.position.y
        dz = center[2] - pose_goal.position.z

        
        roll, pitch, yaw = euler_from_quaternion([pose_goal.orientation.x, 
                                                pose_goal.orientation.y, 
                                                pose_goal.orientation.z, 
                                                pose_goal.orientation.w])  # Keep current pitch and roll

        q = quaternion_from_euler(roll, pitch, yaw)

        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def move_toward_center(self, center):
        while True:
            pose_goal = self.move_group.get_current_pose().pose
            direction = [center[0] - pose_goal.position.x,
                         center[1] - pose_goal.position.y,
                         center[2] - pose_goal.position.z]
            norm = math.sqrt(sum([i**2 for i in direction]))
            direction = [i/norm for i in direction]
            pose_goal.position.x += direction[0] * 0.01  # move 1 cm
            pose_goal.position.y += direction[1] * 0.01
            pose_goal.position.z += direction[2] * 0.01
            self.move_group.set_pose_target(pose_goal)
            plan = self.move_group.go(wait=False)
            # self.move_group.stop()
            # self.move_group.clear_pose_targets()
            if self.test_interrupt > 10:  # if force is more than 10N
                # TODO: add pause
                break

    def wrench_cb(self, data):
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
        self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

    @staticmethod
    def circle_points(center, radius, n):
        points = []
        for i in range(n):
            theta = tau * i / n
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)
            z = center[2]
            points.append([x, y, z])
        return points

tactile_sensor = "Digit"

mp = Experiment(tactile_sensor)
mp.move_toward_center([0, 0.5, 0])