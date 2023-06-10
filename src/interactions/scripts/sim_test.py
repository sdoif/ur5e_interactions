#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, BoundingVolume
from moveit_msgs.msg import PositionConstraint
from shape_msgs.msg import SolidPrimitive
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerRequest
from rospy.rostime import Duration
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel
from visualization_msgs.msg import Marker

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
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

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

        zeroFT()
        self.approaching = False
        
        ### Subscriber for wrench data
        rospy.Subscriber("/wrench", WrenchStamped, self.wrench_cb)
        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]
        self.frame = 0

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

        self.pub_approaching = rospy.Publisher('Approaching', Int16, queue_size=10)

        self.tactile_sensor = ts

        self.count = 0

    def setJoint(self, angle, joint=4):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()

        joint_goal[joint] = angle
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)

    def wrench_cb(self, data):
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z] 
        self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

    def moveForwardUntilForce(self, increment = 0.005, max_force = 3, y_max = 0.42):
        move_group = self.move_group
        wpose_start = move_group.get_current_pose().pose

        # control logic: move forward until force exceeds
        wpose = wpose_start

        total_increment = 0

        out_of_bounds = False

        while True:

            current_position = move_group.get_current_pose().pose.position

            print(current_position)
            print()

            if (abs(self.force[2]) < max_force and current_position.y < y_max):
                waypoints = []        
                total_increment = total_increment + increment
                wpose.position.y = wpose.position.y + increment  # and sideways (y)
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = move_group.compute_cartesian_path(
                    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold 
                move_group.execute(plan, wait=True)

                time.sleep(2)

            else:

                if current_position.y > y_max:
                    out_of_bounds = True

                time.sleep(2)

                waypoints = []        
                wpose.position.y = wpose.position.y - total_increment  # and sideways (y)
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = move_group.compute_cartesian_path(
                    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold 
                move_group.execute(plan, wait=True)

                time.sleep(2)

                break

        return out_of_bounds

    def moveXZ(self, x_increment=0, z_increment=0):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose

        waypoints = []        
        wpose.position.x = wpose.position.x - x_increment  # and sideways (y)
        wpose.position.z = wpose.position.z + z_increment  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold 
        move_group.execute(plan, wait=True)

    def servoInXY(self, contact_force=3, x_increment=0.01):
        # image we have at coordinates (0,0.5,0) a pole which is infinitely tall and thin 
        # we want to move the robot to the pole until contact is made
        # contact is made when the force in the z direction is greater than 3 N (contact_force)
        # we want to servo in the XY plane until we are no longer in contact with the pole (out of bounds)
        # we will first move in the positive x direction until we are out of bounds
        # then we will move in the negative x direction until we are out of bounds
        no_positive_x = True
        while True:
            out_of_bounds = self.moveForwardUntilForce(y_max=0.4)
            if not out_of_bounds:
                no_positive_x = False
                self.moveXZ(x_increment=x_increment)
            else:
                break
        
        self.moveHome()

        no_negative_x = True
        while True:
            out_of_bounds = self.moveForwardUntilForce(y_max=0.4)
            if not out_of_bounds:
                no_negative_x = False
                self.moveXZ(x_increment=-x_increment)
            else:
                break
        
        self.moveHome()
        
        # return true if we were able to servo in both directions
        # return false if we were not able to servo in both directions
        # signal for servoInXYZ to stop
        return no_positive_x and no_negative_x
    
    def servoInXYZ(self, contact_force=3, x_increment=0.015, z_increment=0.02):
        # current logic for this is to servo in the XY plane until we are out of bounds
        # move in the positive z direction
        # and repeat until we get the signal to stop from servoInXY
        
        while True:
            done = self.servoInXY(contact_force=contact_force, x_increment=x_increment)
            if not done:
                self.moveXZ(z_increment=z_increment)
                self.updateHome()
            else:
                break
        
        self.moveActualHome()

    def getPositionAndLoop(self):
        # loop and print current position until enter is pressed
        move_group = self.move_group

        while True:
            wpose = move_group.get_current_pose().pose
            print(wpose)
            key = input("Press Enter to continue or type 'x' to quit: ")
            if key == "":
                continue
            elif key == "x":
                break
    
    def generateCircleIncrement(self, start, cx = 0, cy = 0.5, n=8, r=0.11):
        # radius is magnitude between [cx,cy] and [start.position.x, start.position.y]
        #r = math.sqrt((start.position.x - cx)**2 + (start.position.y - cy)**2)

        # create list of waypoints
        waypoints_list = []

        for i in range(n):

            angle = 2 * math.pi * i / n
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)

            waypoints_list.append([x,y])
        
        def sort_points(waypoints_list, center):
            # Compute angle for a point
            def get_angle(point):
                dx, dy = point[0] - center[0], point[1] - center[1]
                return math.atan2(dy, dx)

            # Sort points counterclockwise from the reference point
            waypoints_list.sort(key=get_angle)

            return waypoints_list

        waypoints_list = [wp for wp in waypoints_list if wp[0] >= cx-(r/math.sqrt(2))]
        waypoints_list = [wp for wp in waypoints_list if wp[1] <= cy+(r/math.sqrt(2))]
        
        waypoints_list = sort_points(waypoints_list, [cx,cy])

        return waypoints_list, r
    
    def calculateArcChange(self, start, end, cx = 0, cy = 0.5):

        delta_x_prev = start[0] - cx
        delta_y_prev = start[1] - cy
        delta_x = end[0] - cx
        delta_y = end[1] - cy
        
        # Calculate the angles
        theta_prev = math.atan2(delta_y_prev, delta_x_prev)
        theta = math.atan2(delta_y, delta_x)
        
        # Calculate the angle of the arc
        angle = theta_prev-theta

        if start[0] > end[0]:
            angle = angle

        return angle
    
    def circleApproach(self, n = 8, center = [0,0.5], radius = 0.1, offset = 0.9):

        move_group = self.move_group
        start = self.move_group.get_current_pose().pose   
        cx = center[0]
        cy = center[1]

        waypoints_list,r = self.generateCircleIncrement(start, cx=cx, cy=cy, n=n, r=radius)
        # print(waypoints_list) each coordinate on a new line 
        print(*waypoints_list, sep = "\n")

        self.moveTo([cx,cy-r])

        # plot waypoints
        x = [wp[0] for wp in waypoints_list]
        y = [wp[1] for wp in waypoints_list]

        # give option to abort function before moving
        key = input("Press Enter to continue or type 'x' to quit: ")
        if key == "x":
            return

        start = self.move_group.get_current_pose().pose 

        path = [start]

        for (i,(x,y)) in enumerate(waypoints_list):

            # calculate the angle of the arc formed by the current waypoint and the previous waypoint
            # the arc is formed on a circle with radius r, and center (cx,cy)

            if i == 0:
                angle = self.calculateArcChange([start.position.x, start.position.y], [x,y], cx=cx, cy=cy)
            else:
                angle = self.calculateArcChange(waypoints_list[i-1], [x,y], cx=cx, cy=cy)
            
            wrist_angle = self.move_group.get_current_joint_values()

            print(f'Moving to waypoint {i}: ({x},{y})')

            time.sleep(1)
            
            #waypoints = []
            wpose = self.move_group.get_current_pose().pose   

            roll, pitch, yaw = euler_from_quaternion([wpose.orientation.x, 
                                            wpose.orientation.y, 
                                            wpose.orientation.z, 
                                            wpose.orientation.w])  # Keep current pitch and roll
            yaw -= angle
            q = quaternion_from_euler(roll, pitch, yaw)
            
            wpose.position.x = x
            wpose.position.y = y
            wpose.orientation.x = q[0]
            wpose.orientation.y = q[1]
            wpose.orientation.z = q[2]
            wpose.orientation.w = q[3]
            
            path.append(copy.deepcopy(wpose))
            
            move_group.go(wpose, wait=True)   

            # self.approaching = True
            # self.pub_approaching.publish(1)
            # centerPath = self.moveToObjectCenter([cx,cy], offset=offset)            
            # move_group.go(wpose, wait=True)
            # self.approaching = False
            # self.pub_approaching.publish(0)

            
            # path.append(copy.deepcopy(centerPath))

            constraint_yaw = yaw-math.pi/2
            # Print constraint yaw in degrees
            print('Constraint yaw: ', math.degrees(constraint_yaw))
            print()

            self.moveToObjectCenterWithZ(wpose, cx, cy, offset, constraint_yaw, max_z=0.155)
            
            zeroFT()

        return path
    
    def moveToObjectCenterWithZ(self, wwpose, cx, cy, offset, constraint_yaw, max_z = 0.155):
            move_group = self.move_group
            wpose = self.move_group.get_current_pose().pose

            while wpose.position.z < max_z:
                # Enforce updated constraints
                #self.enforceConstraints(wpose, constraint_yaw)
                
                # Move to object center
                print('Approaching object @ z = ', wpose.position.z)
                self.approaching = True
                self.pub_approaching.publish(1)
                centerPath = self.moveToObjectCenter([cx,cy], offset=offset) 

                # Return to cicumference
                move_group.go(wpose, wait=True)
                self.approaching = False
                self.pub_approaching.publish(0)
                
                # Clear constraints to increment z
                #self.clearConstraints()
                
                # Increment z
                wpose.position.z += 0.021
                move_group.go(wpose, wait=True)
                move_group.stop()
                move_group.clear_pose_targets()            

            # Move back to initial pose with initial height
            move_group.go(wwpose, wait=True)

    def moveToObjectCenter(self, center, offset = 0.9):
        
        # move to the center of the object
        move_group = self.move_group
        #move_group.set_max_velocity_scaling_factor(0.004)
        #move_group.set_max_acceleration_scaling_factor(0.0003)

        start = move_group.get_current_pose().pose
        wpose = move_group.get_current_pose().pose

        wpose.position.x = wpose.position.x + offset * (center[0] - wpose.position.x)
        wpose.position.y = wpose.position.y + offset * (center[1] - wpose.position.y)

        move_group.go(wpose, wait=False)

        # waypoints = []
        # waypoints.append(copy.deepcopy(wpose))
        # plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        # move_group.execute(plan, wait=False)

        threshold = 0.01
        path = []
        timeout = 0
        
        print('Starting moveToObjectCenter')
        while True:
            current_pose = move_group.get_current_pose().pose
            path.append(current_pose)

            if abs(self.force[2]) > 3:
                print('Force exceeded threshold. Aborting motion.')
                move_group.stop()
                time.sleep(1)  # Pause for 2 seconds
                move_group.clear_pose_targets()
                break
            elif abs(current_pose.position.x - wpose.position.x) < threshold and abs(current_pose.position.y - wpose.position.y) < threshold:
                timeout += 1
                if timeout > 1000:
                    print('Timeout. Aborting motion.')
                    move_group.stop()
                    time.sleep(1)  # Pause for 2 seconds
                    move_group.clear_pose_targets()
                    break

        #move_group.set_max_velocity_scaling_factor(1)
        #move_group.set_max_acceleration_scaling_factor(0.5)

        return path
                
    def updateHome(self):
        self.home = self.move_group.get_current_pose().pose

    def moveHome(self, waypoints = []):
        move_group = self.move_group
        wpose = self.home

        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold 
        move_group.execute(plan, wait=True)
    
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
        
    def defineStart(self):
        move_group = self.move_group
        
        self.start = move_group.get_current_pose().pose

    def moveBack2Start(self, center):
        self.pub_approaching.publish(-1)
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose

        radius = ((center[0]-current_pose.position.x)**2+(center[1]-current_pose.position.y)**2)**0.5
        
        if current_pose.position.x > center[0] and current_pose.position.y > center[1]:
            current_pose.position.x += radius/2
            move_group.set_pose_target(current_pose)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()   

        elif current_pose.position.x < center[0] and current_pose.position.y > center[1]:
            current_pose.position.x -= radius/2
            move_group.set_pose_target(current_pose)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
        
        current_pose.orientation = self.start.orientation
        current_pose.position.y = self.start.position.y
        

        move_group.set_pose_target(current_pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()   

        # current_pose.position.x -= radius  
        move_group.set_pose_target(self.start)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets() 

        #print(self.start)

        #self.moveLinearly(self.start)  

    def move2Start(self, center):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose

        radius = ((center[0]-current_pose.position.x)**2+(center[1]-current_pose.position.y)**2)**0.5
        

        if current_pose.position.x > center[0] and current_pose.position.y > center[1]:
            current_pose.position.x = center[0] + radius
            move_group.set_pose_target(current_pose)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()   

        elif current_pose.position.x < center[0] and current_pose.position.y > center[1]:
            current_pose.position.x = center[0] - radius
            move_group.set_pose_target(current_pose)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
        
        current_pose.orientation = self.start.orientation
        current_pose.position.y = self.start.position.y
        

        move_group.set_pose_target(current_pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()   

        # current_pose.position.x -= radius  
        # move_group.set_pose_target(current_pose)
        # move_group.go(wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets() 

        #print(self.start)

        #self.moveLinearly(self.start)  

    def moveLinearly(self, goal):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose

        # Create a Cartesian path constraint
        waypoints = []
        waypoints.append(goal)

        # Compute the Cartesian path
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0)        # jump_threshold

        # Execute the computed path
        self.move_group.execute(plan, wait=True)

    def addEnvironment(self):
        # the purpose of this function is to add the environment to the scene
        # this will allow us to see the environment in rviz
        # and ensure there are no collisions with the environment
        scene = self.scene
        
        # first we add the wall, which is 40cm behind the robot
        # it is assumed that the wall is 4m tall and 10m wide
        # the wall is 0.4m thick
        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "base"
        wall_pose.pose.position.x = 0
        wall_pose.pose.position.y = 0.3
        wall_pose.pose.position.z = 0
        wall_pose.pose.orientation.z = 0.7071
        wall_pose.pose.orientation.w = 0.7071
        scene.add_box("wall", wall_pose, (0.001, 5, 2))

        #add imaginary walls
        left_wall_pose = geometry_msgs.msg.PoseStamped()
        left_wall_pose.header.frame_id = "base"
        left_wall_pose.pose.position.x = -0.38
        left_wall_pose.pose.position.y = 0.5
        left_wall_pose.pose.position.z = 0
        left_wall_pose.pose.orientation.z = 0
        left_wall_pose.pose.orientation.w = 1
        scene.add_box("left_wall", left_wall_pose, (0.001, 5, 2))

        # right_wall_pose = geometry_msgs.msg.PoseStamped()
        # right_wall_pose.header.frame_id = "base"
        # right_wall_pose.pose.position.x = 0.38
        # right_wall_pose.pose.position.y = 0.5
        # right_wall_pose.pose.position.z = 0
        # right_wall_pose.pose.orientation.z = 0
        # right_wall_pose.pose.orientation.w = 1
        # scene.add_box("right_wall", right_wall_pose, (0.001, 5, 2))


        # add the roof, which is 1m above the base
        # it is assumed that the roof is 10m wide and 10m deep
        # the roof is 0.01m thick
        roof_pose = geometry_msgs.msg.PoseStamped()
        roof_pose.header.frame_id = "base"
        roof_pose.pose.position.x = 0
        roof_pose.pose.position.y = 0
        roof_pose.pose.position.z = 1
        roof_pose.pose.orientation.w = 1.0
        scene.add_box("roof", roof_pose, (10, 2, 0.01))


        floor_pose = geometry_msgs.msg.PoseStamped()
        floor_pose.header.frame_id = "base"
        floor_pose.pose.position.x = 0
        floor_pose.pose.position.y = -0.5
        floor_pose.pose.position.z = 0.03
        floor_pose.pose.orientation.w = 1.0
        scene.add_box("floor", floor_pose, (0.90, 0.6, 0.01))

    def removeEnvironment(self):
        scene = self.scene
        scene.remove_world_object("wall")
        scene.remove_world_object("left_wall")
        #scene.remove_world_object("right_wall")
        scene.remove_world_object("roof")
        scene.remove_world_object("floor")
    
    def testMotion(self):
        print('Moving')
        move_group = self.move_group
        start_pose = move_group.get_current_pose().pose
        target_pose = move_group.get_current_pose().pose
        target_pose.position.x += 0.2

        self.enforceConstraints(start_pose, target_pose)
        self.moveTo([target_pose.position.x, target_pose.position.y])
        self.clearConstraints()

        self.enforceConstraints(target_pose, start_pose)
        self.moveTo([start_pose.position.x, start_pose.position.y])
        self.clearConstraints()

    def moveTo(self, pose):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose
        
        current_pose.position.x = pose[0]
        current_pose.position.y = pose[1]

        move_group.go(current_pose, wait=True)

        move_group.stop()
        move_group.clear_pose_targets()
    
    def enforceConstraints(self, start_pose, angle=math.pi):
        move_group = self.move_group

        # Calculate the midpoint between start and wpose
        midpoint = geometry_msgs.msg.Pose()
        midpoint.position.x = (start_pose.position.x )
        midpoint.position.y = (start_pose.position.y )
        midpoint.position.z = (start_pose.position.z )

        # Calculate the orientation of the constraint
        roll = 0.0  # Since the constraint is parallel to the line, we can set roll to 0
        pitch = 0.0  # Since the constraint is parallel to the line, we can set pitch to 0
        yaw = angle #math.atan2(normalized_direction[1], normalized_direction[0])  # Calculate the yaw angle based on the direction vector


        constraint_orientation = quaternion_from_euler(roll, pitch, yaw)

        # Set the constraint orientation using the normalized direction vector
        midpoint.orientation.x = constraint_orientation[0]
        midpoint.orientation.y = constraint_orientation[1]
        midpoint.orientation.z = constraint_orientation[2]
        midpoint.orientation.w = constraint_orientation[3]

        # Create a path constraint
        constraints = Constraints()

        # Create a position constraint to enforce a straight-line path
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = move_group.get_planning_frame()
        position_constraint.link_name = move_group.get_end_effector_link()

        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Specify the straight-line constraint region
        constraint_region = BoundingVolume()

        # Create a box-shaped constraint region (change width height and depth to be more or less strict)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        box.dimensions = [1, 0.002, 0.03]  # Adjust the box dimensions as needed

        constraint_region.primitives.append(box)
        constraint_region.primitive_poses.append(midpoint)  # Use the target pose as the constraint region

        position_constraint.constraint_region = constraint_region
        position_constraint.weight = 1

        # Add the position constraint to the path constraints
        constraints.position_constraints.append(position_constraint)

        # Set the path constraints for the MoveGroupCommander
        move_group.set_path_constraints(constraints)

        #################################
        # Visualize the path constraints#
        #################################

        marker_pub = rospy.Publisher('constraint_marker', Marker, queue_size=10)

        marker = Marker()
        marker.header.frame_id = move_group.get_planning_frame()
        marker.ns = "constraint"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = constraint_region.primitive_poses[0]

        marker.scale.x = constraint_region.primitives[0].dimensions[0]
        marker.scale.y = constraint_region.primitives[0].dimensions[1]
        marker.scale.z = constraint_region.primitives[0].dimensions[2]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Set transparency (0.0 = fully transparent, 1.0 = fully opaque)

        marker.lifetime = rospy.Duration(0)

        marker_pub.publish(marker)

        # for debugging
        #time.sleep(5)
    
    def clearConstraints(self):
        move_group = self.move_group

        # Clear the path constraints
        move_group.clear_path_constraints()

        marker_pub = rospy.Publisher('constraint_marker', Marker, queue_size=10)
        marker = Marker()
        marker.header.frame_id = move_group.get_planning_frame()
        marker.ns = "constraint"
        marker.id = 1
        marker.action = Marker.DELETE

        marker_pub.publish(marker)

    def spawnObj(self, center, model_path):
        initial_pose = Pose()
        initial_pose.position.x = center[0]
        initial_pose.position.y = center[1]
        initial_pose.position.z = 0

        with open(model_path, "r") as f:
            model_xml = f.read()

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        try:
            spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            resp_sdf = spawn_sdf("cylinder", model_xml, "/", initial_pose, "world")
        except rospy.ServiceException as e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    def delObj(self):
        rospy.wait_for_service("/gazebo/delete_model")
        try:
            delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            resp_delete = delete_model("cylinder")
        except rospy.ServiceException as e:
            rospy.logerr("Delete Model service call failed: {0}".format(e))


import csv
          
tactile_sensor = "Digit"

mp = Experiment(tactile_sensor)

# center = [-0.132,0.507]
# center = [-0.23,0.517]
n = 20
center = [-0.135,0.650]
interface_z = 0.075
radius = 0.15

offset = 0.7 # 0.5 = interface_z/radius (what percentage of the radius is the interface)

mp.addEnvironment()

# time.sleep(10)

mp.defineStart()
path = mp.circleApproach(n=n, center=center, offset=offset, radius=radius)
mp.moveBack2Start(center)

# mp.moveToObjectCenter(center, offset)

# mp.testMotion()

mp.removeEnvironment()

# # save path to pickle file
# with open('path.pkl', 'wb') as f:
#     pickle.dump(path, f)


# ## Data Saving ###
# try:
#     mp.saveData()
#     print("Data saved")
# except:
#     print("Data not saved")

