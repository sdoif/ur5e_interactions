#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
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
            rospy.wait_for_service("/ur_hardware_interface/zero_ftsensor", timeout=5)
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
        rospy.Subscriber("/DigitFrames", Image, self.digit_cb)
        self.bridge = CvBridge()
        self.digit = None

        self.test_interrupt = 0
        ## Subscribe to Interrupt topic to obtain interrupt signal AND UPDATE self.test_interrupt
        rospy.Subscriber("/Interrupt", Int16, self.interrupt_cb)


        self.tactile_sensor = ts

    def interrupt_cb(self, msg):
        print(msg.data)
        self.test_interrupt = msg.data
        
    def go_to_joint_state(self):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()

        # loop through each joint and increment by 0.1
        for i in range(len(joint_goal)):
            print(f'Moving joint {i}')
            joint_goal[i] += 0.1
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            time.sleep(1)

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)

    def setJoint(self, angle, joint=4):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()

        joint_goal[joint] = angle
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):

        move_group = self.move_group

        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
    
    def wrench_cb(self, data):
        self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z] 

    def digit_cb(self, data):
        rospy.loginfo("digit")
        ## convert data.data using cvbridge
        frame = self.bridge.imgmsg_to_cv2(data)        
        # obtain cartesian coordinates at current position
        # store current force, coordinates, frames, and time in self.data dictionary
        # format time as day-month-year-hour-minute-second-millisecond
        current_position =  self.move_group.get_current_pose().pose
        self.data['force'].append(self.force)
        self.data['position'].append(current_position)
        self.data['tactile'].append(frame)
        self.data['time'].append(datetime.now().strftime("%d-%m-%Y-%H-%M-%S-%f"))

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
    
    def generateCircleIncrement(self, start, cx = 0, cy = 0.5, n=8):
        # radius is magnitude between [cx,cy] and [start.position.x, start.position.y]
        r = math.sqrt((start.position.x - cx)**2 + (start.position.y - cy)**2)

        # create list of waypoints
        waypoints_list = []

        for i in range(n):

            angle = 2 * math.pi * i / n
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)

            waypoints_list.append([x,y])
        
        # keep only values where x=>0, then order in inreasing y
        waypoints_list = [wp for wp in waypoints_list if wp[0] >= cx]
        waypoints_list = [wp for wp in waypoints_list if wp[1] <= cy]
        waypoints_list.sort(key=lambda x: x[1])

        return waypoints_list
    
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
        return angle
    
    def circleApproach(self, n = 8, center = [0,0.5]):

        move_group = self.move_group
        start = self.move_group.get_current_pose().pose   
        cx = center[0]
        cy = center[1]
        r = abs(cy) - abs(start.position.y)

        waypoints_list = self.generateCircleIncrement(start, cx=cx, cy=cy, n=n)
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
            #waypoints.append(copy.deepcopy(wpose))
            path.append(copy.deepcopy(wpose))
    
            #(plan, fraction) = move_group.compute_cartesian_path(
            #    waypoints, 0.01, 0.0 # waypoints to follow  # eef_step
            #)  # jump_threshold 
            #plan.joint_trajectory.points[0].time_from_start = Duration.from_sec(0.0005)
            #move_group.execute(plan, wait=True)
            # print joint angles

            move_group.go(wpose, wait=True)

            #print(fraction)
            centerPath = self.moveToObjectCenter([cx,cy])
            move_group.go(wpose, wait=True)
            path.append(copy.deepcopy(centerPath))

        return path

    def moveToObjectCenter(self, center):
        print('Starting moveToObjectCenter')
        # move to the center of the object
        move_group = self.move_group

        wpose = move_group.get_current_pose().pose
        wpose.position.x = center[0]
        wpose.position.y = center[1]
        move_group.go(wpose, wait=False)
        threshold = 0.01
        # continoually command the robot to move to the center of the object until self.force[2] > 3
        path = []
        timeout = 0
        while True:
            current_pose = move_group.get_current_pose().pose
            path.append(current_pose)
            if self.test_interrupt > 3:
                print('Force exceeded threshold. Aborting motion.')
                move_group.stop()
                time.sleep(2)  # Pause for 2 seconds
                move_group.clear_pose_targets()
                self.test_interrupt = 0
                break
            # TODO: else if it reaches within a threshold of the center of the object, break
            elif abs(current_pose.position.x - center[0]) < threshold and abs(current_pose.position.y - center[1]) < threshold:
                timeout += 1
                if timeout > 10000:
                    print('Timeout. Aborting motion.')
                    move_group.stop()
                    time.sleep(2)  # Pause for 2 seconds
                    break
                    # Perform linear interpolation towards the goal pose
            else:
                current_position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
                goal_position = np.array([center[0], center[1], current_pose.position.z])
                distance = np.linalg.norm(current_position - goal_position)

                if distance > threshold:
                    t = threshold / distance
                    intermediate_position = (1 - t) * current_position + t * goal_position

                    intermediate_pose = current_pose
                    intermediate_pose.position.x = intermediate_position[0]
                    intermediate_pose.position.y = intermediate_position[1]
                    intermediate_pose.position.z = intermediate_position[2]
                    move_group.go(intermediate_pose, wait=False)

        return path
                
    def interpolateAndMove(self, start_pose, end_pose):
        print('Starting interpolateAndMove')
        # move to the start pose
        move_group = self.move_group
        move_group.go(start_pose, wait=False)

        # Linear interpolation
        num_steps = 100  # Number of interpolation steps
        delta = 1.0 / num_steps

        # Compute and execute the path step by step
        for i in range(num_steps + 1):
            t = i * delta
            # Interpolate position
            interp_x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
            interp_y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
            interp_z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)

            # Create the interpolated pose
            interp_pose = Pose()
            interp_pose.position.x = interp_x
            interp_pose.position.y = interp_y
            interp_pose.position.z = interp_z
            

            # Command the robot to move to the interpolated pose
            move_group.go(interp_pose, wait=False)

            # Check for interruption condition
            if self.test_interrupt > 3:
                print('Force exceeded threshold. Aborting motion.')
                move_group.stop()
                time.sleep(2)  # Pause for 2 seconds
                move_group.clear_pose_targets()
                self.test_interrupt = 0
                break

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
        
    def moveActualHome(self):
        move_group = self.move_group
        
        wpose = self.actual_home
        wpose.position.x = 0
        waypoints = []

        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)     

    def moveStandardHome(self):
        move_group = self.move_group
        
        wpose = self.aactual_home
        waypoints = []

        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

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
        wall_pose.pose.position.y = 0.2
        wall_pose.pose.position.z = 0
        wall_pose.pose.orientation.z = 0.7071
        wall_pose.pose.orientation.w = 0.7071
        scene.add_box("wall", wall_pose, (0.001, 5, 2))

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

        roof_pose = geometry_msgs.msg.PoseStamped()
        roof_pose.header.frame_id = "base"
        roof_pose.pose.position.x = 0
        roof_pose.pose.position.y = 0
        roof_pose.pose.position.z = -0.05
        roof_pose.pose.orientation.w = 1.0
        scene.add_box("floor", roof_pose, (10, 2, 0.01))

    def removeEnvironment(self):
        scene = self.scene
        scene.remove_world_object("wall")
        scene.remove_world_object("roof")
        scene.remove_world_object("floor")


import csv
          
tactile_sensor = "Digit"

mp = Experiment(tactile_sensor)

# test add environment
#mp.addEnvironment()



# #wait for user to press enter
# path = mp.moveToObjectCenter([0,0.9])
path = mp.circleApproach(20)
mp.moveActualHome()

# save path to pickle file
with open('path.pkl', 'wb') as f:
    pickle.dump(path, f)



#mp.removeEnvironment()

# ## Data Saving ###
# try:
#     mp.saveData()
#     print("Data saved")
# except:
#     print("Data not saved")
