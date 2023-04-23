#!/usr/bin/env python3

import rospy 
import rosservice

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from std_msgs.msg import String
import sys
import copy
from datetime import datetime
import os
import moveit_commander
from moveit_msgs.msg import MoveGroupActionGoal
from cv_bridge import CvBridge
import cv2
import csv

from moveit_commander.conversions import pose_to_list

# UR Imports 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

import time


# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

path = datetime.now().strftime("%d-%m-%Y--%H:%M:%S")
path = '/home/mtl/Desktop/salman_fyp/single_poke/digit/soup/ts_data/'+path
os.mkdir(path)

class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        ########################
        #####  TRAJECTORY ######
        ########################
        rospy.init_node("test_move")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

        ########################
        #####  experiment ######
        ########################

        moveit_commander.roscpp_initialize(sys.argv)
        
        # Subscribe to wrench topic which provides force data 
        rospy.Subscriber("/wrench", WrenchStamped, self.ft_cb)
        rospy.Subscriber("/joint_states", JointState, self.js_cb)
        rospy.Subscriber("/DigitFrames", Image, self.ts_cb)
        self.ts_data = []
        self.ts_ts = []

        # Subscribe to joint states 

        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]

        # FT calibration stuff because zero_ft_sensor rosservice isn't working
        self.calibCounter = 0
        self.forceOffset = [0,0,0]

        self.posList = []

        #self.startJointPosition = [0,0,0,0,0,0]
        #self.currentJointPosition = self.startJointPosition
        #self.startToolPosition

        #self.home = [2.6036441961871546, -1.3802532118609925, -5.318602506314413, 5.047921913653173, 0.9972196221351624, -2.319242302571432]

        # moveit stuff
        #self.robot = moveit_commander.RobotCommander()
        #group_name = "manipulator"
        #self.move_group = moveit_commander.MoveGroupCommander(group_name)
    
    def send_joint_trajectory(self, pos = [2.61, -1.38, -5.32, 5.06, 0.99, -2.31]):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [pos]
        #position_list.append([[2.07, -1.08, -5.00, 5.31, 1.30, -2.31]])
        #position_list.append([[2.61, -1.38, -5.32, 5.06, 0.99, -2.31]])
        duration_list = [3.0, 7.0, 10.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        #self.ask_confirmation(position_list)
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
    
    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        rospy.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == "y"
        if not confirmed:
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def basicMotion(self):
        home = [2.61, -1.38, -5.32, 5.06, 0.99, -2.31]
        self.send_joint_trajectory(home)
        self.send_joint_trajectory([2.07, -1.08, -5.00, 5.31, 1.30, -2.31])
        print(datetime.now().strftime("%H:%M:%S.%f"))
        time.sleep(2)
        self.send_joint_trajectory(home)
    
    def ft_cb(self, data):
        #seq = data.seq
        #print(seq)
        #ts = data.stamp
        #print(ts)
        #print('Force:', self.force)
        #print('Torque: ', self.torque)

        if self.calibCounter<20:
            self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
            self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

            self.forceOffset[0] += +self.force[0]
            self.forceOffset[1] += self.force[1]
            self.forceOffset[2] += self.force[2]
            
            self.calibCounter += 1
        else:
            self.force = [data.wrench.force.x-(self.forceOffset[0]/20), data.wrench.force.y-(self.forceOffset[1]/20), data.wrench.force.z-(self.forceOffset[2]/20)]
            self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    
    def js_cb(self, data):
        #print(data.data.position)
        self.posList.append(data.position)

    def ts_cb(self, data):

        self.ts_data.append([data, datetime.now().strftime("%H:%M:%S.%f")])

    def ts_save(self):
        file = open(path+'/pos.csv', 'x+', newline='')

        with file:
            write = csv.writer(file)
            write.writerows(self.posList)

        for data in self.ts_data:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(data[0])

            fileName = path+'/img_'+ data[1] + '.png'
            cv2.imwrite(fileName,frame)
        

    def basicInfo(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        rospy.loginfo("")

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_current_pose()
        rospy.loginfo("============ End effector link: %s" % eef_link)

    
class UR5eDigitControl:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("ur5e_moveit_interface", anonymous=True)
        
        # Subscribe to wrench topic which provides force data 
        #rospy.Subscriber("/wrench", WrenchStamped, self.ft_cb)
        #rospy.Subscriber("/joint_states", JointState, self.js_cb)

        # Subscribe to joint states 

        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]

        # FT calibration stuff because zero_ft_sensor rosservice isn't working
        self.calibCounter = 0
        self.forceOffset = [0,0,0]

        #self.startJointPosition = [0,0,0,0,0,0]
        #self.currentJointPosition = self.startJointPosition
        #self.startToolPosition

        #self.home = [2.6036441961871546, -1.3802532118609925, -5.318602506314413, 5.047921913653173, 0.9972196221351624, -2.319242302571432]

        # moveit stuff
        self.robot = moveit_commander.RobotCommander()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.client = TrajectoryClient()


    def ft_cb(self, data):
        #seq = data.seq
        #print(seq)
        #ts = data.stamp
        #print(ts)
        #print('Force:', self.force)
        #print('Torque: ', self.torque)

        if self.calibCounter<20:
            self.force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
            self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]

            self.forceOffset[0] += +self.force[0]
            self.forceOffset[1] += self.force[1]
            self.forceOffset[2] += self.force[2]
            
            self.calibCounter += 1
        else:
            self.force = [data.wrench.force.x-(self.forceOffset[0]/20), data.wrench.force.y-(self.forceOffset[1]/20), data.wrench.force.z-(self.forceOffset[2]/20)]
            self.torque = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    
    def js_cb(self, data):
        pass
    
    def getPos(self):
        pass

    def basicInfo(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        rospy.loginfo("")

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_current_pose()
        rospy.loginfo("============ End effector link: %s" % eef_link)

    def jointMotion(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[5] += 0.2

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def basicMotion(self):
        home = [2.61, -1.38, -5.32, 5.06, 0.99, -2.31]
        self.send_joint_trajectory(home)
        self.send_joint_trajectory([2.07, -1.08, -5.00, 5.31, 1.30, -2.31])
        self.send_joint_trajectory(home)

     
if __name__ == '__main__':
    
    experiment = TrajectoryClient()
    time.sleep(2)
    experiment.basicMotion()
    experiment.ts_save()
    #rospy.spin()
