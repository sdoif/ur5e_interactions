#! /bin/bash

# this is a script to start up the ROS nodes for the robot
# it will be run on Terminator
# it will first divide the screen into 4 panes
# then it will start the roscore in the top left pane
# then it will start the robot driver in the top right pane
# then it will start the robot controller in the bottom left pane
# then it will start the robot simulator in the bottom right pane

terminator --layout="custom" \
           --profile="default" \
           --title="ROS" \
           --command=""
           
xdotool key ctrl+shift+O
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=50002]

xdotool key alt+Right
roslaunch ur5e_moveit_config moveit_planning_execution.launch



