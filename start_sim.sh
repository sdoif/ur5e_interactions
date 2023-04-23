#! /bin/bash

gnome-terminal --command='roslaunch ur_gazebo ur5e_bringup.launch' --tab
gnome-terminal --command='roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true' --tab
gnome-terminal --command='roslaunch ur5e_moveit_config moveit_rviz.launch' --tab