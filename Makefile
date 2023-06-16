
start_ur:
	roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=50002]

start_sim:
	roslaunch ur_gazebo ur5e_bringup.launch

start_moveit:
	gnome-terminal --tab	
	roslaunch ur5e_moveit_config moveit_planning_execution.launch&
	gnome-terminal --tab	
	roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz&
	gnome-terminal --tab	

start_moveit_sim:
	roslaunch ur_gazebo ur5e_bringup.launch &
	roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true &
	roslaunch ur5e_moveit_config moveit_rviz.launch