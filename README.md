First make sure the UR5e is started up and has the external control program running then run this command:

```roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=50002]```

Await the ```Ready to receive control commands.```  message.

Then start up the MoveIt execution planner. This will instasiate a move_group node that can communicate with the robot controllers as specified by the yaml config file of our choice. Currently ros_controllers.yaml is being used but this can be changed via src/universal_robot/ur5e_moveit_config/launch/ur5e_moveit_controller_manager.launch.xml file: 

```roslaunch ur5e_moveit_config moveit_planning_execution.launch```

This starts up our MoveIt server and we are now ready to have a client connected to this. Our options are to either use Rviz:

```roslaunch ur5e_moveit_config moveit_rviz.launch```

The command script provided by MoveIt tutorials: 

```rosrun moveit_commander moveit_commander_cmdline.py```

To use it first enter command ```use manipulator``` which is the name of the movegroup. Then view all possible commands by entering ```help```

Or our own client node: 

``` rosrun interactions sim_test.py ```

This node currently has a basic servoing function where is probes along the XZ plane. It is able to detect when it thinks it is out of bounds of the object and no longer needs to probe and return back to home position. 
