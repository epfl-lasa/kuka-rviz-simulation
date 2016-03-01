# urdf_management tutorial


## Installation

Bootstrap a new workspace from the rosinstall file in this package using ```wstool``` and ```catkin_build```:
* ```cd /tmp```
* ```git clone git@github.com:code-iai/iai_control_pkgs.git```
* ```cd && mkdir EMPTY-WS-DIR```
* ```cd EMPTY-WS-DIR```
* ```wstool init src /tmp/iai_control_pkgs/urdf_management_tutorial/rosinstall/sample.rosinstall```
* ```catkin init```
* ```catkin build urdf_management_tutorial loopback_controller_manager_examples```
* add ```EMPTY-WS-DIR/devel/setup.bash``` to ```~.bashrc``` and source it

## Start-up
Start a roscore in a new terminal:
  * ```roscore```

Start controller for the joints and set the robot_description parameter to the pr2.urdf in a new terminal:
  * ```roslaunch loopback_controller_manager_examples pr2_all_controllers_simulation_dynamic_state.launch```

Start the urdf management service in a new terminal:
  * ```rosrun urdf_management urdf_management_service```

Start rviz in a new terminal:
  * ```rosrun rviz rviz```

In rviz,
  * set the fixed frame to ```base_link```
  * add a plugin of type ```DynamicRobotModel```

You should see the PR2 in rviz like you would with the normal RobotModel:

![rviz view](doc/pr2.png)


## Adding and removing a link
To add a link type in a new terminal:
  * ```rosrun urdf_management_tutorial add_spatula```

In rviz you should now be able to see a spatula in the left gripper of the PR2.

![rviz view](doc/pr2_with_spatula.png)
![rviz view](doc/pr2_spatula_tf.png)

The spatula is now part of the robot description and connected to the left gripper via a fixed joint. To see the arm moving with the spatula in the gripper type:
  * ```rosrun urdf_management_tutorial move_arm```

![rviz view](doc/pr2_moved_arm.png)

After you added the spatula you can remove it again by typing:
  * ```rosrun urdf_management_tutorial remove_spatula```

You can also remove parts of the initial robot description. To remove the left gripper type:
 * ```rosrun urdf_management_tutorial remove_left_gripper```

![rviz view](doc/pr2_no_left_gripper.png)

Note that the parsing of the urdf will fail when you remove the gripper but still had the spatula attached.
