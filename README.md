# KUKA LWR @LASA Simulation 
This package runs a simulation of the KUKA LWR robot in the LASA lab (EPFL) with the velocity/position controllers provided the IAI lab (Uni Bremen).

In order to run this code, install the following packages beforehand:
 
```
sudo apt-get install ros-indigo-pr2-mechanism-model ros-indigo-pr2-controller-manager ros-indigo-control-toolbox ros-indigo-pr2-mechanism-controllers
```

In addition to this, you need to have two more repos from code-iai in your workspace:

```
$ git clone https://github.com/code-iai/iai_control_pkgs
$ git clone https://github.com/code-iai/iai_common_msgs
```

##Functionalities:
In order to **simulate** the KUKA LWR robot in the LASA lab with velocity controllers you need to run the following lines of code:

```
$ roslaunch kuka_lwr_bringup lwr_simulation_viz.launch
$ rosrun rviz rviz
```

This simulation "emulates" a joint velocity controller in ROS. You can send it joint velocity/stiffness commands and it will follow suit. No dynamics or physics simulation is included. This can be used to test code and trajectories before going on to the real robot.

In order to **visualize** the KUKA LWR robot in realtime you will need to install the robot-toolkit package:
```
$ git clone https://github.com/epfl-lasa/robot-toolkit.git
```

For **realtime visualization** there are two options:
If you are using [Nadia's kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages.git) to control the robot in a modular architecture you can start up the environment and visualization as follows:

```
$ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
$ rosrun rtk_mirror run_lwr.sh
$ rosrun rviz rviz
```

If you are using the epfl-lasa standard interface packages (lwr_interface and fri-library-ros) you can just run robot_mirror and rviz as follows:

```
$ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
$ roslaunch robot_mirror robot_mirror.launch
$ rosrun rviz rviz
```

##Package Contents:

#####./lasa-robots/kuka_lwr_bringup/launch/  
- lwr2_realtime_viz.launch (main launcher which starts the robot_state_publisher topic)
- lwr2_loopback_sim_no_controllers.launch (intermediate configuration launcher)
- upload_lwr2.launch (model starter and updater)
- world_to_camera_tf_broadcaster_dataset.launch (world to cameras transformation)

#####./lasa-robots/kuka_lwr_bringup/models/  
- /table/table2.urdf.xacro (main table supporting robot)
- /table/operation_table.urdf.xacro (table where task is taking place)
- /pole/pole.urdf.xacro (kinect camera pole)
- /pole/pole2.urdf.xacro (kinect2 camera pole)

#####./lasa-robots/kuka_lwr_description/robots/  
- kuka_lwr_lasa.urdf.xacro (main file to include all models and transformations) -- Configuration 1:Add picture
- kuka_lwr2_lasa.urdf.xacro (main file to include all models and transformations) -- Configuration 2:Add picture
