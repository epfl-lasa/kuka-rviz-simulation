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
---
##Functionalities:
In order to **simulate** the KUKA LWR robot in the LASA lab with velocity controllers you need to run the following lines of code:

```
$ roslaunch kuka_lwr_bringup lwr_simulation_viz.launch
$ rosrun rviz rviz
```

Once in rviz, 
 1. Add a plugin of type RobotModel
 2. Add the TF plugin

If everything goes well, you should see something like this:

![alt tag](https://cloud.githubusercontent.com/assets/761512/10678185/08057796-7911-11e5-8641-896615534612.png)


This simulation offers a joint velocity/position-resolved controller in ROS. You can send it joint velocity/stiffness or position/stiffness commands and it will follow suit. No dynamics or physics simulation is included. This can be used to test code and trajectories before going on to the real robot.

To test the simulation, you can manually move the robot like so:

```
rostopic pub -r 20 /KUKA/joint_cmd sensor_msgs/JointState '{velocity: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], stiffness: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0]}'
```
Here you are commanding the first joint with a velocity of 0.1rad/s and setting stiffness values for all joint at 200Nm/rad.

An example of it being used in simulation is provided in the [task_motion_planning_cds](https://github.com/nbfigueroa/task_motion_planning_cds) package.

In order to **visualize** the KUKA LWR robot in realtime you will need to install the robot-toolkit package:
```
$ git clone https://github.com/epfl-lasa/robot-toolkit.git
```

For **realtime visualization** there are two options:
If you are using [Nadia's kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages.git) to control the robot in a modular architecture you can start up the environment and visualization as follows:

```
$ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
$ rosrun kuka_fri_bridge run_lwr.sh
$ rosrun rviz rviz
```

If you are using the epfl-lasa standard interface packages ([lwr_interface](https://github.com/epfl-lasa/lwr-interface) and [fri-library-ros](https://github.com/epfl-lasa/fri-library-ros)) you can just run robot_mirror and rviz as follows:

```
$ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
$ roslaunch robot_mirror robot_mirror.launch
$ rosrun rviz rviz
```
---
