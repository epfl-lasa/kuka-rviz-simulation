# KUKA LWR @LASA Simulation 
This package runs a simulation of the KUKA LWR robot in the LASA lab (EPFL) with the velocity-resolved controllers provided the IAI lab (Uni Bremen).

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

![alt tag](https://cloud.githubusercontent.com/assets/761512/10713506/56d76c5e-7ac3-11e5-9e3d-20fae14158c2.png)


This simulation offers a joint velocity-resolved interface for the KUKA LWR robot in ROS. You can send it joint velocity/stiffness commands and it will follow suit. No dynamics or physics simulation is included. This can be used to test code and trajectories before going on to the real robot.

To test the simulation, you can manually move the robot like so:

```
rostopic pub -r 20 /r_arm_vel/command iai_control_msgs/MultiJointVelocityImpedanceCommand '{velocity: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], stiffness: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0]}'

```
Here you are commanding the first joint with a velocity of 0.5rad/s and setting stiffness values for all joints at 200Nm/rad.

Thus, to use it in your project, you should publish the ```/r_arm_vel/command``` topic. The current robot state is published by the simulation on the ```/joint_states``` topic which is of the type ```sensor_msgs/JointState```. An example of it being used in simulation is provided in the [task_motion_planning_cds](https://github.com/nbfigueroa/task_motion_planning_cds) package.


---
You can also use this package as a **realtime visualization** of your experiments, there are two options:
 1. If you are using [Nadia's kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages.git)   which control the robot in a modular architecture using the ```kuka_fri_bridge```  and [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit.git) you can start up the environment and visualization as follows:

 ```
 $ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
 $ rosrun kuka_fri_bridge run_lwr.sh
 $ rosrun rviz rviz
 ```
The ```kuka_fri_bridge``` will be publishing the ```/joint_states```topic.

 2. If you are using the epfl-lasa standard interface packages ([lwr_interface](https://github.com/epfl-lasa/lwr-interface) and [fri-library-ros](https://github.com/epfl-lasa/fri-library-ros)) within your control node/module in [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit.git), you can just start-up robot_mirror and rviz as follows:

 ```
 $ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
 $ roslaunch robot_mirror robot_mirror.launch
 $ rosrun rviz rviz
 ```
 The ```robot_mirror``` will be publishing the ```/joint_states```topic.
 
If you are controlling the robot in some other way, you can still use this package, you only need to publish ```/joint_states```topic.

---

###Robot Environments:

####Robot Setting 1:

The simulation shown above is using the ```lwr_simulation_viz.launch``` it shows the robot arm mounted on a table with a pole to its left *(right corner of the lab - next to the IIWA robot)*, the pole depicts the Kinect mounted on top, which is already calibrated wrt. the robot base frame in the model. Thus, in realtime mode (```lwr_realtime_viz.launch```) if you turn on the Kinect you will visualize the point clouds calibrated wrt. the robot and will be able to compute target frames in the robot reference frame as below:

![alt tag](https://cloud.githubusercontent.com/assets/761512/10713448/f3a4ffbe-7abf-11e5-979a-fc1b6c956fd8.png)

Here the target attractors are being computed for the dough rolling task. If you want to use the Kinect to compute target frames for manipulation go to the [kinect-recognition](https://github.com/epfl-lasa/kinect-recognition) where you can find the implementation of dough recognition and attractor estimation for the Robohow dough rolling task.

####Robot Setting 2:

We also have a second setting: ```lwr2_simulation_viz.launch``` and ```lwr2_realtime_viz.launch``` which has the robot arm mounted on another table *(the one in front of the lab entrance)* and an operation table in front of it. Also, it has the Kinect 2 mounted on a pole and facing the operation table and a Kinect 1 on the side of the operation table. Both kinects are calibrated to the robot base frame, as shown below:

![alt tag](https://cloud.githubusercontent.com/assets/761512/10713496/87c1669a-7ac2-11e5-8171-a8e281fa36d6.png)
