# KUKA LWR @LASA Simulation and Visualization
![Build Status](https://travis-ci.com/epfl-lasa/kuka-rviz-simulation.svg?token=rqxofpr1d48TwpPKQAeJ&branch=master)

This package runs a simulation of the KUKA LWR robot in the LASA lab (EPFL) with the velocity/position-resolved controllers from the standard pr2 control manager, using some of the IAI lab ctrl packages (Uni Bremen).

### This is an unstable multi-robot branch

In order to run this code, install the following packages beforehand:
 
```
sudo apt-get install ros-indigo-pr2-mechanism-model ros-indigo-pr2-controller-manager ros-indigo-control-toolbox ros-indigo-pr2-mechanism-controllers
```
To use the same message type in simulation and with the real robot via the  [kuka_fri_bridge](https://github.com/nbfigueroa/kuka_interface_packages.git)  you need to download and install the following repo:

```
$ git clone https://github.com/nbfigueroa/kuka_interface_packages
```
and don't forget to install all [dependencies](https://github.com/nbfigueroa/kuka_interface_packages) for this package.

---
## Functionalities:
In order to **simulate** multiple KUKA LWR/IIWAs robot in the LASA lab with velocity controllers you need to run the following lines of code:

### Robot Environments:

#### Multi-Robot Setting 1:
The third setting is the bimanual configuration, i.e. two lwr robot arms, as shown below:

![alt tag](https://cloud.githubusercontent.com/assets/761512/13420097/8e7f9012-df83-11e5-8422-c0f2f381f964.png)

To run the bimanual simulation:
```
$ roslaunch kuka_lwr_bringup bimanual_simulation.launch
```

Test velocity control left arm:
```
rostopic pub -r 20 /l_arm_controller/joint_imp_cmd kuka_fri_bridge/JointStateImpedance '{velocity: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

Test velocity control right arm:
```
rostopic pub -r 20 /r_arm_controller/joint_imp_cmd kuka_fri_bridge/JointStateImpedance '{velocity: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

Test position control left arm:
```
rostopic pub -r 20 /l_arm_controller/joint_imp_cmd kuka_fri_bridge/JointStateImpedance '{position: [0.0, -1.0, 0.0, -10.0, 0.0, 0.0, 130]}'

```

test position control right arm:
```
rostopic pub -r 20 /r_arm_controller/joint_imp_cmd kuka_fri_bridge/JointStateImpedance '{position: [0.0, -1.0, 0.0, -10.0, 0.0, 0.0, 130]}'
```

#### Multi-Robot Setting 2:

### Modify/Create Environments:
To modify the simulation environment (i.e. position of the robo/table, add more robots/tables/objects) go to the following directory and create your own urdf.xacro file:
```
~/kuka-rviz-simulation/kuka_lwr_bringup/kuka_lwr_description/robots/kuka_bimanual_lwr_lasa.urdf.xacro

```

To modify initial joint configuration of the robot, modify the following file:
```
~/kuka-rviz-simulation/kuka_lwr_bringup/config/bimanual_lwr_start_config.yaml
```
# Troubleshooting

1. The robots don't move in the simulator, there is no error!
 ->Kill the roscore!
