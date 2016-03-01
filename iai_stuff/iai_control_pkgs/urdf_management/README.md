urdf_management
================

Package to alter the robot description at runtime.
Uses the robot_descritpion from the parameter server as initial description and publishes the altered robot descriptions as a string to ```/dynamic_robot_description```.

## Start
With a running roscore type in a new terminal:
  * ```roscd urdf_management```
  * ```rosrun urdf_management urdf_management_service```

## Usage
The service takes 3 arguments ```action```, ```xml_elements_to_add``` and ```element_names_to_remove```.
  * ```action```: ```ADD``` to add the ```xml_elements_to_add``` to the robot description, ```REMOVE``` to remove the ```element_names_to_remove``` from the robot description
  * ```xml_elements_to_add```: urdf like xml description of the links and joints to add to the robot description. 
  * ```element_names_to_remove```: list of names of links and joints that should be removed from the robot descritpion.

If the robot description should be parsed to urdf, mind that there can only be one root link and the service doesn't check for this.

## Examples
Add a link and a joint:
```
rosservice call /alter_urdf 1 "<link name=\"new_link\"/> <joint name=\"new_joint\" type=\"fixed\"><parent link=\"root_link\"/><child link=\"new_link\"/></joint>" '[]'
```

Remove a link and a joint:
```
rosservice call /alter_urdf 2 "" '["new_link", "new_joint"]'
```
