#Header
Header header

# joints we are commanding
string[] joint_names

# commands send to low-level controller
float64[] velocity     #joint velocity in rad/s
float64[] stiffness    #joint stiffness in Nm/rad
float64[] damping      #joint damping (normalized, 0.7 is critically dampped)
