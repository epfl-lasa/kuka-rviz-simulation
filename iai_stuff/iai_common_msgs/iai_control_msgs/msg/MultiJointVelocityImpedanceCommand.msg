#Header
Header header

# Command to move a set of velocity-resolved robot joints 
float32[] velocity     #joint velocity in rad/s
float32[] stiffness    #joint stiffness in Nm/rad
float32[] damping      #joint damping (normalized, 0.7 is critically dampped)

# Experimental field: DO NOT USE unless you really know what you are doing!
float32[] add_torque   #additional joint torque in Nm
