#Report the state of the cartesian controller
bool running   # Is the controller active?
float64 error  # Represents distance to the goal
float64 error_pos # norm of the cartesian position error
float64 error_rot # norm of the cartesian orientation error
float64 vel_norm # norm of the commanded velocity vector
float64 elapsed_seconds # time since the last goal was set

#geometry_msgs/PoseStamped goal_pose   # Has pose of the running goal
string ee_frame_name  # Frame at the end of the kin. chain
string base_frame_name # Frame at the beginning of the kin. chain


