#!/usr/bin/env python
#
#    Simple boxy joint controller
#
#    Copyright (c) 2014 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#    Authors: Alexis Maldonado  <amaldo@cs.uni-bremen.de>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Simple idea, read on the joint_state topic, and for each message received, answer with des velocities

import rospy
from rospy.numpy_msg import numpy_msg
import numpy

from iai_control_utils.jcontroller import JController

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

#The supported velocity output types:
from iai_control_msgs.msg import MultiJointVelocityImpedanceCommand
from iai_control_msgs.msg import MultiJointVelocityCommand

 
def main():
    rospy.init_node('boxy_joint_controller', anonymous=False)
    rospy.loginfo("%s: Starting" % (rospy.get_name()))

    #Read the parameters for configuration
    vel_output_topic_type = rospy.get_param('~vel_command_type', 'MultiJointVelocityImpedanceCommand')
    max_speed = rospy.get_param('~max_speed', 0.3) #rads/s
    stiffness = rospy.get_param('~stiffness', 100) #Nm/m
    
    p_gain = rospy.get_param('~p_gain', 3.5) #P constant of the P controller

    rospy.loginfo('%s: max_speed=%f p_gain=%f' %(rospy.get_name(), max_speed, p_gain))
    #Instantiate the controller class (starts on its own)
    cont = JController(vel_output_topic_type, max_speed, p_gain, stiffness)

    
    #Register function to call when stopping (to stop the motors)
    rospy.on_shutdown(cont.stop)
    
    #r = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    #Publish diagnostics at slow rate
    #    pass
    #    r.sleep()
    
    #Block until the ROS node is shut down. Let the other threads do the work.
    rospy.spin()
                
    rospy.loginfo("%s: Exiting" % (rospy.get_name()))
        
    
    

if __name__ == '__main__':
    main()
    
