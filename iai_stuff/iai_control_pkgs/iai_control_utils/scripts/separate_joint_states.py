#!/usr/bin/env python
#
#    Separate a unique joint_states topic into several ones (Needed for the simulation)
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

# Simple idea, read on the joint_state topic, load data in a map, and output to separate joint_state topics for each device

import rospy
from rospy.numpy_msg import numpy_msg
import numpy

from sensor_msgs.msg import JointState


class JSseparator(object):
    """Separate /joint_states into sub topics with a subset of the joints"""
    def __init__(self, joint_names):
        
        self.joint_names = joint_names
        self.num_dof = len(self.joint_names)
        self.init_work_variables()
        
        self.js_sub = rospy.Subscriber("~in_joint_states", numpy_msg(JointState), self.js_cb, queue_size=3, tcp_nodelay=True)
        self.out_js_pub = rospy.Publisher("~out_joint_states", numpy_msg(JointState), queue_size=3, tcp_nodelay=True, latch=False)
        
    def init_work_variables(self):
        self.js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32)      
        self.js_velocity = numpy.array([0.0] * self.num_dof, dtype=numpy.float32)
        self.js_effort = numpy.array([0.0] * self.num_dof, dtype=numpy.float32)
        
            
    def stop(self):
        '''Stop the object'''
        self.js_sub.unregister()
        self.out_js_pub.unregister()
        
        
    def js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''
        
        #Find the new values for the desired joints
        for i in range(self.num_dof):
            for j in range(len(msg.name)):
                if (msg.name[j] == self.joint_names[i]):
                    self.js_position[i] = msg.position[j]
                    self.js_velocity[i] = msg.velocity[j]
                    self.js_effort[i] = msg.effort[j]
        #FIXME: Check that all values were found or at least warn
        
        #send the new joint_state message out, with only the desired joints in it
        if (self.out_js_pub.get_num_connections() > 0):
            out_msg = JointState()
            out_msg.header.stamp = msg.header.stamp
            out_msg.name = self.joint_names
            out_msg.position = self.js_position
            out_msg.velocity = self.js_velocity
            out_msg.effort = self.js_effort
            self.out_js_pub.publish(out_msg)

def main():
    rospy.init_node('separate_joint_states', anonymous=True)
    rospy.loginfo("%s: Starting" % (rospy.get_name()))

    #Read the parameters for configuration
    joint_names = rospy.get_param('~joint_names', [])
       
    rospy.loginfo('%s: joint_names=%s' %(rospy.get_name(), joint_names))

    sep = JSseparator(joint_names)
    
    rospy.spin()
    
    rospy.loginfo("%s: Exiting" % (rospy.get_name()))
    sep.stop()

if __name__ == '__main__':
    main()
