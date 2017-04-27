#!/usr/bin/env python
# Read on the joint_state_imp topic, and for each message received, answer with des velocities

import rospy
from rospy.numpy_msg import numpy_msg
import numpy

from sensor.msg             import JointState

def jointStateImpedance_callback(data):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

class JS_merger(object):
    """merge /arm_1/joint_states + /arm_2/joint_states"""
    def __init__(self, num_dof):
        
        self.num_dof = num_dof
        self.init_work_variables()    
        self.output_topic_type = JointState

        self.l_js_sub = rospy.Subscriber("/first_arm/joint_states", numpy_msg(JointState), self.l_js_cb, queue_size=1, tcp_nodelay=True)
        self.r_js_sub = rospy.Subscriber("/second_arm/joint_states", numpy_msg(JointState), self.r_js_cb, queue_size=1, tcp_nodelay=True)
        self.js_pub   = rospy.Publisher("/joint_states",        numpy_msg(self.output_topic_type), queue_size=1, tcp_nodelay=True, latch=False)

    def init_work_variables(self):
        self.js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32) 
                             
            
    def stop(self):
        '''Stop the object'''
        self.l_js_sub.unregister()
        self.r_js_sub.unregister()        
        self.out_js_pub.unregister()
        
    def js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''
        
        #Find values for the desired joints

        if (numpy.size(msg.position) ==  self.num_dof):
            for i in range(self.num_dof):
                 self.js_position[i] = msg.position[i]

            # Send values of desired joint positions to iai-compatible topic
            
            out_pos_msg = Float32MultiArray()
            out_pos_msg.data = self.js_position        
            self.out_js_pos_pub.publish(out_pos_msg)


        if (numpy.size(msg.velocity) ==  self.num_dof):
            

            # rospy.loginfo("\nJoint Velocity Command [%f,%f,%f,%f,%f,%f,%f]", 
                # msg.velocity[0],msg.velocity[1],msg.velocity[2],msg.velocity[3],
                # msg.velocity[4],msg.velocity[5],msg.velocity[6])

            # Send values of desired joint velocities to iai-compatible topic            
            out_vel_msg = self.vel_output_topic_type()
            out_vel_msg.velocity = msg.velocity
            out_vel_msg.header.stamp = rospy.Time.now()
            self.out_js_vel_pub.publish(out_vel_msg)


 
 


def main():
    rospy.init_node('lasa_to_iai', anonymous=True)
    rospy.loginfo("%s: Starting" % (rospy.get_name()))

    convert = JS_merger(14)
    
    rospy.spin()
    
    rospy.loginfo("%s: Exiting" % (rospy.get_name()))
    convert.stop()



if __name__ == '__main__':
    main()

