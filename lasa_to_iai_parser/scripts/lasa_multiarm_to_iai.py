#!/usr/bin/env python
# Read on the joint_state_imp topic, and for each message received, answer with des velocities

import rospy
from rospy.numpy_msg import numpy_msg
import numpy

from std_msgs.msg           import Float32MultiArray
from kuka_fri_bridge.msg    import JointStateImpedance
from iai_control_msgs.msg   import MultiJointVelocityImpedanceCommand

def jointStateImpedance_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

class JS_converter(object):
    """convert kuka_fri_bridge/JointStateImpedance message to std_msgs/Float32MultiArray for position controller simulator"""
    def __init__(self, num_dof):
        
        self.num_dof = num_dof
        self.init_work_variables()    
        self.vel_output_topic_type = MultiJointVelocityImpedanceCommand

        self.l_js_sub = rospy.Subscriber("/first_arm_controller/joint_imp_cmd", numpy_msg(JointStateImpedance), self.l_js_cb, queue_size=3, tcp_nodelay=True)
        self.l_out_js_pos_pub = rospy.Publisher("/first_arm_pos_controller/command", Float32MultiArray, queue_size=3, tcp_nodelay=True, latch=False)
        self.l_out_js_vel_pub = rospy.Publisher("/first_arm_vel/command", numpy_msg(self.vel_output_topic_type), queue_size=3, tcp_nodelay=True, latch=False)

        self.r_js_sub = rospy.Subscriber("/second_arm_controller/joint_imp_cmd", numpy_msg(JointStateImpedance), self.r_js_cb, queue_size=3, tcp_nodelay=True)
        self.r_out_js_pos_pub = rospy.Publisher("/second_arm_pos_controller/command", Float32MultiArray, queue_size=3, tcp_nodelay=True, latch=False)
        self.r_out_js_vel_pub = rospy.Publisher("/second_arm_vel/command", numpy_msg(self.vel_output_topic_type), queue_size=3, tcp_nodelay=True, latch=False)

        self.l2_js_sub = rospy.Subscriber("/third_arm_controller/joint_imp_cmd", numpy_msg(JointStateImpedance), self.l2_js_cb, queue_size=3, tcp_nodelay=True)
        self.l2_out_js_pos_pub = rospy.Publisher("/third_arm_pos_controller/command", Float32MultiArray, queue_size=3, tcp_nodelay=True, latch=False)
        self.l2_out_js_vel_pub = rospy.Publisher("/third_arm_vel/command", numpy_msg(self.vel_output_topic_type), queue_size=3, tcp_nodelay=True, latch=False)

        self.r2_js_sub = rospy.Subscriber("/fourth_arm_controller/joint_imp_cmd", numpy_msg(JointStateImpedance), self.r2_js_cb, queue_size=3, tcp_nodelay=True)
        self.r2_out_js_pos_pub = rospy.Publisher("/fourth_arm_pos_controller/command", Float32MultiArray, queue_size=3, tcp_nodelay=True, latch=False)
        self.r2_out_js_vel_pub = rospy.Publisher("/fourth_arm_vel/command", numpy_msg(self.vel_output_topic_type), queue_size=3, tcp_nodelay=True, latch=False)


    def init_work_variables(self):
        self.l_js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32) 
        self.r_js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32) 
        self.l2_js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32) 
        self.r2_js_position = numpy.array([0.0] * self.num_dof, dtype=numpy.float32) 
            
    def stop(self):
        '''Stop the object'''
        self.l_js_sub.unregister()
        self.r_js_sub.unregister()
        self.l2_js_sub.unregister()
        self.r2_js_sub.unregister()
        self.l_out_js_pos_pub.unregister()
        self.l_out_js_vel_pub.unregister()
        self.r_out_js_pos_pub.unregister()
        self.r_out_js_vel_pub.unregister()
        self.l2_out_js_pos_pub.unregister()
        self.l2_out_js_vel_pub.unregister()
        self.r2_out_js_pos_pub.unregister()
        self.r2_out_js_vel_pub.unregister()

        
    def l_js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''        
        #Find values for the desired joints

        if (numpy.size(msg.position) ==  self.num_dof):
            for i in range(self.num_dof):
                 self.l_js_position[i] = msg.position[i]

            # Send values of desired joint positions to iai-compatible topic            
            out_pos_msg = Float32MultiArray()
            out_pos_msg.data = self.l_js_position        
            self.l_out_js_pos_pub.publish(out_pos_msg)


        if (numpy.size(msg.velocity) ==  self.num_dof):
            # Send values of desired joint velocities to iai-compatible topic            
            out_vel_msg = self.vel_output_topic_type()
            out_vel_msg.velocity = msg.velocity
            out_vel_msg.header.stamp = rospy.Time.now()
            self.l_out_js_vel_pub.publish(out_vel_msg)

    def l2_js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''        
        #Find values for the desired joints

        if (numpy.size(msg.position) ==  self.num_dof):
            for i in range(self.num_dof):
                 self.l2_js_position[i] = msg.position[i]

            # Send values of desired joint positions to iai-compatible topic            
            out_pos_msg = Float32MultiArray()
            out_pos_msg.data = self.l2_js_position        
            self.l2_out_js_pos_pub.publish(out_pos_msg)


        if (numpy.size(msg.velocity) ==  self.num_dof):
            # Send values of desired joint velocities to iai-compatible topic            
            out_vel_msg = self.vel_output_topic_type()
            out_vel_msg.velocity = msg.velocity
            out_vel_msg.header.stamp = rospy.Time.now()
            self.l2_out_js_vel_pub.publish(out_vel_msg)            

    def r_js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''        
        #Find values for the desired joints

        if (numpy.size(msg.position) ==  self.num_dof):
            for i in range(self.num_dof):
                 self.r_js_position[i] = msg.position[i]

            # Send values of desired joint positions to iai-compatible topic            
            out_pos_msg = Float32MultiArray()
            out_pos_msg.data = self.r_js_position        
            self.r_out_js_pos_pub.publish(out_pos_msg)


        if (numpy.size(msg.velocity) ==  self.num_dof):
            # Send values of desired joint velocities to iai-compatible topic            
            out_vel_msg = self.vel_output_topic_type()
            out_vel_msg.velocity = msg.velocity
            out_vel_msg.header.stamp = rospy.Time.now()
            self.r_out_js_vel_pub.publish(out_vel_msg)

    def r2_js_cb(self, msg):
        '''Callback function for the joint_states. It searches for the subset of joints, and publishes those on another topic'''        
        #Find values for the desired joints

        if (numpy.size(msg.position) ==  self.num_dof):
            for i in range(self.num_dof):
                 self.r2_js_position[i] = msg.position[i]

            # Send values of desired joint positions to iai-compatible topic            
            out_pos_msg = Float32MultiArray()
            out_pos_msg.data = self.r2_js_position        
            self.r2_out_js_pos_pub.publish(out_pos_msg)


        if (numpy.size(msg.velocity) ==  self.num_dof):
            # Send values of desired joint velocities to iai-compatible topic            
            out_vel_msg = self.vel_output_topic_type()
            out_vel_msg.velocity = msg.velocity
            out_vel_msg.header.stamp = rospy.Time.now()
            self.r2_out_js_vel_pub.publish(out_vel_msg)


def main():
    rospy.init_node('lasa_to_iai', anonymous=True)
    rospy.loginfo("%s: Starting" % (rospy.get_name()))

    convert = JS_converter(7)
    
    rospy.spin()
    
    rospy.loginfo("%s: Exiting" % (rospy.get_name()))
    convert.stop()



if __name__ == '__main__':
    main()

