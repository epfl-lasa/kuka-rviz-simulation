#!/usr/bin/env python

import roslib
import rospy

from iai_control_msgs.msg import MultiJointVelocityImpedanceCommand

from optparse import OptionParser

parser = OptionParser()
parser.add_option("-s", "--stiffness", dest="stiffness",
                  help="Stiffness commanded to each joint (STIFFNESS) Default=%default (Nm/rad)", metavar="STIFFNESS", default= 80.0)
parser.add_option("-d", "--damping", dest="damping",
                  help="Damping commanded to each joint (DAMPING) Default=%default", metavar="DAMPING", default= 0.7)
parser.add_option("-n", "--number", dest="number",
                  help="Joint number commanded (NUMBER) Default=%default", metavar="NUMBER", default= 6)
parser.add_option("-t", "--topic", dest="topic",
                  help="Command topic Default=%default", metavar="TOPIC", default="/r_arm_vel/command")


(options, args) = parser.parse_args()

def main():
    rospy.init_node("cmd_arm_vel_imp", anonymous=True)

    pub = rospy.Publisher(options.topic, MultiJointVelocityImpedanceCommand, queue_size=1, latch=True)
    rospy.loginfo("Publishing to:")
    print options.topic

    command_vel_step(pub, 0.05, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.05, options.number, rospy.Duration(1.0))

    command_vel_step(pub, 0.1, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.1, options.number, rospy.Duration(1.0))

    command_vel_step(pub, 0.15, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.15, options.number, rospy.Duration(1.0))

    command_vel_step(pub, 0.2, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.2, options.number, rospy.Duration(1.0))

    command_vel_step(pub, 0.25, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.25, options.number, rospy.Duration(1.0))

    command_vel_step(pub, 0.3, options.number, rospy.Duration(1.0))
    command_vel_step(pub, -0.3, options.number, rospy.Duration(1.0))


def command_vel_step(pub, amplitude, joint_nr, duration):
  
    cmd = MultiJointVelocityImpedanceCommand()
    cmd.velocity = [ 0.0 ] * 7
    cmd.velocity[ joint_nr ] = amplitude
    cmd.stiffness = [ float(options.stiffness) ] * 7
    cmd.damping = [ float(options.damping) ] * 7

    rospy.loginfo("Publishing the following command:")
    print cmd

    start_time = rospy.Time.now()
    while((not rospy.is_shutdown()) and (rospy.Time.now() < start_time + duration)):
            cmd.header.stamp = rospy.Time.now()
	    pub.publish(cmd)
            rospy.sleep(0.05)

    rospy.loginfo("Done publishing")

if __name__ == "__main__":
    main()
