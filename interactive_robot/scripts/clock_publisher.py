#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

class ClockRepublisher:
    def __init__(self):
        rospy.init_node('clock_publisher_node')
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        rospy.Subscriber('/hsrb/joint_states', JointState, self.joint_states_callback)
        rospy.loginfo("[HackClock] Republishing joint_states timestamps as /clock.")

    def joint_states_callback(self, msg):
        clock_msg = Clock()
        clock_msg.clock = msg.header.stamp
        self.clock_pub.publish(clock_msg)

if __name__ == '__main__':
    try:
        ClockRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
