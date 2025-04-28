#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from hsrb_interface import Robot
from hsrb_interface import geometry
from geometry_msgs.msg import WrenchStamped
import math

class ForceSensorCapture:
    def __init__(self, topic='/hsrb/wrist_wrench/raw'):
        self._force_z = 0.0
        self._force_buf = []
        self._buf_size = 10
        rospy.Subscriber(topic, WrenchStamped, self._callback)

    def _callback(self, msg):
        force_z = msg.wrench.force.z
        self._force_buf.append(force_z)
        if len(self._force_buf) > self._buf_size:
            self._force_buf.pop(0)
        self._force_z = sum(self._force_buf) / len(self._force_buf)

    def get_force_z(self):
        return self._force_z

class GraspFromHumanService:
    def __init__(self):
        rospy.init_node('grasp_from_human_service')

        self.robot = Robot()
        self.whole_body = self.robot.get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.force_sensor = ForceSensorCapture()
        self.release_threshold = rospy.get_param('~release_threshold', 0.6)
        self.timeout = rospy.get_param('~timeout', 15.0)

        self.service = rospy.Service('/grasp_from_human', Trigger, self.handle_grasp)
        rospy.loginfo("[GraspService] Ready to receive grasp requests.")

    def handle_grasp(self, req):
        rospy.sleep(5.0)
        rospy.loginfo("[GraspService] Opening gripper...")
        self.gripper.command(1.2)
        rospy.sleep(2.0)

        rospy.loginfo("[GraspService] Closing gripper...")
        self.gripper.apply_force(1.0)
        rospy.sleep(1.0)

        rospy.loginfo("[GraspService] Waiting for human to release object...")
        initial_force = self.force_sensor.get_force_z()
        start_time = rospy.Time.now()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_force = self.force_sensor.get_force_z()
            diff = abs(current_force - initial_force)
            if diff > self.release_threshold:
                rospy.loginfo("[GraspService] Detected release: %.3f > %.3f" % (diff, self.release_threshold))
                break
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                rospy.logwarn("[GraspService] Timeout waiting for release.")
                return TriggerResponse(success=False, message="Timeout")
            rate.sleep()

        rospy.loginfo("[GraspService] Moving to neutral pose.")
        try:
            self.whole_body.move_to_neutral()
        except Exception as e:
            rospy.logwarn("[GraspService] Failed to move to neutral: %s" % e)

        return TriggerResponse(success=True, message="Grasp complete and human released object.")

if __name__ == '__main__':
    try:
        GraspFromHumanService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
