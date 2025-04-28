#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from hsrb_interface import Robot
from geometry_msgs.msg import WrenchStamped

class ForceSensorCapture:
    def __init__(self, topic='/hsrb/wrist_wrench/raw'):
        self._force_z = 0.0
        self._buffer = []
        self._buf_size = 10
        rospy.Subscriber(topic, WrenchStamped, self._callback)

    def _callback(self, msg):
        fz = msg.wrench.force.z
        self._buffer.append(fz)
        if len(self._buffer) > self._buf_size:
            self._buffer.pop(0)
        self._force_z = sum(self._buffer) / len(self._buffer)

    def get_force_z(self):
        return self._force_z

class HandoverToHumanService:
    def __init__(self):
        rospy.init_node('object_handover')

        self.robot = Robot()
        self.whole_body = self.robot.get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.force_sensor = ForceSensorCapture()

        self.drop_threshold = rospy.get_param('~drop_threshold', 0.6)
        self.timeout = rospy.get_param('~timeout', 150.0)

        self.service = rospy.Service('/release_object', Trigger, self.handle_handover)
        rospy.loginfo("[HandoverService] Ready to release object to human.")

    def handle_handover(self, req):
        rospy.loginfo("[HandoverService] Measuring initial holding force.")
        rospy.sleep(1.0)
        initial_force = self.force_sensor.get_force_z()
        rospy.loginfo("[HandoverService] Initial Z force: %.3f", initial_force)

        rospy.loginfo("[HandoverService] Waiting for force drop...")
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_force = self.force_sensor.get_force_z()
            force_drop = initial_force - current_force

            if force_drop > self.drop_threshold:
                rospy.loginfo("[HandoverService] Detected object taken by human. ΔFz = %.3f", force_drop)
                break

            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                rospy.logwarn("[HandoverService] Timeout waiting for human grasp.")
                return TriggerResponse(success=False, message="Timeout")

            rate.sleep()

        rospy.loginfo("[HandoverService] Releasing object.")
        self.gripper.command(1.2)
        rospy.sleep(1.0)

        rospy.loginfo("[HandoverService] Moving to neutral pose.")
        try:
            self.whole_body.move_to_neutral()
        except Exception as e:
            rospy.logwarn("[HandoverService] Failed to return to neutral: %s", e)

        return TriggerResponse(success=True, message="Object handed over and arm reset.")

if __name__ == '__main__':
    try:
        HandoverToHumanService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
