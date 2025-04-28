#!/usr/bin/env python3

import rospy
from interactive_robot.srv import GetObjectMarker, ManipulateToObject
from std_srvs.srv import Trigger

class MainController:
    def __init__(self):
        rospy.init_node('main_controller')

        self.object_name = rospy.get_param("~object_name", "bottle")
        self.action = rospy.get_param("~action", "get bottle").lower()

        rospy.loginfo(f"[MainController] Object: {self.object_name}, Action: {self.action}")

        self.run()

    def wait_and_call(self, service_name, srv_type, request=None):
        rospy.loginfo(f"[MainController] Waiting for {service_name}...")
        rospy.wait_for_service(service_name)
        try:
            proxy = rospy.ServiceProxy(service_name, srv_type)
            response = proxy(request) if request else proxy()
            rospy.loginfo(f"[MainController] {service_name} call success.")
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"[MainController] Failed to call {service_name}: {e}")
            return None

    def run(self):
        if self.action.startswith("get"):
            # 1. Detect object
            resp = self.wait_and_call('/get_object_marker', GetObjectMarker, self.object_name)
            if not resp:
                return
            pose = resp.object_position

            # 2. Move to the object
            if not self.wait_and_call('/manipulate_object', ManipulateToObject, pose).success:
                return

            # 3. Grasp from table
            if not self.wait_and_call('/grasp_from_table', Trigger).success:
                return

            # 4. Go near the human hand
            if not self.wait_and_call('/handover_to_human', Trigger).success:
                return

            # 5. Open the gripper to release
            self.wait_and_call('/release_object', Trigger)

        elif self.action.startswith("keep"):
            # 1. Go near the human hand
            if not self.wait_and_call('/handover_to_human', Trigger).success:
                return

            # 2. Grasp from the human hand
            if not self.wait_and_call('/grasp_from_human', Trigger).success:
                return

        else:
            rospy.logerr(f"[MainController] Invalid action: {self.action}")

if __name__ == '__main__':
    try:
        MainController()
    except rospy.ROSInterruptException:
        pass
