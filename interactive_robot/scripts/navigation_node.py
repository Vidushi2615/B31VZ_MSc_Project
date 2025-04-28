#!/usr/bin/env python3
import rospy
import tf
import uuid
import math
from geometry_msgs.msg import PointStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from interactive_robot.srv import NavigateToObject, NavigateToObjectResponse

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')
        self.goal_pub = rospy.Publisher('/move_base/move/goal', MoveBaseActionGoal, queue_size=1)
        self.tf_listener = tf.TransformListener()

        self.offset_distance = rospy.get_param("~offset_distance", 0.0)
        self.target_frame = rospy.get_param("~target_frame", "odom")

        self.right_shift = 0.141  # HSRB arm_x offset
        self.current_goal_id = None
        self.goal_status = None

        rospy.Subscriber('/move_base/move/status', GoalStatusArray, self.status_callback)

        self.service = rospy.Service('/navigate_to_object', NavigateToObject, self.handle_navigation)
        rospy.loginfo("Navigation Service is ready.")

    def status_callback(self, msg):
        for status in msg.status_list:
            if status.goal_id.id == self.current_goal_id:
                self.goal_status = status.status

    def handle_navigation(self, req):
        rospy.loginfo("Received navigation request.")
        target = req.target

        latest_object = PointStamped()
        latest_object.header.stamp = rospy.Time(0)
        latest_object.header.frame_id = target.header.frame_id
        latest_object.point = target.point

        try:
            self.tf_listener.waitForTransform(
                self.target_frame,
                latest_object.header.frame_id,
                rospy.Time(0),
                rospy.Duration(3.0)
            )
            point_in_map = self.tf_listener.transformPoint(self.target_frame, latest_object)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return NavigateToObjectResponse(success=False)

        x = point_in_map.point.x
        y = point_in_map.point.y
        rospy.loginfo(f"Transformed object position in '{self.target_frame}': ({x:.2f}, {y:.2f})")

        dx = x
        dy = y
        distance = math.hypot(dx, dy)
        if distance < 0.05:
            rospy.logwarn("Object too close to compute a valid offset.")
            return NavigateToObjectResponse(success=False)

        # Calculate offset goal position behind the object
        ratio = max((distance - self.offset_distance) / distance, 0)
        goal_x = dx * ratio
        goal_y = dy * ratio

        # Calculate rightward shift vector (perpendicular to forward)
        ux = dx / distance
        uy = dy / distance
        right_x = uy
        right_y = -ux

        # Apply rightward offset
        final_x = goal_x + self.right_shift * right_x
        final_y = goal_y + self.right_shift * right_y

        # Orientation (facing towards the object)
        yaw = math.atan2(dy, dx)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # Publish goal
        goal_id = str(uuid.uuid4())
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal_id.id = goal_id
        goal_msg.goal.target_pose.header.frame_id = self.target_frame
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.pose.position.x = final_x
        goal_msg.goal.target_pose.pose.position.y = final_y
        goal_msg.goal.target_pose.pose.position.z = 0.0
        goal_msg.goal.target_pose.pose.orientation = Quaternion(*q)

        self.goal_status = None
        self.current_goal_id = goal_id
        self.goal_pub.publish(goal_msg)

        rospy.loginfo(f"Published goal at ({final_x:.2f}, {final_y:.2f}), yaw: {yaw:.2f}")

        timeout = rospy.Time.now() + rospy.Duration(45.0)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("Navigation timed out.")
                return NavigateToObjectResponse(success=False)
            if self.goal_status == 3:
                rospy.loginfo("Navigation completed successfully.")
                return NavigateToObjectResponse(success=True)
            elif self.goal_status in [4, 5, 6, 7, 8]:
                rospy.logerr(f"Navigation failed with status: {self.goal_status}")
                return NavigateToObjectResponse(success=False)
            rate.sleep()

if __name__ == '__main__':
    try:
        NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
