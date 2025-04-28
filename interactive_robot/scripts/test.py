#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from interactive_robot.srv import GetObjectMarker
from hsrb_interface import Robot
from hsrb_interface.collision_world import CollisionWorld
from hsrb_interface.exceptions import MotionPlanningError

PALM_SIZE = 0.02
XY_OFFSET = 0.1
Z_OFFSET = 0.4

def publish_marker(pose_stamped):
    marker_pub = rospy.Publisher('/manip_target_marker', Marker, queue_size=1, latch=True)
    marker = Marker()
    marker.header = pose_stamped.header
    marker.ns = "manip_target"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose_stamped.pose
    marker.scale.x = 0.2
    marker.scale.y = 0.04
    marker.scale.z = 0.04
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.8
    marker.color.a = 1.0
    marker_pub.publish(marker)

def main():
    rospy.init_node('test2_manip_with_collision')

    robot = Robot()
    whole_body = robot.get('whole_body')
    gripper = robot.get('gripper')
    collision_world = CollisionWorld('global_collision_world')

    object_name = rospy.get_param("~object_name", "bottle")
    rospy.loginfo(f"\033[92m[Test] Requesting object: {object_name}\033[0m")

    rospy.wait_for_service('/get_object_marker')
    try:
        get_marker = rospy.ServiceProxy('/get_object_marker', GetObjectMarker)
        response = get_marker(object_name)
    except rospy.ServiceException as e:
        rospy.logerr(f"[Test] Service call failed: {e}")
        return

    target_point = response.object_position
    if not target_point:
        rospy.logerr("[Test] No object position received.")
        return

    rospy.loginfo("[Test] Executing with collision avoidance enabled...")

    dx = target_point.point.x 
    dy = target_point.point.y
    dz = target_point.point.z
    norm = math.hypot(dx, dy)
    if norm < 1e-3:
        rospy.logerr("[Test] Cannot compute orientation: invalid approach direction.")
        return

    ux, uy = dx / norm, dy / norm

    offset_x = dx - PALM_SIZE * ux
    offset_y = dy - PALM_SIZE * uy 
    offset_z = dz

    roll = math.atan2(dx, dy)
    quat = tf.transformations.quaternion_from_euler(roll, -1.57, 0)

    marker_pose = PoseStamped()
    marker_pose.header.frame_id = "odom"
    marker_pose.header.stamp = rospy.Time.now()
    marker_pose.pose.position.x = offset_x
    marker_pose.pose.position.y = offset_y
    marker_pose.pose.position.z = offset_z
    marker_pose.pose.orientation.x = quat[0]
    marker_pose.pose.orientation.y = quat[1]
    marker_pose.pose.orientation.z = quat[2]
    marker_pose.pose.orientation.w = quat[3]
    publish_marker(marker_pose)

    try:
        # Step 1: Open gripper
        rospy.loginfo("[Test] Opening gripper...")
        gripper.command(1.2)

        whole_body.end_effector_frame = 'hand_palm_link'

        rospy.loginfo(f"\033[94m[Test] Attempting move_end_effector_pose to {marker_pose.pose.position}\033[0m")

        # Step 2: Plan and execute
        whole_body.move_end_effector_pose(marker_pose, ref_frame_id='odom')

        rospy.loginfo(f"\033[92m[Test] Motion execution complete.\033[0m")

        # Step 3: Close gripper
        rospy.loginfo("[Test] Closing gripper...")
        gripper.apply_force(1.0)

        whole_body.move_end_effector_by_line((1.0, 4.0, -4.0), Z_OFFSET)

        whole_body.move_to_go()

    except MotionPlanningError as mpe:
        rospy.logerr(f"[Error] MotionPlanningError: {str(mpe)}")
    except Exception as e:
        rospy.logerr(f"[Exception] General failure: {e}")

if __name__ == '__main__':
    main()
