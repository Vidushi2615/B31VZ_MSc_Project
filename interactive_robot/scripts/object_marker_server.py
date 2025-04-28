#!/usr/bin/env python3

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose2D, PointStamped
from visualization_msgs.msg import Marker
from interactive_robot.msg import ObjectLocalisationAction, ObjectLocalisationGoal
from interactive_robot.srv import GetObjectMarker, GetObjectMarkerResponse

robot_pose = None
tf_buffer = None  
tf_listener = None

def pose2d_callback(msg):
    global robot_pose
    robot_pose = msg

def handle_object_request(req):
    global robot_pose, tf_buffer

    rospy.loginfo(f"[SERVER] Received request for object: {req.object_name}")
    if robot_pose is None:
        rospy.logwarn("[SERVER] Robot pose not yet available.")
        return GetObjectMarkerResponse()

    client = actionlib.SimpleActionClient('/object_localisation', ObjectLocalisationAction)
    rospy.loginfo("[SERVER] Connecting to object localisation server...")
    client.wait_for_server()
    goal = ObjectLocalisationGoal(object_name=req.object_name)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    if result is None or result.position is None:
        rospy.logerr("[SERVER] Localisation failed: No object position returned.")
        return GetObjectMarkerResponse()

    rospy.loginfo(f"[SERVER] Got object position in frame: {result.position.header.frame_id}")

    try:
        tf_buffer.can_transform('odom', result.position.header.frame_id, rospy.Time(0), timeout=rospy.Duration(2.0))
        transform = tf_buffer.lookup_transform('odom', result.position.header.frame_id, rospy.Time(0), rospy.Duration(2.0))
        transformed_point = tf2_geometry_msgs.do_transform_point(result.position, transform)
        rospy.loginfo(f"[SERVER] Transformed point: {transformed_point.point}")
    except Exception as e:
        rospy.logerr(f"[SERVER] TF transform failed: {e}")
        return GetObjectMarkerResponse()

    marker_pub = rospy.Publisher('/object_marker', Marker, queue_size=1, latch=True)

    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.header.stamp = rospy.Time.now()
    marker.ns = "object_localisation_server"
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = transformed_point.point
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.06
    marker.scale.y = 0.06
    marker.scale.z = 0.06
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker_pub.publish(marker)

    return GetObjectMarkerResponse(object_position=transformed_point)

def main():
    global robot_pose, tf_buffer, tf_listener

    rospy.init_node('object_marker_service_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/hsrb/pose2D', Pose2D, pose2d_callback)
    rospy.wait_for_message('/hsrb/pose2D', Pose2D)
    rospy.loginfo("[SERVER] /hsrb/pose2D received. Ready to serve.")

    service = rospy.Service('/get_object_marker', GetObjectMarker, handle_object_request)
    rospy.loginfo("[SERVER] Service /get_object_marker is now available.")
    rospy.spin()

if __name__ == '__main__':
    main()