#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import mediapipe as mp
import numpy as np
import threading
import atexit
import tf.transformations
import math
from cv_bridge import CvBridge
from filterpy.kalman import KalmanFilter
from hsrb_interface import Robot

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from interactive_robot.srv import GetHand3D, GetHand3DRequest
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

PALM_SIZE = 0.15

class HandTrackerService:
    def __init__(self):
        rospy.init_node('human_hand_tracking')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.robot = Robot()
        self.whole_body = self.robot.get('whole_body')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_image = None
        self.latest_caminfo = None
        self.latest_cloud = None
        self.wrist_u = None
        self.wrist_v = None

        self.hand_detector = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)

        rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/camera_info', CameraInfo, self.caminfo_callback)
        rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.cloud_callback)

        self.marker_pub = rospy.Publisher('/tracked_hand_point', PointStamped, queue_size=1)
        self.tracked_image_pub = rospy.Publisher('/hand_tracking/image_annotated', Image, queue_size=1)

        self.service = rospy.Service('/human_hand_tracking', Trigger, self.handle_handover)

    def _init_kalman(self):
        kf = KalmanFilter(dim_x=6, dim_z=3)
        kf.F = np.eye(6)
        kf.H[:, :3] = np.eye(3)
        kf.P *= 0.1
        kf.R *= 0.05
        kf.Q *= 0.01
        return kf

    def caminfo_callback(self, msg):
        with self.lock:
            self.latest_caminfo = msg

    def cloud_callback(self, msg):
        with self.lock:
            self.latest_cloud = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.hand_detector.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            if results.multi_hand_landmarks and results.multi_handedness:
                for i, handedness in enumerate(results.multi_handedness):
                    label = handedness.classification[0].label
                    rospy.loginfo_throttle(1, f"[Visual] Detected hand: {label}")

                    if label == "Right":
                        hand_landmarks = results.multi_hand_landmarks[i]
                        wrist = hand_landmarks.landmark[0]
                        h, w, _ = cv_image.shape
                        self.wrist_u = int(wrist.x * w)
                        self.wrist_v = int(wrist.y * h)

                        # Draw and log
                        cv2.circle(cv_image, (self.wrist_u, self.wrist_v), 5, (0, 255, 0), -1)
                        cv2.putText(cv_image, f"Wrist: ({self.wrist_u}, {self.wrist_v})", (self.wrist_u + 10, self.wrist_v),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        rospy.loginfo_throttle(1, f"[Visual] Right wrist at ({self.wrist_u}, {self.wrist_v})")
                        break  # Stop after finding the right hand
            else:
                rospy.loginfo_throttle(1, "[Visual] No hand detected")

            cv2.imshow("Live Hand Tracking", cv_image)
            cv2.waitKey(1)
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            annotated_msg.header = msg.header
            self.tracked_image_pub.publish(annotated_msg)
            with self.lock:
                self.latest_image = msg
        except Exception as e:
            rospy.logwarn(f"[Visual] Image callback error: {e}")

    def call_tracker_service(self):
        rospy.wait_for_service('/get_hand_position_3d')
        try:
            get_3d = rospy.ServiceProxy('/get_hand_position_3d', GetHand3D)
            req = GetHand3DRequest()
            req.cam_info = self.latest_caminfo
            req.cloud = self.latest_cloud

            detection = Detection2D()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = "head_rgbd_sensor_rgb_frame"
            detection.bbox.center.x = self.wrist_u
            detection.bbox.center.y = self.wrist_v
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = 1.0
            detection.bbox.size_y = 1.0
            detection.results.append(ObjectHypothesisWithPose(id=0, score=1.0))

            detections = Detection2DArray()
            detections.header.stamp = rospy.Time.now()
            detections.header.frame_id = "head_rgbd_sensor_rgb_frame"
            detections.detections.append(detection)

            req.detections = detections
            rospy.loginfo(f"[HandTracker] [Tracker] Wrist pixel coordinates: u={self.wrist_u}, v={self.wrist_v}")
            rospy.loginfo("[HandTracker] [Tracker] Preparing request to /get_hand_position_3d")
            rospy.loginfo(f"[HandTracker] Sending bbox=({self.wrist_u}, {self.wrist_v})")
            res = get_3d(req)
            rospy.loginfo(f"[HandTracker] [Tracker] Received 3D point in camera frame: x={res.point.point.x:.3f}, y={res.point.point.y:.3f}, z={res.point.point.z:.3f}")
            return res.point
        except Exception as e:
            rospy.logerr(f"[HandTracker] Tracker service call failed: {e}")
            return None

    def transform_to_odom(self, point):
        try:
            rospy.loginfo(f"[HandTracker] [TF] Looking up transform from {point.header.frame_id} to 'odom' at time {point.header.stamp.to_sec():.2f}")
            transform = self.tf_buffer.lookup_transform(
                'odom',
                point.header.frame_id,
                point.header.stamp,
                rospy.Duration(1.0)
            )
            transformed = tf2_geometry_msgs.do_transform_point(point, transform)
            rospy.loginfo(f"[HandTracker] [TF] Transformed point: x={transformed.point.x:.3f}, y={transformed.point.y:.3f}, z={transformed.point.z:.3f}")
            return transformed
        except Exception as e:
            rospy.logwarn(f"[HandTracker] TF transform failed: {e}")
            return None


    def move_to_target(self, target_point):
        dx = target_point.point.x
        dy = target_point.point.y
        dz = target_point.point.z

        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            rospy.logerr("[HandTracker] Invalid hand direction vector.")
            return False

        ux, uy = dx / norm, dy / norm

        offset_x = dx + PALM_SIZE * ux
        offset_y = dy - PALM_SIZE * uy
        offset_z = dz

        # Adjust roll so palm aligns with hand direction (top-down approach)
        roll = 3.14 + math.atan2(dx, dy)
        quat = tf.transformations.quaternion_from_euler(roll, -1.57, 0)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = offset_x
        goal.pose.position.y = offset_y
        goal.pose.position.z = offset_z
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        rospy.loginfo(f"[HandTracker] [Move] Target orientation (roll={roll:.2f}): quat={quat}")
        rospy.loginfo(f"[HandTracker] [Move] Target position: x={offset_x:.3f}, y={offset_y:.3f}, z={offset_z:.3f}")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                current_pose = self.robot.get('whole_body').get_end_effector_pose()
                current_x = current_pose.pos.x
                current_y = current_pose.pos.y
                current_z = current_pose.pos.z

                distance = math.sqrt(
                    (current_x - offset_x) ** 2 +
                    (current_y - offset_y) ** 2 +
                    (current_z - offset_z) ** 2
                )
                rospy.loginfo(f"[HandTracker] [Move] Distance to goal: {distance:.3f}")

                if distance < 0.06:
                    rospy.loginfo("[HandTracker] Reached hand offset.")
                    return True

                self.whole_body.move_end_effector_pose(goal, ref_frame_id='odom')

            except Exception as e:
                rospy.logwarn(f"[HandTracker] Motion failed: {e}")

            rate.sleep()

    def handle_handover(self, req):
        rospy.sleep(8.0)
        rospy.loginfo("[HandTracker] Service called")
        rate = rospy.Rate(10)
        success = False
        for _ in range(300):  # 30s
            with self.lock:
                image = self.latest_image
                caminfo = self.latest_caminfo
                cloud = self.latest_cloud

            if image and caminfo and cloud:
                point = self.call_tracker_service()
                if point:
                    odom_point = self.transform_to_odom(point)
                    if odom_point:
                        rospy.loginfo(f"[HandTracker] [Move] Using raw odom-transformed point for movement.")
                        self.marker_pub.publish(odom_point)
                        success = self.move_to_target(odom_point)

                        if success:
                            rospy.signal_shutdown("Handover completed successfully.")
            rate.sleep()
        return TriggerResponse(success=success, message="Reached hand" if success else "Failed to reach")

def cleanup():
    cv2.destroyAllWindows()

atexit.register(cleanup)

if __name__ == '__main__':
    HandTrackerService()
    rospy.spin()
