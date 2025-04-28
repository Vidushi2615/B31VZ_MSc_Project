#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # COCO model

        # Subscribe to head camera image stream
        self.image_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.callback)

        # Publishers
        self.det_pub = rospy.Publisher('/detected_objects_2d', Detection2DArray, queue_size=10)
        self.img_pub = rospy.Publisher('/yolo_detector/annotated_image', Image, queue_size=1)

        rospy.loginfo("YOLO Detector Node started (listening to /hsrb/head_rgbd_sensor/rgb/image_raw)")

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion error: {e}")
            return

        results = self.model.predict(img)[0]
        det_array = Detection2DArray()
        det_array.header = msg.header

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_id = int(box.cls[0])
            score = float(box.conf[0])
            label = self.model.names[class_id]

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"{label} {score:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Build Detection2D message
            bbox = BoundingBox2D()
            bbox.center.x = (x1 + x2) / 2
            bbox.center.y = (y1 + (2 * y2)) / 3 
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1

            det = Detection2D()
            det.bbox = bbox
            det.header.frame_id = label  # Use class name here
            det.source_img.header.frame_id = "head_rgbd_sensor_rgb_frame"


            hypo = ObjectHypothesisWithPose()
            hypo.id = 0  # Unused now
            hypo.score = score
            det.results.append(hypo)

            det_array.detections.append(det)

        self.det_pub.publish(det_array)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            annotated_msg.header = msg.header
            self.img_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish annotated image: {e}")

if __name__ == '__main__':
    try:
        YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
