#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

class ClockSyncedOctomapRepublisher:
    def __init__(self):
        rospy.init_node('octomap_center_header_fixer')
        self.pub = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=1)
        self.last_cloud = None

        rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.cloud_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)

        rospy.loginfo("[HeaderFixer] Waiting for data from /octomap_point_cloud_centers and /clock...")
        rospy.spin()

    def cloud_callback(self, msg):
        self.last_cloud = msg

    def clock_callback(self, msg):
        if self.last_cloud is not None:
            repub = self.last_cloud
            repub.header.stamp = rospy.Time.now()
            repub.header.frame_id = "odom"
            self.pub.publish(repub)
            rospy.loginfo_throttle(2.0, "[HeaderFixer] Published synced cloud on /filtered_pointcloud")

if __name__ == '__main__':
    try:
        ClockSyncedOctomapRepublisher()
    except rospy.ROSInterruptException:
        pass
