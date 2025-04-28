#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from tmc_mapping_msgs.msg import CollisionMap
from tmc_geometry_msgs.msg import OrientedBoundingBox
from geometry_msgs.msg import Point, Vector3
import numpy as np

frame_id = "odom"
resolution = 0.05
publisher = None

def pointcloud_callback(msg):
    global publisher

    voxel_map = {}
    for point in pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True):
        voxel = tuple((np.array(point) / resolution).astype(int))
        voxel_map[voxel] = point

    boxes = []
    for voxel, pt in voxel_map.items():
        box = OrientedBoundingBox()
        box.center = Point(*pt)
        box.extents = Vector3(resolution, resolution, resolution)
        box.axis.x = 1.0
        box.axis.y = 0.0
        box.axis.z = 0.0
        box.angle = 0.0
        boxes.append(box)

    cmap = CollisionMap()
    cmap.header.stamp = rospy.Time.now()
    cmap.header.frame_id = frame_id
    cmap.boxes = boxes
    publisher.publish(cmap)

def main():
    global publisher
    rospy.init_node("voxel_collision_map_publisher")
    rospy.Subscriber("/filtered_pointcloud", PointCloud2, pointcloud_callback)
    publisher = rospy.Publisher("/collision_map", CollisionMap, queue_size=1)
    rospy.loginfo("[VoxelMap] Converting /filtered_pointcloud to /collision_map with voxel resolution %.2f", resolution)
    rospy.spin()

if __name__ == '__main__':
    main()
