#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def point_cloud_callback(msg):
    # rospy.loginfo("Received Point Cloud data") --> if needed for debugging

    # Convert ROS PointCloud2 message to a list of points
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    if not cloud_points:
        rospy.logwarn("No valid points received in the point cloud!")
        return

    # rospy.loginfo(f"Number of points in cloud: {len(cloud_points)}") --> if needed for debugging

    # TODO: Implement filtering & segmentation here


def point_cloud_segmentation():
    rospy.init_node('point_cloud_segmentation', anonymous=True)
    
    # Subscribe to Kinect's point cloud topic
    rospy.Subscriber("/kinect/depth/points", PointCloud2, point_cloud_callback)
    
    rospy.loginfo("Point cloud segmentation node started.")
    rospy.spin()

if __name__ == '__main__':
    try:
        point_cloud_segmentation()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")


