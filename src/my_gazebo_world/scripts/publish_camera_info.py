#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/kinect/depth/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "kinect_link"
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.K = [525.0, 0, 320.0, 0, 525.0, 240.0, 0, 0, 1]
    camera_info_msg.P = [525.0, 0, 320.0, 0, 0, 525.0, 240.0, 0, 0, 0, 1, 0]

    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
