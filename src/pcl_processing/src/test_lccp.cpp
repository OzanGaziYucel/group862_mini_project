#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_lccp");
    ros::NodeHandle nh;

    // Publisher for the input_cloud topic
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);

    // Load the PLY file
    if (argc < 2) {
        ROS_ERROR("Please provide the path to a .ply file as an argument.");
        return -1;
    }

    std::string ply_file = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_file, *cloud) == -1) {
        ROS_ERROR("Failed to load PLY file: %s", ply_file.c_str());
        return -1;
    }

    ROS_INFO("Loaded PLY file with %zu points.", cloud->points.size());

    // Convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    // Publish the point cloud once
    ros::Duration(1.0).sleep(); // Give the publisher some time to set up
    output.header.stamp = ros::Time::now();
    pub.publish(output);

    ROS_INFO("Published point cloud to topic 'input_cloud'.");

    return 0;
}

