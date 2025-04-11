// filepath: pcl_processing/src/downsample.cpp

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>

#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

// Declare the publisher as a global variable
ros::Publisher pub;
 
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    // Convert ROS message to PCL point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromROSMsg(*cloud_msg, *cloud);
 
    // Downsample the point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

    voxel_filter.setInputCloud(cloud);

    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust leaf size as needed

    voxel_filter.filter(*cloud_filtered);
 
    // Print the number of points in the downsampled cloud
    ROS_INFO("Downsampled cloud has %zu points.", cloud_filtered->points.size());
    
    // Convert filtered cloud back to ROS message

    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_filtered, output);

    output.header = cloud_msg->header;
 
    // Publish the downsampled cloud


    pub.publish(output);

}
 
int main(int argc, char** argv) {

    ros::init(argc, argv, "downsample_node");

    ros::NodeHandle nh;
    
    // Initialize the publisher in the main function
    pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);

 
    // Subscribe to input point cloud topic

    ros::Subscriber sub = nh.subscribe("input_cloud", 1, cloudCallback);
 
    ros::spin();

    return 0;

}
 