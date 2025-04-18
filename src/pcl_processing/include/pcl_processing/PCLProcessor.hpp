#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <map>
#include <vector>

#include "PCLProcessorConfig.hpp"

class PCLProcessor {
public:
    PCLProcessor(ros::NodeHandle& nh, const PCLProcessorConfig& config);

    void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    PCLProcessorConfig config_;
    ros::Publisher pub_;
    ros::Publisher supervoxel_pub_;
    ros::Publisher centroid_marker_pub_;

    // Processing methods
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterDepth(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    pcl::PointCloud<pcl::PointXYZL>::Ptr segmentLCCP(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> createSupervoxelClusters(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
        pcl::SupervoxelClustering<pcl::PointXYZRGB>& super);
        
    pcl::PointCloud<pcl::PointXYZL>::Ptr performLCCPSegmentation(
        const std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
        const std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& sv_labeled_cloud);
        
    pcl::PointCloud<pcl::PointXYZL>::Ptr filterSmallSegments(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud);

    pcl::PointCloud<pcl::PointXYZL>::Ptr filterPlanarSegments(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud);
        
    std::map<uint32_t, Eigen::Vector4f> computeCentroids(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSegments(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr&);


    std::vector<uint32_t> generateColors(size_t count);
};