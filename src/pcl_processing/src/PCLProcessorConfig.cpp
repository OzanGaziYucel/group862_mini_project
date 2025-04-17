#include "pcl_processing/PCLProcessorConfig.hpp"
#include <ros/ros.h>

void loadConfig(ros::NodeHandle& nh, PCLProcessorConfig& config) {
    nh.param<std::string>("input_topic", config.input_topic, "/input_cloud");
    nh.param<std::string>("output_topic", config.output_topic, "/segmented_cloud");
    nh.param<bool>("debug_mode", config.debug_mode, false);
    nh.param<int>("queue_size", config.queue_size, 1);
    nh.param<bool>("publish_supervoxel_cloud", config.publish_supervoxel_cloud, false);
    nh.param<std::string>("supervoxel_cloud_topic", config.supervoxel_cloud_topic, "/supervoxel_cloud");
    nh.param<std::string>("centroid_marker_topic", config.centroid_marker_topic, "/centroid_markers");
    nh.param<float>("voxel_leaf_size", config.voxel_leaf_size, 0.01f);
    nh.param<float>("min_depth", config.min_depth, 1.1f);
    nh.param<float>("max_depth", config.max_depth, 5.0f);
    nh.param<float>("voxel_resolution", config.voxel_resolution, 0.03f);
    nh.param<float>("seed_resolution", config.seed_resolution, 0.15f);
    nh.param<float>("color_importance", config.color_importance, 0.1f);
    nh.param<float>("spatial_importance", config.spatial_importance, 1.0f);
    nh.param<float>("normal_importance", config.normal_importance, 5.0f);
    nh.param<float>("concavity_tolerance_threshold", config.concavity_tolerance_threshold, 15.0f);
    nh.param<bool>("use_smoothness_check", config.use_smoothness_check, true);
    nh.param<float>("smoothness_threshold", config.smoothness_threshold, 0.15f);
    nh.param<float>("k_factor", config.k_factor, 0.5f);
    nh.param<int>("min_segment_size", config.min_segment_size, 500);
    nh.param<bool>("use_supervoxel_refinement", config.use_supervoxel_refinement, true);
    nh.param<float>("small_segment_threshold_percent", config.small_segment_threshold_percent, 1.0f);
    nh.param<bool>("use_single_camera_transform", config.use_single_camera_transform, false);
    nh.param<int>("supervoxel_refinement_iterations", config.supervoxel_refinement_iterations, 2);
}