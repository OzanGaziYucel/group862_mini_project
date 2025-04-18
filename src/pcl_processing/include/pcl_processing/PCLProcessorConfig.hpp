#pragma once
#include <string>
#include <ros/ros.h> // Include the ROS header for NodeHandle

struct PCLProcessorConfig {
    std::string input_topic;
    std::string output_topic;

    bool debug_mode;
    int queue_size;
    bool publish_supervoxel_cloud;
    std::string supervoxel_cloud_topic;
    std::string centroid_marker_topic;

    float voxel_leaf_size;
    float min_depth;
    float max_depth;

    float voxel_resolution;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance;

    float concavity_tolerance_threshold;
    bool use_smoothness_check;
    float smoothness_threshold;
    float k_factor;
    int min_segment_size;
    bool use_supervoxel_refinement;
    int supervoxel_refinement_iterations;
    float small_segment_threshold_percent;
    bool use_single_camera_transform;

    // Planar filtering parameters
    bool filter_planar_segments;
    float planar_distance_threshold;
    float min_planar_inlier_percentage;
    int max_planar_segment_size; // Max size (points) to be considered potentially planar
};

void loadConfig(ros::NodeHandle& nh, PCLProcessorConfig& config);