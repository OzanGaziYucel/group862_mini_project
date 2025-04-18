#include <ros/ros.h>
#include "pcl_processing/PCLProcessor.hpp"
#include "pcl_processing/PCLProcessorConfig.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <iostream>
#include <algorithm>
#include <set>

PCLProcessor::PCLProcessor(ros::NodeHandle& nh, const PCLProcessorConfig& config)
    : config_(config)
{
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.output_topic, 1);
    supervoxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.supervoxel_cloud_topic, 1);
    centroid_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(config_.centroid_marker_topic, 1);
}

std::vector<uint32_t> PCLProcessor::generateColors(size_t count) {
    std::vector<uint32_t> colors(count);
    srand(42);
    for (size_t i = 0; i < count; i++) {
        float hue = static_cast<float>(i) / count * 360.0f;
        float saturation = 0.7f + 0.3f * (rand() % 100) / 100.0f;
        float value = 0.7f + 0.3f * (rand() % 100) / 100.0f;
        int hi = static_cast<int>(hue / 60.0f) % 6;
        float f = hue / 60.0f - hi;
        float p = value * (1.0f - saturation);
        float q = value * (1.0f - f * saturation);
        float t = value * (1.0f - (1.0f - f) * saturation);
        float r, g, b;
        switch (hi) {
            case 0: r = value; g = t; b = p; break;
            case 1: r = q; g = value; b = p; break;
            case 2: r = p; g = value; b = t; break;
            case 3: r = p; g = q; b = value; break;
            case 4: r = t; g = p; b = value; break;
            case 5: r = value; g = p; b = q; break;
            default: r = g = b = 0; break;
        }
        uint8_t rr = static_cast<uint8_t>(r * 255);
        uint8_t gg = static_cast<uint8_t>(g * 255);
        uint8_t bb = static_cast<uint8_t>(b * 255);
        colors[i] = ((uint32_t)rr << 16 | (uint32_t)gg << 8 | (uint32_t)bb);
    }
    return colors;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::colorSegments(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud) {
    std::set<uint32_t> unique_labels;
    for (const auto& point : labeled_cloud->points) {
        unique_labels.insert(point.label);
    }
    std::vector<uint32_t> colors = generateColors(unique_labels.size());
    std::map<uint32_t, uint32_t> label_to_color;
    size_t color_idx = 0;
    for (const auto& label : unique_labels) {
        label_to_color[label] = colors[color_idx++];
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    colored_cloud->width = labeled_cloud->width;
    colored_cloud->height = labeled_cloud->height;
    colored_cloud->is_dense = labeled_cloud->is_dense;
    colored_cloud->points.resize(labeled_cloud->points.size());
    for (size_t i = 0; i < labeled_cloud->points.size(); i++) {
        const auto& labeled_point = labeled_cloud->points[i];
        auto& colored_point = colored_cloud->points[i];
        colored_point.x = labeled_point.x;
        colored_point.y = labeled_point.y;
        colored_point.z = labeled_point.z;
        uint32_t rgb_color = label_to_color[labeled_point.label];
        colored_point.r = (rgb_color >> 16) & 0xFF;
        colored_point.g = (rgb_color >> 8) & 0xFF;
        colored_point.b = rgb_color & 0xFF;
    }
    return colored_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::downsample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
    voxel_filter.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::filterDepth(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(config_.min_depth, config_.max_depth);
    pass.filter(*filtered_cloud);
    ROS_INFO("Depth filter: removed %zu points with depth less than %.2f meters",
             input_cloud->points.size() - filtered_cloud->points.size(), config_.min_depth);
    return filtered_cloud;
}

std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> PCLProcessor::createSupervoxelClusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    pcl::SupervoxelClustering<pcl::PointXYZRGB>& super) {
    
    super.setInputCloud(input_cloud);
    super.setColorImportance(config_.color_importance);
    super.setSpatialImportance(config_.spatial_importance);
    super.setNormalImportance(config_.normal_importance);
    super.setUseSingleCameraTransform(config_.use_single_camera_transform);

    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    ROS_INFO("Extracted %lu supervoxels", supervoxel_clusters.size());

    if (config_.use_supervoxel_refinement) {
        super.refineSupervoxels(config_.supervoxel_refinement_iterations, supervoxel_clusters);
    }
    
    return supervoxel_clusters;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::performLCCPSegmentation(
    const std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
    const std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& sv_labeled_cloud) {
    
    pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
    lccp.setConcavityToleranceThreshold(config_.concavity_tolerance_threshold);
    lccp.setSanityCheck(true);
    if (config_.use_smoothness_check) {
        lccp.setSmoothnessCheck(true, config_.voxel_resolution, config_.seed_resolution, config_.smoothness_threshold);
    }
    lccp.setKFactor(config_.k_factor);
    lccp.setMinSegmentSize(config_.min_segment_size);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.segment();

    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    
    return lccp_labeled_cloud;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::filterSmallSegments(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud) {
    
    std::map<uint32_t, size_t> segment_sizes;
    for (const auto& point : labeled_cloud->points) {
        segment_sizes[point.label]++;
    }
    
    std::vector<std::pair<uint32_t, size_t>> sorted_segments(segment_sizes.begin(), segment_sizes.end());
    std::sort(sorted_segments.begin(), sorted_segments.end(),
              [](const std::pair<uint32_t, size_t>& a, const std::pair<uint32_t, size_t>& b) {
                  return a.second > b.second;
              });
              
    std::set<uint32_t> segments_to_keep;
    size_t points_threshold = static_cast<size_t>(labeled_cloud->points.size() *
                                                  config_.small_segment_threshold_percent / 100.0);
                                                  
    for (const auto& segment : sorted_segments) {
        if (segment.second >= points_threshold) {
            segments_to_keep.insert(segment.first);
        }
    }
    
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    for (const auto& point : labeled_cloud->points) {
        if (segments_to_keep.find(point.label) != segments_to_keep.end()) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::filterPlanarSegments(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud) {

    if (!config_.filter_planar_segments) {
        ROS_INFO("Planar segment filtering disabled.");
        return labeled_cloud;
    }

    ROS_INFO("Starting planar segment filtering...");

    // Group points by segment label
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> segments;
    for (const auto& point : labeled_cloud->points) {
        uint32_t label = point.label;
        if (segments.find(label) == segments.end()) {
            segments[label] = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
            segments[label]->header = labeled_cloud->header; // Copy header info
        }
        segments[label]->points.push_back(point);
    }

    std::set<uint32_t> planar_labels_to_remove;
    pcl::SACSegmentation<pcl::PointXYZL> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config_.planar_distance_threshold);
    seg.setMaxIterations(100); // Max iterations for RANSAC

    size_t total_points_before = labeled_cloud->points.size();
    size_t planar_segments_found = 0;

    for (const auto& segment_pair : segments) {
        uint32_t label = segment_pair.first;
        const auto& segment_cloud = segment_pair.second;

        // Only check segments larger than the configured size threshold
        if (segment_cloud->points.size() > config_.max_planar_segment_size) {
            if (segment_cloud->points.size() < 3) { // Need at least 3 points for plane fitting
               ROS_WARN("Segment %u too small for plane fitting (%zu points), skipping.", label, segment_cloud->points.size());
               continue;
            }

            seg.setInputCloud(segment_cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                ROS_WARN("Could not fit plane to segment %u with %zu points.", label, segment_cloud->points.size());
                continue;
            }

            double inlier_percentage = static_cast<double>(inliers->indices.size()) / segment_cloud->points.size();

            if (inlier_percentage >= config_.min_planar_inlier_percentage) {
                planar_labels_to_remove.insert(label);
                planar_segments_found++;
                ROS_INFO("Segment %u identified as planar (size: %zu, inliers: %.2f%%). Marked for removal.",
                         label, segment_cloud->points.size(), inlier_percentage * 100.0);
            }
        }
    }

    // Create the filtered cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    filtered_cloud->header = labeled_cloud->header;

    for (const auto& point : labeled_cloud->points) {
        if (planar_labels_to_remove.find(point.label) == planar_labels_to_remove.end()) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true; // Assuming filtering doesn't introduce NaN/inf

    size_t total_points_after = filtered_cloud->points.size();
    ROS_INFO("Planar filtering removed %zu planar segments and %zu points.",
             planar_segments_found, total_points_before - total_points_after);

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::segmentLCCP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    
    // Step 1: Create supervoxel clustering
    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(config_.voxel_resolution, config_.seed_resolution);
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters = 
        createSupervoxelClusters(input_cloud, super);
    
    // Step 2: Get adjacency information
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    
    // Step 3: Get labeled cloud for visualization
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    sv_labeled_cloud->header = input_cloud->header;
    
    // Step 4: Publish supervoxel cloud if requested
    if(config_.publish_supervoxel_cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sv_colored_cloud = colorSegments(sv_labeled_cloud);
        sensor_msgs::PointCloud2 sv_msg;
        pcl::toROSMsg(*sv_colored_cloud, sv_msg);
        sv_msg.header.frame_id = input_cloud->header.frame_id;
        supervoxel_pub_.publish(sv_msg);
    }
    
    // Step 5: Perform LCCP segmentation
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = 
        performLCCPSegmentation(supervoxel_clusters, supervoxel_adjacency, sv_labeled_cloud);
    
    // Step 6: Filter small segments
    auto filtered_cloud = filterSmallSegments(lccp_labeled_cloud);
    filtered_cloud->header = input_cloud->header;
    return filtered_cloud;
}

std::map<uint32_t, Eigen::Vector4f> PCLProcessor::computeCentroids(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& segmented_cloud) {
    
    // Group points by segment label
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> segments;
    
    for (const auto& point : segmented_cloud->points) {
        uint32_t label = point.label;
        
        if (segments.find(label) == segments.end()) {
            segments[label] = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
        }
        
        segments[label]->points.push_back(point);
    }
    
    // Compute centroid for each segment
    std::map<uint32_t, Eigen::Vector4f> centroids;
    
    for (const auto& segment_pair : segments) {
        uint32_t label = segment_pair.first;
        const auto& segment_cloud = segment_pair.second;
        
        // Use PCL's built-in centroid computation
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*segment_cloud, centroid);
        centroids[label] = centroid;
        
        ROS_INFO("Segment %u: centroid at (%.3f, %.3f, %.3f) with %zu points", 
                 label, centroid[0], centroid[1], centroid[2], segment_cloud->points.size());
    }

    // Visualize centroids with marker ROS messages
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& centroid_pair : centroids) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = segmented_cloud->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "segment_centroids";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Position
        marker.pose.position.x = centroid_pair.second[0];
        marker.pose.position.y = centroid_pair.second[1];
        marker.pose.position.z = centroid_pair.second[2];
        
        // Size (diameter in meters)
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        // Color (RGBA)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.lifetime = ros::Duration(); // Forever
        
        marker_array.markers.push_back(marker);
    }

    centroid_marker_pub_.publish(marker_array);
    
    return centroids;
}

void PCLProcessor::processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ros::Time start_time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ROS_INFO("Input cloud has %zu points", cloud->size());

    auto cloud_filtered = downsample(cloud);
    cloud_filtered = filterDepth(cloud_filtered);

    auto segmented_cloud = segmentLCCP(cloud_filtered);
    auto segmented_cloud_filtered = filterPlanarSegments(segmented_cloud);
    auto segment_centroids = computeCentroids(segmented_cloud_filtered);
    auto colored_cloud = colorSegments(segmented_cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = cloud_msg->header;
    pub_.publish(output);

    ros::Duration duration = ros::Time::now() - start_time;
    ROS_INFO("Processing time: %.2f milliseconds", duration.toSec() * 1000.0);

    if(config_.debug_mode) {
        std::cout << "Press Enter to process the next message..." << std::endl;
        std::cin.get();
    }
}