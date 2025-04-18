#include "stdafx.h"
#include "pcl_processing/PCLProcessor.hpp"
#include <pcl/common/pca.h>
#include <pcl/common/angles.h> // <<< Add this for deg2rad
#include <pcl/filters/extract_indices.h> // <<< Add this for ExtractIndices

PCLProcessor::PCLProcessor(ros::NodeHandle& nh, const PCLProcessorConfig& config)
    : config_(config)
{
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.output_topic, 1);
    supervoxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.supervoxel_cloud_topic, 1);
    centroid_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(config_.centroid_marker_topic, 1);
    primitive_marker_pub_ = nh.advertise<visualization_msgs::Marker>(config_.primitive_marker_topic, 1);
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

bool PCLProcessor::computeCentroids(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& segmented_cloud,
    uint32_t& selected_label_out) {
    
    // Group points by segment label
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> segments;
    for (const auto& point : segmented_cloud->points) {
        uint32_t label = point.label;
        if (segments.find(label) == segments.end()) {
            segments[label] = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
            segments[label]->header = segmented_cloud->header; // Copy header
        }
        segments[label]->points.push_back(point);
    }

    // Clear previous markers
    visualization_msgs::MarkerArray delete_markers;
    visualization_msgs::Marker marker_delete;
    marker_delete.header.frame_id = segmented_cloud->header.frame_id; // Use the same frame_id
    marker_delete.header.stamp = ros::Time::now();
    marker_delete.ns = "segment_centroids"; // Must match the namespace of markers to delete
    marker_delete.id = 0; // ID doesn't matter for DELETEALL
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    delete_markers.markers.push_back(marker_delete);
    centroid_marker_pub_.publish(delete_markers);
    
    // Compute centroid for each segment
    std::map<uint32_t, Eigen::Vector4f> centroids;
    
    for (const auto& segment_pair : segments) {
        uint32_t label = segment_pair.first;
        const auto& segment_cloud = segment_pair.second;
        if (segment_cloud->points.empty()) continue; // Should not happen, but safety check

        // Use PCL's built-in centroid computation
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*segment_cloud, centroid);
        centroids[label] = centroid;
        
        ROS_INFO("Segment %u: centroid at (%.3f, %.3f, %.3f) with %zu points", 
                 label, centroid[0], centroid[1], centroid[2], segment_cloud->points.size());
    }

    // Find the centroid closest to the origin in the XY plane
    bool label_selected = false;
    double min_dist_sq = std::numeric_limits<double>::max();
    Eigen::Vector4f selected_centroid; // To store the selected centroid coords

    for (const auto& centroid_pair : centroids) {
        uint32_t label = centroid_pair.first;
        const Eigen::Vector4f& centroid = centroid_pair.second;
        double dist_sq = centroid[0] * centroid[0] + centroid[1] * centroid[1]; // XY distance squared

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            selected_label_out = label;
            selected_centroid = centroid;
            label_selected = true;
        }
    }

    if (label_selected) {
        ROS_INFO("Selected segment: %u (XY distance: %.3f)", selected_label_out, std::sqrt(min_dist_sq));
    } else {
         ROS_WARN("Could not select a closest segment (centroids map was empty or logic error?).");
         // This case should ideally be caught by the initial segments.empty() check
    }

    // Visualize centroids with marker ROS messages
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& centroid_pair : centroids) {
        uint32_t label = centroid_pair.first;
        const Eigen::Vector4f& centroid = centroid_pair.second;

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
        
        // Color (RGBA) - Red by default, Green if selected
        if (label_selected && label == selected_label_out) {
            marker.color.r = 0.0; // Green for selected
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 1.0; // Red for others
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(); // Forever

        marker_array.markers.push_back(marker);
    }

    centroid_marker_pub_.publish(marker_array);
    
    return label_selected;
}

void PCLProcessor::fitAndPublishPrimitive(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& segment_cloud,
    const std_msgs::Header& header) {

    if (!segment_cloud || segment_cloud->points.empty()) {
        ROS_WARN("Cannot fit primitive to empty segment cloud.");
        return;
    }

    ROS_INFO("Attempting to fit primitives to segment with %zu points.", segment_cloud->points.size());

    // --- Clear previous marker ---
    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "fitted_primitive";
    clear_marker.id = 0; // Use a consistent ID for the primitive marker
    clear_marker.action = visualization_msgs::Marker::DELETE; // Delete the single marker
    primitive_marker_pub_.publish(clear_marker);

    // --- Common variables ---
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Use the base class for sphere fitting
    pcl::SACSegmentation<pcl::PointXYZL> seg_base;
    seg_base.setOptimizeCoefficients(true);
    seg_base.setMethodType(pcl::SAC_RANSAC);
    seg_base.setMaxIterations(1000);
    seg_base.setDistanceThreshold(config_.primitive_distance_threshold);
    seg_base.setRadiusLimits(0.0, config_.sphere_max_radius); // Min radius is 0

    // Use the derived class for cylinder fitting
    pcl::SACSegmentationFromNormals<pcl::PointXYZL, pcl::Normal> seg_normals;
    seg_normals.setOptimizeCoefficients(true);
    seg_normals.setMethodType(pcl::SAC_RANSAC);
    seg_normals.setMaxIterations(1000);
    seg_normals.setDistanceThreshold(config_.primitive_distance_threshold);
    // Set normal-specific parameters here
    seg_normals.setNormalDistanceWeight(config_.cylinder_normal_distance_weight);
    seg_normals.setRadiusLimits(config_.cylinder_min_radius, config_.cylinder_max_radius);

    double best_inlier_percentage = 0.0;
    int best_primitive_type = -1; // -1: None, 0: Sphere, 1: Cylinder, 2: Box
    visualization_msgs::Marker best_marker; // Store the best marker found

    // --- 1. Sphere Fitting ---
    if (segment_cloud->points.size() >= 4) { // Need at least 4 points for sphere RANSAC
        seg_base.setModelType(pcl::SACMODEL_SPHERE);
        seg_base.setInputCloud(segment_cloud);
        seg_base.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            double inlier_percentage = static_cast<double>(inliers->indices.size()) / segment_cloud->points.size();
            ROS_INFO("Sphere fit: %.2f%% inliers (Coeffs: center=%.3f,%.3f,%.3f radius=%.3f)",
                     inlier_percentage * 100.0,
                     coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

            if (inlier_percentage >= config_.min_primitive_inlier_percentage && inlier_percentage > best_inlier_percentage) {
                best_inlier_percentage = inlier_percentage;
                best_primitive_type = 0;

                best_marker.header = header;
                best_marker.ns = "fitted_primitive";
                best_marker.id = 0;
                best_marker.type = visualization_msgs::Marker::SPHERE;
                best_marker.action = visualization_msgs::Marker::ADD;
                best_marker.pose.position.x = coefficients->values[0];
                best_marker.pose.position.y = coefficients->values[1];
                best_marker.pose.position.z = coefficients->values[2];
                best_marker.pose.orientation.w = 1.0; // Identity orientation
                best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = coefficients->values[3] * 2.0; // Diameter
                best_marker.color.r = 0.0; best_marker.color.g = 1.0; best_marker.color.b = 1.0; best_marker.color.a = 0.5; // Cyan
                best_marker.lifetime = ros::Duration();
            }
        } else {
            ROS_INFO("Sphere fit: Failed (no inliers or radius limit exceeded).");
        }
    }

    // --- 2. Cylinder Fitting ---
    // Requires normals
    if (segment_cloud->points.size() >= 3) {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZL>());
        pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> ne;
        ne.setSearchMethod(tree);
        ne.setInputCloud(segment_cloud);
        ne.setKSearch(20);
        ne.compute(*normals);

        if (normals->points.size() == segment_cloud->points.size()) {
            seg_normals.setModelType(pcl::SACMODEL_CYLINDER); // Set model type on seg_normals
            seg_normals.setInputCloud(segment_cloud);
            seg_normals.setInputNormals(normals); // Use setInputNormals
            // NormalDistanceWeight and RadiusLimits are already set above
            seg_normals.segment(*inliers, *coefficients); // Perform segmentation

            if (!inliers->indices.empty() && coefficients->values.size() == 7) {
                // ... (cylinder marker creation logic - remains the same) ...
                 double inlier_percentage = static_cast<double>(inliers->indices.size()) / segment_cloud->points.size();
                ROS_INFO("Cylinder fit: %.2f%% inliers (Coeffs: pt=%.3f,%.3f,%.3f dir=%.3f,%.3f,%.3f radius=%.3f)",
                         inlier_percentage * 100.0,
                         coefficients->values[0], coefficients->values[1], coefficients->values[2], // Point on axis
                         coefficients->values[3], coefficients->values[4], coefficients->values[5], // Axis direction
                         coefficients->values[6]); // Radius

                if (inlier_percentage >= config_.min_primitive_inlier_percentage && inlier_percentage > best_inlier_percentage) {
                    best_inlier_percentage = inlier_percentage;
                    best_primitive_type = 1;

                    Eigen::Vector3d axis_dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
                    axis_dir.normalize();
                    Eigen::Vector3d point_on_axis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

                    double min_proj = std::numeric_limits<double>::max();
                    double max_proj = std::numeric_limits<double>::lowest();
                    for (int index : inliers->indices) {
                        Eigen::Vector3d point(segment_cloud->points[index].x, segment_cloud->points[index].y, segment_cloud->points[index].z);
                        double proj = (point - point_on_axis).dot(axis_dir);
                        min_proj = std::min(min_proj, proj);
                        max_proj = std::max(max_proj, proj);
                    }
                    double height = max_proj - min_proj;
                    Eigen::Vector3d center = point_on_axis + axis_dir * (min_proj + height / 2.0);
                    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis_dir);

                    best_marker.header = header;
                    best_marker.ns = "fitted_primitive";
                    best_marker.id = 0;
                    best_marker.type = visualization_msgs::Marker::CYLINDER;
                    best_marker.action = visualization_msgs::Marker::ADD;
                    best_marker.pose = tf2::toMsg(Eigen::Affine3d(q * Eigen::Translation3d(center)));
                    best_marker.scale.x = best_marker.scale.y = coefficients->values[6] * 2.0;
                    best_marker.scale.z = height > 0 ? height : 0.01;
                    best_marker.color.r = 1.0; best_marker.color.g = 1.0; best_marker.color.b = 0.0; best_marker.color.a = 0.5; // Yellow
                    best_marker.lifetime = ros::Duration();
                }
            } else {
                ROS_INFO("Cylinder fit: Failed (no inliers or invalid coefficients).");
            }
        } else {
             ROS_WARN("Cylinder fit: Failed to compute normals.");
        }
    }

    // --- 3. Box Fitting (Plane-based Method from Paper) ---
    if (segment_cloud->points.size() >= 3) { // Need points for plane fitting

        pcl::PointCloud<pcl::PointXYZL>::Ptr remaining_cloud = segment_cloud; // Start with the full segment
        pcl::PointIndices::Ptr inliers_plane1(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane1(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_plane1(new pcl::PointCloud<pcl::PointXYZL>);

        pcl::PointIndices::Ptr inliers_plane2(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane2(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_plane2(new pcl::PointCloud<pcl::PointXYZL>);

        pcl::ExtractIndices<pcl::PointXYZL> extract;
        pcl::SACSegmentation<pcl::PointXYZL> seg_plane;
        seg_plane.setOptimizeCoefficients(true);
        seg_plane.setModelType(pcl::SACMODEL_PLANE);
        seg_plane.setMethodType(pcl::SAC_RANSAC);
        seg_plane.setDistanceThreshold(config_.primitive_distance_threshold); // Use same threshold?
        seg_plane.setMaxIterations(1000);

        bool plane1_found = false;
        bool plane2_found = false;
        size_t total_inliers = 0;

        // --- Fit First Plane ---
        seg_plane.setInputCloud(remaining_cloud);
        seg_plane.segment(*inliers_plane1, *coefficients_plane1);

        if (!inliers_plane1->indices.empty()) {
            plane1_found = true;
            total_inliers += inliers_plane1->indices.size();

            // Extract inliers for plane 1
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers_plane1);
            extract.setNegative(false);
            extract.filter(*cloud_plane1);

            // Extract outliers (remaining points) for potential second plane fit
            extract.setNegative(true);
            extract.filter(*remaining_cloud); // Update remaining_cloud

            ROS_INFO("Box fit: Found first plane with %zu inliers.", inliers_plane1->indices.size());

            // --- Fit Second Perpendicular Plane ---
            if (remaining_cloud->points.size() >= 3) {
                seg_plane.setInputCloud(remaining_cloud);
                // Set constraints for perpendicular plane
                seg_plane.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                Eigen::Vector3f axis(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2]);
                seg_plane.setAxis(axis);
                seg_plane.setEpsAngle(pcl::deg2rad(15.0)); // Angle tolerance (e.g., 15 degrees) - make configurable?

                seg_plane.segment(*inliers_plane2, *coefficients_plane2);

                if (!inliers_plane2->indices.empty()) {
                    plane2_found = true;
                    total_inliers += inliers_plane2->indices.size();

                    // Extract inliers for plane 2
                    extract.setInputCloud(remaining_cloud); // Use the cloud *after* first plane removal
                    extract.setIndices(inliers_plane2);
                    extract.setNegative(false);
                    extract.filter(*cloud_plane2);
                    ROS_INFO("Box fit: Found second perpendicular plane with %zu inliers.", inliers_plane2->indices.size());
                } else {
                    ROS_INFO("Box fit: Could not find a second perpendicular plane.");
                }
            } else {
                 ROS_INFO("Box fit: Not enough remaining points (%zu) to search for second plane.", remaining_cloud->points.size());
            }
        } else {
             ROS_INFO("Box fit: Could not find the first plane.");
        }

        // --- Check Total Inlier Percentage ---
        double box_inlier_percentage = static_cast<double>(total_inliers) / segment_cloud->points.size();
        ROS_INFO("Box fit (plane method): Total inliers = %zu (%.2f%%)", total_inliers, box_inlier_percentage * 100.0);

        // Proceed only if plane 1 was found and total inliers meet threshold
        if (plane1_found && box_inlier_percentage >= config_.min_primitive_inlier_percentage) {

            // --- PCA on First Plane ---
            pcl::PCA<pcl::PointXYZL> pca;
            pca.setInputCloud(cloud_plane1);
            Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
            Eigen::Vector3f axis1 = eigenvectors.col(0); // First principal component
            Eigen::Vector3f axis2 = eigenvectors.col(1); // Second principal component
            Eigen::Vector3f axis3; // Third axis

            // Ensure axes form a right-handed coordinate system
            // Check if plane normal is roughly aligned with PCA's 3rd eigenvector
            Eigen::Vector3f plane1_normal(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2]);
            plane1_normal.normalize();
            if (axis1.cross(axis2).dot(plane1_normal) < 0) {
                // Flip axis2 if necessary to align cross product with normal (or PCA's 3rd axis)
                axis2 = -axis2;
            }
            axis3 = axis1.cross(axis2); // Third axis from cross product

            // --- Calculate Width and Height ---
            float min_proj1 = std::numeric_limits<float>::max();
            float max_proj1 = std::numeric_limits<float>::lowest();
            float min_proj2 = std::numeric_limits<float>::max();
            float max_proj2 = std::numeric_limits<float>::lowest();
            Eigen::Vector4f pca_mean_4d = pca.getMean();
            Eigen::Vector3f pca_mean(pca_mean_4d[0], pca_mean_4d[1], pca_mean_4d[2]);

            for (const auto& pt : cloud_plane1->points) {
                Eigen::Vector3f centered_pt = pt.getVector3fMap() - pca_mean;
                float proj1 = centered_pt.dot(axis1);
                float proj2 = centered_pt.dot(axis2);
                min_proj1 = std::min(min_proj1, proj1);
                max_proj1 = std::max(max_proj1, proj1);
                min_proj2 = std::min(min_proj2, proj2);
                max_proj2 = std::max(max_proj2, proj2);
            }
            float width = max_proj1 - min_proj1;
            float height = max_proj2 - min_proj2;
            Eigen::Vector3f center_plane = pca_mean + axis1 * (min_proj1 + width / 2.0f) + axis2 * (min_proj2 + height / 2.0f);

            // --- Calculate Depth ---
            float depth = 0.005f; // Default "thin" object depth
            if (plane2_found) {
                float min_proj3 = std::numeric_limits<float>::max();
                float max_proj3 = std::numeric_limits<float>::lowest();
                for (const auto& pt : cloud_plane2->points) {
                    // Project onto the third axis relative to the first plane's center
                    float proj3 = (pt.getVector3fMap() - center_plane).dot(axis3);
                    min_proj3 = std::min(min_proj3, proj3);
                    max_proj3 = std::max(max_proj3, proj3);
                }
                // Check if projection range is reasonable
                if (max_proj3 > min_proj3) {
                     depth = max_proj3 - min_proj3;
                } else {
                    ROS_WARN("Box fit: Second plane projection resulted in non-positive depth. Using default.");
                }
            }

            // --- Calculate Final Center and Orientation ---
            Eigen::Vector3f geometric_center_world = center_plane + axis3 * (depth / 2.0f); // Adjust center based on depth
            Eigen::Matrix3f rotation_matrix;
            rotation_matrix.col(0) = axis1;
            rotation_matrix.col(1) = axis2;
            rotation_matrix.col(2) = axis3;
            Eigen::Quaternionf obb_quat_f(rotation_matrix);
            obb_quat_f.normalize();

            ROS_INFO("Box fit (plane method): Center: %.3f,%.3f,%.3f Dims: %.3f,%.3f,%.3f",
                     geometric_center_world[0], geometric_center_world[1], geometric_center_world[2],
                     width, height, depth);

            // --- Compare with other primitives ---
            if (box_inlier_percentage > best_inlier_percentage) { // Note: Using > instead of >= config... because we checked that earlier
                best_inlier_percentage = box_inlier_percentage;
                best_primitive_type = 2;

                best_marker.header = header;
                best_marker.ns = "fitted_primitive";
                best_marker.id = 0;
                best_marker.type = visualization_msgs::Marker::CUBE;
                best_marker.action = visualization_msgs::Marker::ADD;
                best_marker.pose.position.x = geometric_center_world[0];
                best_marker.pose.position.y = geometric_center_world[1];
                best_marker.pose.position.z = geometric_center_world[2];
                best_marker.pose.orientation = tf2::toMsg(obb_quat_f.cast<double>());
                best_marker.scale.x = width > 0 ? width : 0.01; // Ensure positive scale
                best_marker.scale.y = height > 0 ? height : 0.01;
                best_marker.scale.z = depth > 0 ? depth : 0.01;
                best_marker.color.r = 1.0; best_marker.color.g = 0.5; best_marker.color.b = 0.0; best_marker.color.a = 0.5; // Orange
                best_marker.lifetime = ros::Duration();
            }
        } else {
             ROS_INFO("Box fit (plane method): Failed (Plane 1 not found or total inliers < %.2f%%).", config_.min_primitive_inlier_percentage * 100.0);
        }
    } // End Box Fitting

    // --- Publish the best marker ---
    if (best_primitive_type != -1) {
        ROS_INFO("Selected best primitive: %s (Inlier %%: %.2f)",
                 (best_primitive_type == 0 ? "Sphere" : (best_primitive_type == 1 ? "Cylinder" : "Box")),
                 best_inlier_percentage * 100.0);
        primitive_marker_pub_.publish(best_marker);
    } else {
        ROS_INFO("No suitable primitive could be fitted to the selected segment.");
        // The DELETE marker was already published at the start.
    }
}

void PCLProcessor::processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ros::Time start_time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ROS_INFO("Input cloud has %zu points", cloud->size());

    auto cloud_filtered = downsample(cloud);
    cloud_filtered = filterDepth(cloud_filtered);

    auto segmented_cloud = segmentLCCP(cloud_filtered);

    // Apply planar filtering
    auto segmented_cloud_filtered = filterPlanarSegments(segmented_cloud);

    // Compute centroids and find the selected segment label
    uint32_t selected_label; // Variable to hold the result
    auto label_selected = computeCentroids(segmented_cloud_filtered, selected_label);

    // Color the filtered cloud (containing potentially multiple segments)
    auto colored_cloud = colorSegments(segmented_cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = cloud_msg->header;
    pub_.publish(output);

    if (label_selected) {
        ROS_INFO("Processing completed. Selected segment label: %u", selected_label);

        // --- Extract selected segment and fit primitive ---
        pcl::PointCloud<pcl::PointXYZL>::Ptr selected_segment_cloud(new pcl::PointCloud<pcl::PointXYZL>());
        selected_segment_cloud->header = segmented_cloud_filtered->header; // Copy header
        for (const auto& pt : segmented_cloud_filtered->points) {
            if (pt.label == selected_label) {
                selected_segment_cloud->points.push_back(pt);
            }
        }
        selected_segment_cloud->width = selected_segment_cloud->points.size();
        selected_segment_cloud->height = 1;
        selected_segment_cloud->is_dense = true;

        if (!selected_segment_cloud->points.empty()) {
            fitAndPublishPrimitive(selected_segment_cloud, cloud_msg->header); // Pass header
        } else {
            ROS_WARN("Selected segment cloud (label %u) is empty after extraction.", selected_label);
            // Clear primitive marker if the extracted cloud is empty
            visualization_msgs::Marker clear_marker;
            clear_marker.header = cloud_msg->header;
            clear_marker.ns = "fitted_primitive";
            clear_marker.id = 0;
            clear_marker.action = visualization_msgs::Marker::DELETE;
            primitive_marker_pub_.publish(clear_marker);
        }
        // --- End primitive fitting ---

    } else {
        ROS_WARN("Processing completed, but no segment was selected.");
        // Clear primitive marker if no segment was selected
        visualization_msgs::Marker clear_marker;
        clear_marker.header = cloud_msg->header;
        clear_marker.ns = "fitted_primitive";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETE;
        primitive_marker_pub_.publish(clear_marker);
    }

    ros::Duration duration = ros::Time::now() - start_time;
    ROS_INFO("Processing time: %.2f milliseconds", duration.toSec() * 1000.0);

    if(config_.debug_mode) {
        std::cout << "Press Enter to process the next message..." << std::endl;
        std::cin.get();
    }
}