#include "stdafx.h"
#include "pcl_processing/PCLProcessor.hpp"
#include <pcl/common/pca.h>
#include <pcl/common/angles.h> // <<< Add this for deg2rad
#include <pcl/filters/extract_indices.h> // <<< Add this for ExtractIndices
#include <std_msgs/Header.h>

PCLProcessor::PCLProcessor(ros::NodeHandle& nh, const PCLProcessorConfig& config)
    : config_(config)
{
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.output_topic, 1);
    supervoxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.supervoxel_cloud_topic, 1);
    centroid_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(config_.centroid_marker_topic, 1);
    primitive_marker_pub_ = nh.advertise<visualization_msgs::Marker>(config_.primitive_marker_topic, 1);
    filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.filtered_cloud_topic, 1);
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

        // --- Store original size ---
        const size_t original_segment_size = segment_cloud->points.size();

        // --- Create a copy for modification ---
        pcl::PointCloud<pcl::PointXYZL>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZL>(*segment_cloud));

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
        size_t plane1_inliers_count = 0; // Store counts separately
        size_t plane2_inliers_count = 0;

        // --- Fit First Plane ---
        // Use the copy 'remaining_cloud' for input
        seg_plane.setInputCloud(remaining_cloud);
        seg_plane.segment(*inliers_plane1, *coefficients_plane1);

        if (!inliers_plane1->indices.empty()) {
            plane1_found = true;
            plane1_inliers_count = inliers_plane1->indices.size(); // Store count

            // Extract inliers for plane 1 from the copy
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers_plane1);
            extract.setNegative(false);
            extract.filter(*cloud_plane1);

            // Extract outliers (remaining points) for potential second plane fit
            // This modifies the 'remaining_cloud' copy
            extract.setNegative(true);
            extract.filter(*remaining_cloud);

            ROS_INFO("Box fit: Found first plane with %zu inliers.", plane1_inliers_count);

            // --- Fit Second Perpendicular Plane ---
            if (remaining_cloud->points.size() >= 3) {
                // Use the modified 'remaining_cloud' copy
                seg_plane.setInputCloud(remaining_cloud);
                // Set constraints for perpendicular plane
                seg_plane.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
                Eigen::Vector3f axis(coefficients_plane1->values[0], coefficients_plane1->values[1], coefficients_plane1->values[2]);
                seg_plane.setAxis(axis);
                seg_plane.setEpsAngle(pcl::deg2rad(15.0)); // Angle tolerance

                seg_plane.segment(*inliers_plane2, *coefficients_plane2);

                if (!inliers_plane2->indices.empty()) {
                    plane2_found = true;
                    plane2_inliers_count = inliers_plane2->indices.size(); // Store count

                    // Extract inliers for plane 2 from the modified copy
                    extract.setInputCloud(remaining_cloud);
                    extract.setIndices(inliers_plane2);
                    extract.setNegative(false);
                    extract.filter(*cloud_plane2);
                    ROS_INFO("Box fit: Found second perpendicular plane with %zu inliers.", plane2_inliers_count);
                } else {
                    ROS_INFO("Box fit: Could not find a second perpendicular plane.");
                }
            } else {
                 ROS_INFO("Box fit: Not enough remaining points (%zu) to search for second plane.", remaining_cloud->points.size());
            }
        } else {
             ROS_INFO("Box fit: Could not find the first plane.");
        }

        // --- Check Total Inlier Percentage (using original size) ---
        // Calculate the total number of points explained by the planes found
        size_t total_explained_points = plane1_inliers_count + plane2_inliers_count;
        // Calculate percentage relative to the ORIGINAL segment size
        double box_inlier_percentage = static_cast<double>(total_explained_points) / original_segment_size;

        ROS_INFO("Box fit (plane method): Total inliers = %zu (%.2f%% of original %zu)",
                 total_explained_points, box_inlier_percentage * 100.0, original_segment_size);

        // Proceed only if plane 1 was found and total explained percentage meets threshold
        // Use the calculated box_inlier_percentage here
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

            // --- Calculate Depth (Paper's Method: Use Plane 2 if found) ---
            float depth = 0.005f; // Default "thin" object depth
            float min_proj3 = 0.0f; // Initialize projection values
            float max_proj3 = 0.0f;

            if (plane2_found && !cloud_plane2->points.empty()) {
                min_proj3 = std::numeric_limits<float>::max();
                max_proj3 = std::numeric_limits<float>::lowest();
                for (const auto& pt : cloud_plane2->points) { // Use second plane's inliers
                    // Project onto the third axis relative to the first plane's center
                    float proj3 = (pt.getVector3fMap() - center_plane).dot(axis3);
                    min_proj3 = std::min(min_proj3, proj3);
                    max_proj3 = std::max(max_proj3, proj3);
                }
                // Check if projection range is reasonable
                if (max_proj3 > min_proj3) {
                     depth = max_proj3 - min_proj3;
                     ROS_INFO("Box fit: Depth calculated from second plane: %.3f", depth);
                } else {
                    ROS_WARN("Box fit: Second plane projection resulted in non-positive depth. Using default %.3f.", depth);
                    // Keep the default depth = 0.005f
                    // Reset projection range for center calculation if depth is default
                    min_proj3 = -depth / 2.0f;
                    max_proj3 = depth / 2.0f;
                }
            } else {
                 ROS_INFO("Box fit: Second plane not found or empty. Using default depth %.3f.", depth);
                 // Set projection range for center calculation if depth is default
                 min_proj3 = -depth / 2.0f;
                 max_proj3 = depth / 2.0f;
            }


            // --- Calculate Final Center and Orientation ---
            // Adjust center based on the projection range along axis3 relative to center_plane
            // Use the min_proj3 and calculated depth
            Eigen::Vector3f geometric_center_world = center_plane + axis3 * (min_proj3 + depth / 2.0f);
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
            // Use the calculated box_inlier_percentage for comparison
            if (box_inlier_percentage > best_inlier_percentage) {
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
             ROS_INFO("Box fit (plane method): Failed (Plane 1 not found or explained points < %.2f%%).", config_.min_primitive_inlier_percentage * 100.0);
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
    ROS_INFO("Received cloud message.");

    // 0. Convert message to PCL object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
        ROS_WARN("Input cloud is empty after conversion.");
        return;
    }
    ROS_INFO("Input cloud has %zu points", cloud->size());

    // 1. Crop & Downsample
    auto cloud_preprocessed = preprocessCloud(cloud);
    if (cloud_preprocessed->empty()) {
        ROS_WARN("Cloud is empty after preprocessing.");
        return;
    }
    ROS_INFO("Cloud has %zu points after preprocessing.", cloud_preprocessed->size());

    // 2. Segmentation with LCCP -> Map
    auto segment_map = segmentWithLCCP(cloud_preprocessed);
    if (segment_map.empty()) {
        ROS_WARN("No segments found after LCCP segmentation.");
        return;
    }
    ROS_INFO("Found %zu segments initially.", segment_map.size());

    // 3. Filter non-graspable planar objects
    auto filtered_segment_map = filterPlanarSegmentsFromMap(segment_map);
    if (filtered_segment_map.empty()) {
        ROS_WARN("No segments remaining after planar filtering.");
        return;
    }
    ROS_INFO("%zu segments remain after planar filtering.", filtered_segment_map.size());

    // Optional: Publish filtered cloud
    if (config_.publish_filtered_cloud) {
        publishSegmentMapVisualization(filtered_segment_map, cloud_msg->header);
    }

    // 4. Select target object
    pcl::PointCloud<pcl::PointXYZL>::Ptr target_segment_cloud = selectTargetSegment(filtered_segment_map);
    if (!target_segment_cloud) {
        ROS_WARN("No target segment selected.");
        return;
    }
    uint32_t selected_label = target_segment_cloud->points[0].label; // Assuming all points have the same label
    ROS_INFO("Selected target segment label: %u with %zu points.", selected_label, target_segment_cloud->size());

    // 5. Fit primitive to target
    // auto best_fitting_primitive = fitBestPrimitive(target_segment_cloud);
    fitAndPublishPrimitive(target_segment_cloud, cloud_msg->header);

    ros::Duration duration = ros::Time::now() - start_time;
    ROS_INFO("Processing time: %.2f milliseconds", duration.toSec() * 1000.0);

    // Debug wait
    if(config_.debug_mode) {
        std::cout << "Press Enter to process the next message..." << std::endl;
        std::cin.get();
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::preprocessCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {

    ROS_INFO("Preprocessing cloud with %zu points...", input_cloud->size());

    // Step 1: Filter by depth
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_filtered_cloud = filterDepth(input_cloud);

    if (depth_filtered_cloud->empty()) {
        ROS_WARN("Cloud is empty after depth filtering.");
        return depth_filtered_cloud; // Return empty cloud
    }
    ROS_INFO("Cloud has %zu points after depth filtering.", depth_filtered_cloud->size());

    // Step 2: Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud = downsample(depth_filtered_cloud);

    if (downsampled_cloud->empty()) {
         ROS_WARN("Cloud is empty after downsampling.");
         // Return empty cloud, but preserve header if possible
         downsampled_cloud->header = input_cloud->header;
         return downsampled_cloud;
    }
    ROS_INFO("Cloud has %zu points after downsampling.", downsampled_cloud->size());

    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::filterDepth(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    filtered_cloud->header = input_cloud->header;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(config_.min_depth, config_.max_depth);
    pass.filter(*filtered_cloud);
    ROS_INFO("Depth filtering with limits [%.2f, %.2f] removed %zu points.",
             config_.min_depth, config_.max_depth,
             input_cloud->size() - filtered_cloud->size());
    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLProcessor::downsample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_filtered->header = input_cloud->header;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
    voxel_filter.filter(*cloud_filtered);
    ROS_INFO("Downsampling with leaf size %.2f removed %zu points.",
             config_.voxel_leaf_size,
             input_cloud->size() - cloud_filtered->size());
    return cloud_filtered;
}

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> PCLProcessor::segmentWithLCCP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {

    ROS_DEBUG("Starting LCCP segmentation process...");

    // Step 1: Create supervoxel clustering object
    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(config_.voxel_resolution, config_.seed_resolution);
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters =
        createSupervoxelClusters(input_cloud, super);

    if (supervoxel_clusters.empty()) {
        ROS_WARN("Supervoxel clustering resulted in zero clusters.");
        return {}; // Return empty map
    }

    // Step 2: Get adjacency information
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    ROS_DEBUG("Got supervoxel adjacency information.");

    // Step 3: Get labeled cloud from supervoxels
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    // IMPORTANT: Ensure header is set immediately after getting the cloud
    sv_labeled_cloud->header = input_cloud->header;
    ROS_DEBUG("Got labeled supervoxel cloud with %zu points.", sv_labeled_cloud->size());

    // Step 4: Publish supervoxel cloud if requested (calls helper function)
    if (config_.publish_supervoxel_cloud) {
        publishSupervoxelVisualization(sv_labeled_cloud);
    }

    // Step 5: Perform LCCP segmentation
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud =
        performLCCPSegmentation(supervoxel_clusters, supervoxel_adjacency, sv_labeled_cloud);

    if (lccp_labeled_cloud->empty()) {
        ROS_WARN("LCCP segmentation resulted in an empty cloud.");
        return {}; // Return empty map
    }
    ROS_DEBUG("LCCP segmentation completed, resulting cloud has %zu points.", lccp_labeled_cloud->size());

    // Step 6: Convert LCCP cloud to map
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> initial_segment_map =
        createSegmentMapFromCloud(lccp_labeled_cloud);

    if (initial_segment_map.empty()) {
        ROS_WARN("LCCP segmentation resulted in an empty map (after conversion).");
        return {};
    }
    ROS_DEBUG("Created initial segment map with %zu segments.", initial_segment_map.size());

    // Step 7: Filter small segments
    auto filtered_map = filterSmallSegments(initial_segment_map);

    ROS_DEBUG("LCCP segmentation and filtering process finished.");
    return filtered_map;
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

void PCLProcessor::publishSupervoxelVisualization(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& sv_labeled_cloud) {
    
    if (!sv_labeled_cloud || sv_labeled_cloud->empty()) {
        ROS_WARN("Cannot publish empty supervoxel cloud.");
        return;
    }

    ROS_DEBUG("Coloring and publishing supervoxel cloud...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sv_colored_cloud = colorSegments(sv_labeled_cloud);
    sensor_msgs::PointCloud2 sv_msg;
    pcl::toROSMsg(*sv_colored_cloud, sv_msg);
    supervoxel_pub_.publish(sv_msg);
    ROS_DEBUG("Published supervoxel cloud with %zu points.", sv_colored_cloud->size());
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

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> PCLProcessor::createSegmentMapFromCloud(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud) {

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> segment_map;
    if (!labeled_cloud || labeled_cloud->empty()) {
        ROS_WARN("Cannot create segment map from empty labeled cloud.");
        return segment_map; // Return empty map
    }

    for (const auto& point : labeled_cloud->points) {
        uint32_t label = point.label;
        // Skip label 0 if it represents background/unlabeled
        // if (label == 0) continue;

        if (segment_map.find(label) == segment_map.end()) {
            segment_map[label] = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
            // IMPORTANT: Copy header to each new segment cloud
            segment_map[label]->header = labeled_cloud->header;
        }
        segment_map[label]->points.push_back(point);
    }

    // Update width/height for each segment cloud in the map (optional but good practice)
    for (auto& pair : segment_map) {
        pair.second->width = pair.second->points.size();
        pair.second->height = 1;
        pair.second->is_dense = true; // Assuming no NaN/inf points in segments
    }

    ROS_DEBUG("Created segment map with %zu segments.", segment_map.size());
    return segment_map;
}

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> PCLProcessor::filterSmallSegments(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map) {

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> filtered_map;
    size_t original_segment_count = segment_map.size();
    size_t points_in_removed_segments = 0;

    ROS_DEBUG("Filtering small segments from map. Input map has %zu segments.", original_segment_count);

    // Use the same minimum segment size threshold as LCCP for consistency,
    // or introduce a new config parameter if different behavior is desired.
    size_t min_size_threshold = config_.min_segment_size;
    ROS_DEBUG("Using minimum segment size threshold: %zu", min_size_threshold);

    for (const auto& pair : segment_map) {
        const uint32_t& label = pair.first;
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud = pair.second;

        if (cloud && cloud->size() >= min_size_threshold) {
            // Keep this segment, add it to the filtered map.
            // The PointCloud::Ptr is copied, preserving the cloud and its header.
            filtered_map[label] = cloud;
        } else {
            // Segment is too small (or cloud pointer is null), filter it out.
            size_t removed_size = (cloud ? cloud->size() : 0);
            points_in_removed_segments += removed_size;
            ROS_DEBUG("Removing segment %u with size %zu (threshold %zu)", label, removed_size, min_size_threshold);
        }
    }

    size_t removed_segment_count = original_segment_count - filtered_map.size();
    ROS_DEBUG("Small segment filtering removed %zu segments and %zu points. Remaining segments: %zu",
             removed_segment_count,
             points_in_removed_segments,
             filtered_map.size());

    return filtered_map;
}

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> PCLProcessor::filterPlanarSegmentsFromMap(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map) {

    if (!config_.filter_planar_segments) {
        ROS_INFO("Planar segment filtering disabled. Returning original map.");
        return segment_map; // Return the input map directly
    }

    ROS_INFO("Starting planar segment filtering on map with %zu segments...", segment_map.size());

    // Step 1: Identify labels to remove
    std::set<uint32_t> planar_labels_to_remove = identifyPlanarLabelsToRemove(segment_map);

    // Step 2: Create the filtered map
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> filtered_map =
        createFilteredMap(segment_map, planar_labels_to_remove);

    // Step 3: Log summary (Optional: Calculate points removed)
    size_t planar_segments_found = planar_labels_to_remove.size();
    size_t original_segment_count = segment_map.size();
    // Optional: Calculate points removed for more detailed logging
    size_t total_points_before = 0;
    size_t total_points_after = 0;
    for(const auto& pair : segment_map) {
        if (pair.second) total_points_before += pair.second->size();
    }
    for(const auto& pair : filtered_map) {
        if (pair.second) total_points_after += pair.second->size();
    }

    ROS_INFO("Planar filtering removed %zu segments identified as planar, %zu points removed. Remaining segments: %zu",
             planar_segments_found,
             total_points_before - total_points_after,
             filtered_map.size());

    return filtered_map;
}

std::set<uint32_t> PCLProcessor::identifyPlanarLabelsToRemove(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map) {

    std::set<uint32_t> labels_to_remove;
    pcl::SACSegmentation<pcl::PointXYZL> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Setup RANSAC parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config_.planar_distance_threshold);
    seg.setMaxIterations(100); // Max iterations for RANSAC

    for (const auto& segment_pair : segment_map) {
        uint32_t label = segment_pair.first;
        const auto& segment_cloud = segment_pair.second;

        // --- Guard Clauses ---
        if (!segment_cloud || segment_cloud->empty()) {
            ROS_WARN("Segment %u has null or empty cloud pointer, skipping planar check.", label);
            continue;
        }
        if (segment_cloud->points.size() < 3) { // Need at least 3 points for plane fitting
            continue;
        }
        // Only check segments larger than the configured size threshold
        if (segment_cloud->points.size() < config_.max_planar_segment_size) {
            continue;
        }

        // --- Perform RANSAC ---
        seg.setInputCloud(segment_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            ROS_DEBUG("Could not fit plane to segment %u with %zu points (size > %zu).",
                      label, segment_cloud->points.size(), static_cast<size_t>(config_.max_planar_segment_size));
            continue;
        }

        // --- Check Inlier Percentage ---
        double inlier_percentage = static_cast<double>(inliers->indices.size()) / segment_cloud->points.size();
        if (inlier_percentage >= config_.min_planar_inlier_percentage) {
            labels_to_remove.insert(label);
            ROS_INFO("Segment %u identified as planar (size: %zu > %zu, inliers: %.2f%% >= %.2f%%). Marked for removal.",
                     label, segment_cloud->points.size(), static_cast<size_t>(config_.max_planar_segment_size),
                     inlier_percentage * 100.0, config_.min_planar_inlier_percentage * 100.0);
        }
    }
    return labels_to_remove;
}

std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> PCLProcessor::createFilteredMap(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& original_map,
    const std::set<uint32_t>& labels_to_remove) {

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> filtered_map;
    for (const auto& segment_pair : original_map) {
        uint32_t label = segment_pair.first;
        // Keep the segment if its label is NOT in the removal set
        if (labels_to_remove.find(label) == labels_to_remove.end()) {
            filtered_map[label] = segment_pair.second; // Copy the shared_ptr
        }
    }
    return filtered_map;
}

void PCLProcessor::publishSegmentMapVisualization(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
    const std_msgs::Header& header) {

    if (!config_.publish_filtered_cloud || segment_map.empty()) {
        return;
    }

    // Combine all clouds in the map into one
    pcl::PointCloud<pcl::PointXYZL>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    // Set header early
    pcl_conversions::toPCL(header, combined_cloud->header);

    for (const auto& pair : segment_map) {
        if (pair.second) { // Check if the pointer is valid
            *combined_cloud += *(pair.second);
        }
    }

    if (combined_cloud->empty()) {
        ROS_WARN("Cannot publish empty combined filtered segment cloud.");
        return;
    }

    ROS_DEBUG("Coloring and publishing filtered segment map cloud...");
    // Color the combined cloud based on original labels
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = colorSegments(combined_cloud);

    // Publish
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*colored_cloud, msg);
    // Ensure header is correctly set on the outgoing message
    msg.header = header;
    filtered_pub_.publish(msg);
    ROS_DEBUG("Published filtered segment map cloud with %zu points.", colored_cloud->size());
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::selectTargetSegment(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map) {

    // Initial check for empty map
    if (segment_map.empty()) {
        ROS_WARN("Cannot select target segment: Input map is empty.");
        // Clear markers if visualization is enabled
        publishCentroidMarkersIfNeeded({}, {}, false, 0); // Pass empty maps/data
        return nullptr;
    }

    // Step 1: Compute centroids
    std::map<uint32_t, Eigen::Vector4f> centroids = computeSegmentCentroids(segment_map);

    // Step 2: Find the label of the closest centroid
    uint32_t closest_label = 0; // Initialize (value used only if label_selected is true)
    bool label_selected = findClosestCentroidLabel(centroids, closest_label);

    // Step 3: Publish markers if needed (handles config check and header finding)
    publishCentroidMarkersIfNeeded(segment_map, centroids, label_selected, closest_label);

    // Step 4: Return the selected cloud's pointer or nullptr
    if (label_selected) {
        return getSelectedCloudPtr(segment_map, closest_label);
    } else {
        // No segment was selected (findClosestCentroidLabel returned false)
        return nullptr;
    }
}

void PCLProcessor::publishCentroidMarkersIfNeeded(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
    const std::map<uint32_t, Eigen::Vector4f>& centroids,
    bool label_selected,
    uint32_t selected_label) {

    if (!config_.publish_centroid_markers) {
        return; // Visualization disabled
    }

    // Try to find a valid header from the input map
    std_msgs::Header current_header;
    bool header_found = false;
    for (const auto& pair : segment_map) {
        if (pair.second && !pair.second->header.frame_id.empty()) {
            pcl_conversions::fromPCL(pair.second->header, current_header); // <<< With this line
            header_found = true;
            break;
        }
    }

    if (header_found) {
        // Publish using the found header
        publishCentroidMarkers(centroids, label_selected, selected_label, current_header);
    } else {
        ROS_WARN("Could not find valid header in segment map for centroid markers.");
        // Attempt to clear using the last known header if no current one is available
        // This handles the case where segment_map might be non-empty but contain no valid headers
        publishCentroidMarkers({}, false, 0, last_centroid_header_);
    }
}

void PCLProcessor::publishCentroidMarkers(
    const std::map<uint32_t, Eigen::Vector4f>& centroids,
    bool has_selected_label, // Flag
    uint32_t selected_label,   // Label (only valid if flag is true)
    const std_msgs::Header& header) {

    // Clear previous markers using the LAST known header
    if (!last_centroid_header_.frame_id.empty()) {
        // ... (DELETEALL logic using last_centroid_header_ as before) ...
        visualization_msgs::MarkerArray delete_markers;
        visualization_msgs::Marker marker_delete;
        marker_delete.header = last_centroid_header_; // Use stored header
        marker_delete.header.stamp = ros::Time::now(); // Use current time for delete action
        marker_delete.ns = "segment_centroids";
        marker_delete.id = 0;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(marker_delete);
        centroid_marker_pub_.publish(delete_markers);
    }

    // Create and publish new markers using the CURRENT header
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    bool markers_published = false; // Track if any markers are actually added
    for (const auto& centroid_pair : centroids) {
        uint32_t label = centroid_pair.first;
        const Eigen::Vector4f& centroid = centroid_pair.second;

        if (!std::isfinite(centroid[0]) || !std::isfinite(centroid[1]) || !std::isfinite(centroid[2])) {
             ROS_WARN("Centroid for label %u contains NaN/Inf, skipping marker.", label);
             continue;
        }

        visualization_msgs::Marker marker;
        marker.header = header; // Use CURRENT header for ADD
        marker.header.stamp = ros::Time::now();
        marker.ns = "segment_centroids";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;

        // Color: Green if selected, Red otherwise
        // Use the boolean flag to check if selected_label is valid
        if (has_selected_label && label == selected_label) {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green
        } else {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red
        }
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
        markers_published = true;
    }

    if (markers_published) { // Only publish and update header if markers were added
        centroid_marker_pub_.publish(marker_array);
        last_centroid_header_ = header; // Store the header used for the successful publish
        ROS_DEBUG("Published %zu centroid markers.", marker_array.markers.size());
    } else if (!centroids.empty()) {
        ROS_WARN("No valid centroid markers were created despite having centroids.");
    }
}

std::map<uint32_t, Eigen::Vector4f> PCLProcessor::computeSegmentCentroids(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map) {

    std::map<uint32_t, Eigen::Vector4f> centroids;
    ROS_DEBUG("Computing centroids for %zu segments...", segment_map.size());

    for (const auto& segment_pair : segment_map) {
        uint32_t label = segment_pair.first;
        const auto& segment_cloud = segment_pair.second;

        if (!segment_cloud || segment_cloud->points.empty()) {
            ROS_WARN("Segment %u has null or empty cloud pointer, cannot compute centroid.", label);
            continue;
        }

        // Use PCL's built-in centroid computation
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*segment_cloud, centroid);
        centroids[label] = centroid;

        ROS_DEBUG("Segment %u: centroid at (%.3f, %.3f, %.3f) with %zu points",
                 label, centroid[0], centroid[1], centroid[2], segment_cloud->points.size());
    }
    ROS_DEBUG("Computed %zu centroids.", centroids.size());
    return centroids;
}

bool PCLProcessor::findClosestCentroidLabel( // Return bool
    const std::map<uint32_t, Eigen::Vector4f>& centroids,
    uint32_t& selected_label_out) { // Output parameter

    if (centroids.empty()) {
        ROS_WARN("Cannot find closest centroid: Input map is empty.");
        return false; // Indicate failure
    }

    bool label_found = false; // Flag to track if a valid label was found
    double min_dist_sq = std::numeric_limits<double>::max();

    for (const auto& centroid_pair : centroids) {
        uint32_t label = centroid_pair.first;
        const Eigen::Vector4f& centroid = centroid_pair.second;

        if (!std::isfinite(centroid[0]) || !std::isfinite(centroid[1])) {
             ROS_WARN("Centroid for label %u contains NaN/Inf, skipping.", label);
             continue;
        }
        double dist_sq = centroid[0] * centroid[0] + centroid[1] * centroid[1];

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            selected_label_out = label; // Set the output parameter
            label_found = true;       // Mark that we found one
        }
    }

    if (label_found) {
        ROS_INFO("Selected segment label: %u (XY distance: %.3f)", selected_label_out, std::sqrt(min_dist_sq));
    } else {
         ROS_WARN("Could not select a closest segment (no valid centroids found?).");
    }

    return label_found; // Return true if found, false otherwise
}

pcl::PointCloud<pcl::PointXYZL>::Ptr PCLProcessor::getSelectedCloudPtr(
    const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
    uint32_t selected_label) {

    auto it = segment_map.find(selected_label);
    if (it != segment_map.end()) {
        return it->second; // Return the shared_ptr
    } else {
        // This indicates an internal logic error if selected_label came from findClosestCentroidLabel
        ROS_ERROR("Internal error: Selected label %u not found in segment map during pointer retrieval!", selected_label);
        return nullptr;
    }
}
