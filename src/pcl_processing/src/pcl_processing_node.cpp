// filepath: pcl_processing/src/downsample.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>


// Add this function to generate random colors for segments
// Improved function to generate distinct, vivid colors
std::vector<uint32_t> generateColors(size_t count) {
    std::vector<uint32_t> colors(count);
    
    // Use a deterministic seed for consistent colors between runs
    srand(42);
    
    for (size_t i = 0; i < count; i++) {
        // Use HSV to RGB conversion for more distinct colors
        float hue = static_cast<float>(i) / count * 360.0f;
        float saturation = 0.7f + 0.3f * (rand() % 100) / 100.0f; // 0.7-1.0
        float value = 0.7f + 0.3f * (rand() % 100) / 100.0f;      // 0.7-1.0
        
        // Convert HSV to RGB (simplified conversion)
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
        
        // Scale to 0-255 and store as RGB
        uint8_t rr = static_cast<uint8_t>(r * 255);
        uint8_t gg = static_cast<uint8_t>(g * 255);
        uint8_t bb = static_cast<uint8_t>(b * 255);
        
        colors[i] = ((uint32_t)rr << 16 | (uint32_t)gg << 8 | (uint32_t)bb);
    }
    return colors;
}

// Function to create a colored point cloud from segmented cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSegmentedCloud(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud) {
    
    // First, collect all unique labels
    std::set<uint32_t> unique_labels;
    for (const auto& point : labeled_cloud->points) {
        unique_labels.insert(point.label);
    }
    
    // Generate random colors for each label
    std::vector<uint32_t> colors = generateColors(unique_labels.size());
    std::map<uint32_t, uint32_t> label_to_color;
    
    size_t color_idx = 0;
    for (const auto& label : unique_labels) {
        label_to_color[label] = colors[color_idx++];
    }

    // Create a new colored cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    colored_cloud->width = labeled_cloud->width;
    colored_cloud->height = labeled_cloud->height;
    colored_cloud->is_dense = labeled_cloud->is_dense;
    colored_cloud->points.resize(labeled_cloud->points.size());
    
    // Assign colors based on segment labels
    for (size_t i = 0; i < labeled_cloud->points.size(); i++) {
        const auto& labeled_point = labeled_cloud->points[i];
        auto& colored_point = colored_cloud->points[i];
        
        // Copy XYZ coordinates
        colored_point.x = labeled_point.x;
        colored_point.y = labeled_point.y;
        colored_point.z = labeled_point.z;
        
        // Set color based on segment label
        uint32_t rgb_color = label_to_color[labeled_point.label];
        uint8_t r = (rgb_color >> 16) & 0xFF;
        uint8_t g = (rgb_color >> 8) & 0xFF;
        uint8_t b = rgb_color & 0xFF;
        
        colored_point.r = r;
        colored_point.g = g;
        colored_point.b = b;

    }
    
    return colored_cloud;
}


// Function to perform LCCP segmentation
pcl::PointCloud<pcl::PointXYZL>::Ptr segmentPointCloudLCCP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {

   // Parameters for supervoxel clustering - using values closer to the example
   float voxel_resolution = 0.03f;     // Similar to example (0.0075)
   float seed_resolution = 0.15f;      // Similar to example (0.03)
   float color_importance = 0.1f;      // From example
   float spatial_importance = 1.0f;    // From example
   float normal_importance = 5.0f;     // From example
   bool use_supervoxel_refinement = true;

   // Create supervoxel clustering object
   pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
   super.setInputCloud(input_cloud);
   super.setColorImportance(color_importance);
   super.setSpatialImportance(spatial_importance);
   super.setNormalImportance(normal_importance);
   super.setUseSingleCameraTransform(false);


    // Perform supervoxel clustering
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    ROS_INFO("Extracted %lu supervoxels", supervoxel_clusters.size());


    if (use_supervoxel_refinement) {
        super.refineSupervoxels(2, supervoxel_clusters); // Pass the supervoxel_clusters map// Refine supervoxels (optional)
    }

    // Get adjacency map of supervoxels
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // Perform LCCP segmentation - using values closer to the example
    pcl::LCCPSegmentation<pcl::PointXYZRGB> lccp;
    lccp.setConcavityToleranceThreshold(15.0);      // From example
    lccp.setSanityCheck(true);                      // From example
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, 0.15); // From example
    lccp.setKFactor(0.5);                             // From example
    lccp.setMinSegmentSize(500);                    // Adjusted for your data
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.segment();
   
 
    // Get the labeled point cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);

    // Count unique labels in the segmented cloud
    std::set<uint32_t> unique_labels;
    for (const auto& point : lccp_labeled_cloud->points) {
        unique_labels.insert(point.label);
    }

    // Count points in each segment
    std::map<uint32_t, size_t> segment_sizes;
    for (const auto& point : lccp_labeled_cloud->points) {
        segment_sizes[point.label]++;
    }
    
    // Sort segments by size (largest to smallest)
    std::vector<std::pair<uint32_t, size_t>> sorted_segments;
    for (const auto& segment : segment_sizes) {
        sorted_segments.push_back(segment);
    }
    
    std::sort(sorted_segments.begin(), sorted_segments.end(), 
              [](const std::pair<uint32_t, size_t>& a, const std::pair<uint32_t, size_t>& b) {
                  return a.second > b.second; 
              });
    
    // Print segment sizes
    ROS_INFO("Segment sizes (label: point count):");
    for (const auto& segment : sorted_segments) {
        ROS_INFO("  Label %u: %zu points (%.2f%% of cloud)", 
                segment.first, 
                segment.second,
                100.0f * segment.second / lccp_labeled_cloud->points.size());
    }

    // Filter out small segments (less than 1% of total points)
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    filtered_cloud->header = lccp_labeled_cloud->header;
    filtered_cloud->is_dense = lccp_labeled_cloud->is_dense;
    
    // Determine which segments to keep (segments with >= 1% of points)
    std::set<uint32_t> segments_to_keep;
    size_t points_threshold = static_cast<size_t>(lccp_labeled_cloud->points.size() * 0.01); // 1% threshold
    
    for (const auto& segment : sorted_segments) {
        if (segment.second >= points_threshold) {
            segments_to_keep.insert(segment.first);
            ROS_INFO("Keeping segment %u with %zu points (%.2f%%)", 
                    segment.first, segment.second, 
                    100.0f * segment.second / lccp_labeled_cloud->points.size());
        } else {
            ROS_INFO("Filtering out segment %u with %zu points (%.2f%%)", 
                    segment.first, segment.second,
                    100.0f * segment.second / lccp_labeled_cloud->points.size());
        }
    }
    
    // Copy only the points from segments we want to keep
    for (const auto& point : lccp_labeled_cloud->points) {
        if (segments_to_keep.find(point.label) != segments_to_keep.end()) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    // Update width and height for unorganized point cloud
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    
    ROS_INFO("After filtering: Kept %zu segments with %zu points (%.2f%% of original cloud)",
             segments_to_keep.size(), filtered_cloud->points.size(),
             100.0f * filtered_cloud->points.size() / lccp_labeled_cloud->points.size());
    
    // Return the filtered cloud instead of the original
    return filtered_cloud;
}

// Declare the publisher as a global variable
ros::Publisher pub;

// Function to downsample the point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust leaf size as needed
    voxel_filter.filter(*cloud_filtered);

    // ROS_INFO("Downsampled cloud has %zu points.", cloud_filtered->points.size());
    return cloud_filtered;
}

// Function to filter out points with depth less than min_depth using PCL's PassThrough filter
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointsByDepth(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    float min_depth = 0.05) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");  // Filter based on z-axis (depth)
    pass.setFilterLimits(min_depth, FLT_MAX);  // Keep points with z > min_depth
    pass.filter(*filtered_cloud);
    
    ROS_INFO("Depth filter: removed %zu points with depth less than %.2f meters",
             input_cloud->points.size() - filtered_cloud->points.size(), min_depth);
    
    return filtered_cloud;
}
 
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ROS_INFO("Input cloud has %zu points", cloud->size());
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = downsamplePointCloud(cloud);

    cloud_filtered = filterPointsByDepth(cloud_filtered, 1.1);

    // Perform LCCP segmentation
    pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud = segmentPointCloudLCCP(cloud_filtered);
    
    // Color the segmented cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = colorSegmentedCloud(segmented_cloud);
        
    // Convert colored cloud back to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = cloud_msg->header;
 
    // Publish the downsampled cloud
    pub.publish(output);

}
 
int main(int argc, char** argv) {

    ros::init(argc, argv, "segmentation_node");

    ros::NodeHandle nh;
    
    // Initialize the publisher in the main function
    pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);

 
    // Subscribe to input point cloud topic

    ros::Subscriber sub = nh.subscribe("input_cloud", 1, cloudCallback);
 
    ros::spin();

    return 0;

}
 