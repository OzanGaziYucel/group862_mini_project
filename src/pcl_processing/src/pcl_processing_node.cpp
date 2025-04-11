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
    // print out the list of colors
    // for (size_t i = 0; i < colors.size(); ++i) {
    //     uint8_t r = (colors[i] >> 16) & 0xFF;
    //     uint8_t g = (colors[i] >> 8) & 0xFF;
    //     uint8_t b = colors[i] & 0xFF;
    //     ROS_INFO("Color %zu: R=%u, G=%u, B=%u", i, r, g, b);
    // }
    // ROS_INFO("Generated %zu colors for %zu unique labels", colors.size(), unique_labels.size());
    std::map<uint32_t, uint32_t> label_to_color;
    
    size_t color_idx = 0;
    for (const auto& label : unique_labels) {
        label_to_color[label] = colors[color_idx++];
    }

    // print out the label to color mapping
    // for (const auto& pair : label_to_color) {
    //     ROS_INFO("Label %u -> Color %u", pair.first, pair.second);
    // }
    // ROS_INFO("Created label to color mapping for %zu unique labels", label_to_color.size());
    
    // Create a new colored cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    colored_cloud->width = labeled_cloud->width;
    colored_cloud->height = labeled_cloud->height;
    colored_cloud->is_dense = labeled_cloud->is_dense;
    colored_cloud->points.resize(labeled_cloud->points.size());
    
    // Assign colors based on segment labels
    for (size_t i = 0; i < labeled_cloud->points.size(); i++) {
        const auto& labeled_point = labeled_cloud->points[i];
        // ROS_INFO("Point %zu: Label %u, X=%f, Y=%f, Z=%f", i, labeled_point.label, labeled_point.x, labeled_point.y, labeled_point.z);
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

        // Print out the color values for debugging
        // ROS_INFO("Colored Point %zu: R=%u, G=%u, B=%u", i, r, g, b);
        // ROS_INFO("Colored Point %zu: Label %u, X=%f, Y=%f, Z=%f", i, labeled_point.label, colored_point.x, colored_point.y, colored_point.z);
        // ROS_INFO("Colored Point %zu: Label %u, Color %u", i, labeled_point.label, rgb_color);
    }
    ROS_INFO("Colored cloud has %zu points.", colored_cloud->points.size());
    
    return colored_cloud;
}


// Function to perform LCCP segmentation
pcl::PointCloud<pcl::PointXYZL>::Ptr segmentPointCloudLCCP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {

   // Parameters for supervoxel clustering - using values closer to the example
   float voxel_resolution = 0.01f;     // Similar to example (0.0075)
   float seed_resolution = 0.05f;      // Similar to example (0.03)
   float color_importance = 0.2f;      // From example
   float spatial_importance = 1.0f;    // From example
   float normal_importance = 4.0f;     // From example
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
    lccp.setConcavityToleranceThreshold(10.0);      // From example
    lccp.setSanityCheck(true);                      // From example
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, 0.1); // From example
    lccp.setKFactor(0);                             // From example
    lccp.setMinSegmentSize(100);                    // Adjusted for your data
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
    
    ROS_INFO("LCCP found approximately %lu segments (unique labels)", unique_labels.size());
    ROS_INFO("LCCP segmentation completed. Cloud has %zu points.", lccp_labeled_cloud->points.size());


    return lccp_labeled_cloud;
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
 
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // print out the fırst 10 points of the input cloud
    for (size_t i = 0; i < 10 && i < cloud_msg->width * cloud_msg->height; ++i) {
        float x, y, z;
        memcpy(&x, &cloud_msg->data[i * cloud_msg->point_step], sizeof(float));
        memcpy(&y, &cloud_msg->data[i * cloud_msg->point_step + 4], sizeof(float));
        memcpy(&z, &cloud_msg->data[i * cloud_msg->point_step + 8], sizeof(float));
        // and the colors
        uint8_t r, g, b;
        memcpy(&r, &cloud_msg->data[i * cloud_msg->point_step + 12], sizeof(uint8_t));
        memcpy(&g, &cloud_msg->data[i * cloud_msg->point_step + 13], sizeof(uint8_t));
        memcpy(&b, &cloud_msg->data[i * cloud_msg->point_step + 14], sizeof(uint8_t));
        // Print out the point coordinates and colors
        ROS_INFO("Point %zu: X=%f, Y=%f, Z=%f, R=%u, G=%u, B=%u", i, x, y, z, r, g, b);
    }
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ROS_INFO("Input cloud has %zu points", cloud->size());
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = downsamplePointCloud(cloud);

    // Perform LCCP segmentation
    pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud = segmentPointCloudLCCP(cloud_filtered);
    
    // Color the segmented cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = colorSegmentedCloud(segmented_cloud);
        
    // Convert colored cloud back to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = cloud_msg->header;

    // print out the first 10 points of the output cloud
    for (size_t i = 0; i < 10 && i < output.width * output.height; ++i) {
        float x, y, z;
        memcpy(&x, &output.data[i * output.point_step], sizeof(float));
        memcpy(&y, &output.data[i * output.point_step + 4], sizeof(float));
        memcpy(&z, &output.data[i * output.point_step + 8], sizeof(float));
        // and the colors
        uint8_t r, g, b;
        memcpy(&r, &output.data[i * output.point_step + 12], sizeof(uint8_t));
        memcpy(&g, &output.data[i * output.point_step + 13], sizeof(uint8_t));
        memcpy(&b, &output.data[i * output.point_step + 14], sizeof(uint8_t));
        // Print out the point coordinates and colors
        ROS_INFO("Output Point %zu: X=%f, Y=%f, Z=%f, R=%u, G=%u, B=%u", i, x, y, z, r, g, b);
    }
 
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
 