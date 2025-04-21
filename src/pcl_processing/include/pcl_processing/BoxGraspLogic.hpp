#pragma once

#include "pcl_processing/GraspPoseEstimatorConfig.hpp"
#include "pcl_processing/GraspReference.h" // For grasp type constants
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <cmath>
#include <numeric>   // For std::iota
#include <algorithm> // For std::sort, std::max_element

namespace pcl_processing {

class BoxGraspLogic {
private:
    // Store relevant config parameters
    double length_threshold_;
    double width_threshold_;
    double padding_;
    double lateral_grasp_offset_;
    double min_wrist_angle_;
    double max_wrist_angle_;

    // Helper struct to associate axis, dimension, and original index
    struct BoxFaceAxis {
        geometry_msgs::Vector3 axis;
        float dimension;
        size_t original_index; // 0, 1, or 2
    };


public:
    // Constructor takes the config struct
    explicit BoxGraspLogic(const GraspPoseEstimatorConfig& config);

    // Public methods for calculations specific to boxes
    // Assumes dimensions correspond to axes in the same order
    int8_t selectGraspType(const std::vector<float>& dimensions);
    float computeWristOrientation(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions, int8_t grasp_type);
    float computeGraspSize(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);

private:
    // Private helper methods
    float clamp(float value, float min_val, float max_val);
    // Helper to determine the face being grasped and its properties
    BoxFaceAxis getGraspingFaceAxis(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);
    // Potentially add vector math helpers if needed
};

} // namespace pcl_processing