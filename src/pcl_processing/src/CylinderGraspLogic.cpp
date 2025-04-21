#include "pcl_processing/CylinderGraspLogic.hpp"
#include <limits> // Required for std::numeric_limits if using NaN

namespace pcl_processing {

// Constructor: Store relevant config values
CylinderGraspLogic::CylinderGraspLogic(const GraspPoseEstimatorConfig& config) :
    approach_angle_threshold_(config.cylinder_approach_angle_threshold),
    length_threshold_(config.cylinder_length_threshold),
    diameter_threshold_(config.cylinder_diameter_threshold),
    padding_(config.grasp_size_padding),
    lateral_grasp_offset_(config.lateral_grasp_orientation_offset),
    min_wrist_angle_(config.wrist_min_angle),
    max_wrist_angle_(config.wrist_max_angle)
{}

// Stub implementation: Always suggest PALMAR for now
int8_t CylinderGraspLogic::selectGraspType(float radius, float length, const geometry_msgs::Vector3& axis) {
    // Placeholder: Implement actual logic based on thresholds and approach angle later
    // For now, just return a default valid grasp if dimensions seem reasonable
    if (radius > 0 && length > 0) {
        return GraspReference::GRASP_PALMAR;
    }
    return GraspReference::GRASP_NONE;
}

// Stub implementation: Return 0 orientation
float CylinderGraspLogic::computeWristOrientation(const geometry_msgs::Vector3& axis, int8_t grasp_type) {
    // Placeholder: Implement actual logic based on axis and grasp type later
    // For now, return 0
    return 0.0f;
}

// Stub implementation: Return diameter + padding
float CylinderGraspLogic::computeGraspSize(float radius) {
    return (2.0f * radius) + static_cast<float>(padding_);
}

// Helper function
float CylinderGraspLogic::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace pcl_processing