#include "pcl_processing/SphereGraspLogic.hpp"
#include <limits> // Required for std::numeric_limits if using NaN

namespace pcl_processing {

// Constructor: Store relevant config values
SphereGraspLogic::SphereGraspLogic(const GraspPoseEstimatorConfig& config) :
    diameter_threshold_(config.sphere_diameter_threshold),
    padding_(config.grasp_size_padding),
    lateral_grasp_offset_(config.lateral_grasp_orientation_offset),
    min_wrist_angle_(config.wrist_min_angle),
    max_wrist_angle_(config.wrist_max_angle)
{}

// Stub implementation: Always suggest PALMAR for now if above threshold
int8_t SphereGraspLogic::selectGraspType(float radius) {
    // Example stub logic: Use PALMAR if diameter > threshold
    if (2.0 * radius > diameter_threshold_) {
        return GraspReference::GRASP_PALMAR; // Or GRASP_PINCH based on size?
    }
    return GraspReference::GRASP_NONE;
}

// Stub implementation: Return 0 orientation for sphere
float SphereGraspLogic::computeWristOrientation(int8_t grasp_type) {
    // Sphere orientation is simple, often aligned with hand base.
    // Could potentially add offset for lateral grasp if needed.
    // if (grasp_type == GraspReference::GRASP_LATERAL) {
    //     return lateral_grasp_offset_;
    // }
    return 0.0f; // Default orientation
}

// Stub implementation: Return diameter + padding
float SphereGraspLogic::computeGraspSize(float radius) {
    return (2.0f * radius) + static_cast<float>(padding_);
}

// Helper function (if needed, could be in a common utility)
float SphereGraspLogic::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace pcl_processing