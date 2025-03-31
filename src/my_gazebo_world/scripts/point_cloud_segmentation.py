#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from scipy.optimize import minimize, Bounds


# Global variable
o3d_cloud = None

def pointcloud2_to_open3d(pointcloud_msg):
    """ Convert ROS PointCloud2 message to Open3D PointCloud format."""
    
    points = []
    for p in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])

    if not points:
        rospy.logwarn("Empty or invalid point cloud received!")
        return None
    
    # Create Open3D points
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array(points))
    return cloud

def point_cloud_callback(msg):
    """ Converts and processes the PointCloud2 message from Kinect into Open3D format."""
    global o3d_cloud
    
    rospy.loginfo("PointCloud2 message received, converting to Open3D...")
    o3d_cloud = pointcloud2_to_open3d(msg)
    
    if o3d_cloud is not None:
        rospy.loginfo("Successfully converted to Open3D format!")

        # Every time new Kinect data arrives, mp_main_block() will be called
        mp_main_block()

def point_cloud_segmentation():
    rospy.init_node('point_cloud_segmentation', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.Subscriber("/kinect/depth/points", PointCloud2, point_cloud_callback)
    rospy.loginfo("Started listening to PointCloud2 messages...")
    rospy.spin()

def mp_main_block():
    global o3d_cloud

    if o3d_cloud is None:
        rospy.logwarn("No point cloud data available yet!")
        return

    else:
        rospy.logwarn("Point cloud data available, rotating and processing...")
        # 1. Load the point cloud data
        pcd = o3d_cloud

        rotation_matrix = np.array([
                            [1, 0, 0, 0], 
                            [0, -1, 0, 0], 
                            [0, 0, -1, 0], 
                            [0, 0, 0, 1]
                                         ])
        pcd.transform(rotation_matrix)
        points = np.asarray(pcd.points)

        # Downsampling and outlier removal
        pcd = pcd.voxel_down_sample(voxel_size=0.002)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # 2. Find and remove table using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                            ransac_n=3,
                                            num_iterations=1000)
        [a, b, c, d] = plane_model
        
        # Check if this is a horizontal plane (table)
        # Normal vector should be roughly [0,0,1] or [0,0,-1]
        is_horizontal = abs(abs(c) - 1.0) < 0.2
        
        if is_horizontal:
            # Extract table plane
            table_cloud = pcd.select_by_index(inliers)
            table_cloud.paint_uniform_color([1.0, 0, 0])  # Red for visualization
            
            # Calculate area from width in X and Y direction
            extent = table_cloud.get_axis_aligned_bounding_box().get_extent()
            area = extent[0] * extent[1]  # x * y 

            if area > 0.25:
                rospy.loginfo("✅ Plane is large enough to be considered a table. Removing it.")
                remaining_cloud = pcd.select_by_index(inliers, invert=True)
            else:
                rospy.logwarn("❌ Plane is too small to be a table. Skipping removal.")
                remaining_cloud = pcd  # Masayı silme, aynen devam
            
            # 3. Find and remove wall (if present)
            plane_model, inliers = remaining_cloud.segment_plane(distance_threshold=0.02,
                                                            ransac_n=3,
                                                            num_iterations=1000)
            
            # Check if this is a vertical plane and sufficiently large 
            inlier_ratio = len(inliers) / len(np.asarray(remaining_cloud.points))
            [a, b, c, d] = plane_model
            is_vertical = abs(c) < 0.2  # normal vector roughly perpendicular to [0,0,1]
            
            if is_vertical and inlier_ratio > 0.4:
                # Extract wall plane
                wall_cloud = remaining_cloud.select_by_index(inliers)
                wall_cloud.paint_uniform_color([0, 1.0, 0])  # Green for visualization
                
                # Remove wall points
                object_cloud = remaining_cloud.select_by_index(inliers, invert=True)
            else:
                object_cloud = remaining_cloud
        else:
            # If no horizontal plane found, continue with the original cloud
            object_cloud = pcd
        pcd=object_cloud
        pcd

        table_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Paint the town red
        o3d.visualization.draw_geometries([table_cloud], window_name="❌ Removed Surface (RANSAC Inliers)")
        o3d.visualization.draw_geometries([pcd], window_name="Planar Surfaces Removed")

        # Euclidean clustering with fine-tuned parameters
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
        labels = np.array(pcd.cluster_dbscan(eps=0.03, min_points=10, print_progress=True))

        if labels.max() < 0:
            print("No clusters found!")
            exit()

        # Select closest cluster
        centroids, cluster_indices = [], {}
        for i in range(labels.max() + 1):
            idx = np.where(labels == i)[0]
            cluster = np.asarray(pcd.points)[idx]
            centroid = cluster.mean(axis=0)
            centroids.append((i, centroid))
            cluster_indices[i] = idx

        selected_cluster_idx = max(centroids, key=lambda c: c[1][2])[0]
        selected_cluster = pcd.select_by_index(cluster_indices[selected_cluster_idx])

        # Colorize
        selected_cluster.paint_uniform_color([1.0, 0.0, 0.0])
        o3d.visualization.draw_geometries([selected_cluster], window_name="Selected Cluster")
        
        # --- Testing on Selected Cluster ---
        points = np.asarray(selected_cluster.points)

        # 1️⃣ Sphere RANSAC
        cluster_size = np.max(np.ptp(points, axis=0))
        sphere_params, sphere_error = fit_sphere(points, centroids[selected_cluster_idx][1], cluster_size)
        print("\n🔴 Sphere:")
        print(f"Center: {sphere_params['center']}")
        print(f"Radius: {sphere_params['radius']}")
        print(f"Error: {sphere_error}")

        # Sphere visualization
        sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_params['radius'])
        sphere_mesh.compute_vertex_normals()
        sphere_mesh.translate(sphere_params['center'] - np.array([0, 0, 0]))
        visualize_fit(selected_cluster, sphere_mesh, color=[0, 0, 1], title="🔵 Sphere RANSAC")

        # 2️⃣ Cylinder RANSAC
        cylinder_params, cylinder_error = fit_cylinder_bounded(points, centroids[selected_cluster_idx][1], cluster_size)
        print("\n🟣 Cylinder:")
        print(f"Center: {cylinder_params['center']}")
        print(f"Axis: {cylinder_params['axis']}")
        print(f"Radius: {cylinder_params['radius']}")
        print(f"Height: {cylinder_params['height']}")
        print(f"Error: {cylinder_error}")

        # Create Cylinder Model
        cylinder_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=cylinder_params['radius'], height=cylinder_params['height'])
        cylinder_mesh.compute_vertex_normals()

        # Normalize Axis
        axis = cylinder_params['axis'] / np.linalg.norm(cylinder_params['axis'])

        # Rotate axis to Z axis
        z_axis = np.array([0, 0, 1])
        v = np.cross(z_axis, axis)
        s = np.linalg.norm(v)
        c = np.dot(z_axis, axis)
        if s != 0:
            vx = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
            R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))
        else:
            R = np.eye(3)

        cylinder_mesh.rotate(R, center=np.array([0, 0, 0]))

        # Place it in the world
        cylinder_mesh.translate(cylinder_params['center'])

        # Cylinder visualization
        visualize_fit(selected_cluster, cylinder_mesh, color=[0.5, 0, 0.5], title="🟣 Cylinder Corrected")

        print(f"\nSelected cluster min bounds: {np.min(points, axis=0)}")
        print(f"Selected cluster max bounds: {np.max(points, axis=0)}")

        size = np.ptp(points, axis=0)
        max_size = np.max(size)
        if sphere_params['radius'] > max_size * 2:
            print(f"⚠️ Sphere radius ({sphere_params['radius']}) is too large compared to object size ({max_size})!")

        if cylinder_params['radius'] > max_size * 2:
            print(f"⚠️ Cylinder radius ({cylinder_params['radius']}) is too large compared to object size ({max_size})!")

        # Determine the model with the lowest error value (sphere, cylinder)
        errors = {"sphere": sphere_error, "cylinder": cylinder_error}
        best_model = min(errors, key=errors.get)
        print(f"\nBest model: {best_model} with error: {errors[best_model]}")

        # Grasp type selection
        grasp_type_selection(best_model, sphere_params, cylinder_params)

        print(f"\n\n")
        print_transition()
        print(f"\n\n")

def grasp_type_selection(best_model, sphere_params=None, cylinder_params=None):
    threshold_sphere = 0.03   # 3 cm
    threshold_cyl_radius = 0.015   # 1.5 cm
    margin_p = 0.002  # 2 mm
    margin_l = 0.003  # 3 mm
    reason = None
    
    if best_model == "sphere":
        if sphere_params is not None and sphere_params["radius"] > threshold_sphere:
            grasp_type = "Palmar"
            reason = "Sphere radius is larger than 3 cm"
            grasp_size = 2*sphere_params["radius"] + margin_p
        else:
            grasp_type = "Lateral" 
            reason = "Sphere radius is smaller than 3 cm"
            grasp_size = 2*sphere_params["radius"] + margin_l
    elif best_model == "cylinder":
        if cylinder_params:
            radius = cylinder_params["radius"]
            angle = cylinder_params["approach_angle"]  # Calculated in degrees
            if radius < threshold_cyl_radius:
                # If the radius is less than 1.5 cm, Lateral + 90 degree approach (tiny cylinder situation)
                reason = "Cylinder radius is smaller than 1.5 cm (Tiny cylinder)"
                grasp_type = "Lateral (angle=90 deg) from top"
                grasp_size = 2*radius + margin_l
            else:
                # Palmar + calculated approach angle value for larger cylinders
                grasp_type = f"Palmar (angle={angle:.1f} deg) from side"
                reason = "Cylinder radius is larger than 1.5 cm"
                grasp_size = 2*radius + margin_p
        else:
            # If cylinder_params is None then the default grasp type is Palmar
            reason = "Cylinder parameters are not available"
            grasp_type = "Palmar" 
    
    print(f"Selected Grasp Type: {grasp_type}")
    print(f"Reason: {reason}")
    print(f"Grasp Size: {grasp_size*1000:.1f} mm")


def visualize_fit(cloud, mesh, color, title):
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([cloud, mesh], window_name=title)

def fit_sphere(points, centroid, cluster_size):
    initial_center = centroid
    distances = np.linalg.norm(points - initial_center, axis=1)
    initial_radius = np.mean(distances)

    def sphere_error(params):
        center = params[:3]
        radius = params[3]
        distances = np.linalg.norm(points - center, axis=1)
        return np.sum((distances - radius) ** 2)

    # We add BOUND: radius max should not exceed 2 times the object
    bounds = Bounds([-np.inf, -np.inf, -np.inf, 0], [np.inf, np.inf, np.inf, cluster_size * 0.5])

    initial_params = np.append(initial_center, initial_radius)
    result = minimize(sphere_error, initial_params, method='Powell', bounds=bounds)

    params = {
        "center": result.x[:3],
        "radius": result.x[3]
    }
    error = sphere_error(result.x) / len(points)
    return params, error


def fit_cylinder_bounded(points, centroid, cluster_size):
    # Initial estimate from PCA
    cov = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis = eigvecs[:, np.argmax(eigvals)]

    # Initial estimate
    initial_center = centroid
    v = points - initial_center
    proj_lengths = np.abs(np.dot(v, axis))
    orth_v = v - np.outer(np.dot(v, axis), axis)
    initial_radius = np.mean(np.linalg.norm(orth_v, axis=1))

    def cylinder_error(params):
        center = params[:3]
        axis_dir = params[3:6]
        axis_dir = axis_dir / np.linalg.norm(axis_dir)  # normalize
        radius = params[6]
        v = points - center
        proj = np.outer(np.dot(v, axis_dir), axis_dir)
        orth = v - proj
        dists = np.linalg.norm(orth, axis=1) - radius
        return np.sum(dists ** 2)

    # We add BOUND
    bounds = Bounds(
        [-np.inf, -np.inf, -np.inf, -1, -1, -1, 0],
        [np.inf, np.inf, np.inf, 1, 1, 1, cluster_size * 0.5]
    )

    initial_params = np.concatenate([initial_center, axis, [initial_radius]])
    result = minimize(cylinder_error, initial_params, method='L-BFGS-B', bounds=bounds)

    optimized_center = result.x[:3]
    optimized_axis = result.x[3:6] / np.linalg.norm(result.x[3:6])
    optimized_radius = result.x[6]

    # Calculate height
    v = points - optimized_center
    proj_lengths = np.abs(np.dot(v, optimized_axis))
    height = np.max(proj_lengths) * 2

    approach_angle = compute_approach_angle(optimized_axis, np.array([0, 0, 1]))
    params = {
        "center": optimized_center,
        "axis": optimized_axis,
        "radius": optimized_radius,
        "height": height,
        "approach_angle": approach_angle
    }
    error = cylinder_error(result.x) / len(points)

    return params, error

def compute_approach_angle(cylinder_axis, reference_axis=np.array([0, 0, 1])):
    """
    cylinder_axis: Normalized axis of the cylinder (3D vector)
    reference_axis: Reference axis to compare ([0,0,1] in world coordinates)
    """
    # If the axis of the cylinder is already normalized, we can do the dot multiplication directly.
    dot = np.dot(cylinder_axis, reference_axis)
    # If there are floating errors, we limit them with min/max.
    dot = max(-1.0, min(1.0, dot))
    angle = np.arccos(dot)  # In radians
    return np.degrees(angle)  # Convert it to degrees

def print_header():
    header = r"""
  ###############################################################################################################################################################################################################
  #                                                                                                                                                                                                             #
  #    \__/        \__/        \__/        \__/        \__/        \__/                            Aalborg University                  \__/        \__/        \__/        \__/        \__/        \__/         #
  #    /  \        /  \        /  \        /  \        /  \        /  \                                                                /  \        /  \        /  \        /  \        /  \        /  \         #
  #         \__/        \__/        \__/        \__/        \__/        \__/     Object Manipulation and Task Planning - Mini Project        \__/        \__/        \__/        \__/        \__/        \__/   #
  #         /  \        /  \        /  \        /  \        /  \        /  \                                                                 /  \        /  \        /  \        /  \        /  \        /  \   #
  #    \__/        \__/        \__/        \__/        \__/        \__/                               Spring 2025                      \__/        \__/        \__/        \__/        \__/        \__/         #
  #    /  \        /  \        /  \        /  \        /  \        /  \                                                                /  \        /  \        /  \        /  \        /  \        /  \         #
  #         \__/        \__/        \__/        \__/        \__/        \__/      Development of Semi-Autonomous Prosthesis Control          \__/        \__/        \__/        \__/        \__/        \__/   #
  #         /  \        /  \        /  \        /  \        /  \        /  \                     Using Computer Vision                       /  \        /  \        /  \        /  \        /  \        /  \   #
  #    \__/        \__/        \__/        \__/        \__/        \__/                                                                \__/        \__/        \__/        \__/        \__/        \__/         #
  #    /  \        /  \        /  \        /  \        /  \        /  \          Norbert Pap / Eymen Saitoğlu Gümüş / Jonas Koditek    /  \        /  \        /  \        /  \        /  \        /  \         #
  #         \__/        \__/        \__/        \__/        \__/        \__/     Simão Branco / Konstantinos Panagiotis Avgeropoulos         \__/        \__/        \__/        \__/        \__/        \__/   #
  #         /  \        /  \        /  \        /  \        /  \        /  \                         Ozan Gazi Yücel                         /  \        /  \        /  \        /  \        /  \        /  \   #
  #                                                                                                                                                                                                             #
  ###############################################################################################################################################################################################################
    """
    print(header)

def print_transition():
    transition = r"""
  ###############################################################################################################################################################################################################
  #                                                                           Waiting for new cluster to determine grab type selection                                                                          #
  ###############################################################################################################################################################################################################
    """
    print(transition)

if __name__ == '__main__':

    try:
        print_header()
        point_cloud_segmentation()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node was shut down!")
