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
        # Create empty point clouds for visualization
        table_cloud = o3d.geometry.PointCloud()
        wall_cloud = o3d.geometry.PointCloud()
        # First plane segmentation
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                            ransac_n=3,
                                            num_iterations=1000)
        [a, b, c, d] = plane_model

        # Store first plane parameters
        first_plane = {'normal': [a, b, c], 'distance': d}
        # REPLACE your existing classification logic with this:
        # For a horizontal plane (table) in rotated space, the normal should be close to [0,0,-1]
        # For a vertical plane (wall) in rotated space, the normal should be perpendicular to [0,0,-1]
        
        # Calculate angle between normal and vertical axis [0,0,-1]
        vertical_axis = np.array([0, 0, -1])  # After rotation, this is "up"
        normal_vector = np.array([a, b, c])
        normal_norm = np.linalg.norm(normal_vector)
        
        # For horizontal planes (tables), the normal should be orthogonal to up direction
        # For vertical planes (walls), the normal should be parallel to up direction
        cos_angle = abs(np.dot(normal_vector, vertical_axis)) / normal_norm
        angle_with_vertical = np.arccos(cos_angle) * 180 / np.pi
        
        # ADJUSTED HORIZONTAL DETECTION:
        # A horizontal plane has its normal vector perpendicular to the vertical axis
        is_horizontal = angle_with_vertical > 70  # Within ~15 degrees of perpendicular
        
        rospy.loginfo(f"Plane normal: [{a:.4f}, {b:.4f}, {c:.4f}], angle with vertical: {angle_with_vertical:.2f}°, cos: {cos_angle:.4f}, is_horizontal: {is_horizontal}")

        if is_horizontal:
            # This is a table
            temp_plane_cloud = pcd.select_by_index(inliers)
            # Calculate area from width in X and Y direction
            extent = temp_plane_cloud.get_axis_aligned_bounding_box().get_extent()
            area = extent[0] * extent[1]  # x * y 

            if area > 0.10:
                rospy.loginfo("✅ Plane is large enough to be considered a table. Removing it.")
                table_cloud = temp_plane_cloud
                table_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for table
                remaining_cloud = pcd.select_by_index(inliers, invert=True)
                table_plane = first_plane  # Save first plane parameters
            else:
                rospy.logwarn("❌ Plane is too small to be a table. Skipping removal.")
                remaining_cloud = pcd  # don't erase keep same
        else:
            # This is a wall
            temp_plane_cloud = pcd.select_by_index(inliers)
            # Check if this is large enough to be a wall
            inlier_ratio = len(inliers) / len(np.asarray(pcd.points))
            
            if inlier_ratio > 0.3:
                rospy.loginfo("✅ Detected vertical plane (wall). Removing it.")
                wall_cloud = temp_plane_cloud
                wall_cloud.paint_uniform_color([0.0, 1.0, 0.0])  # Green for wall
                remaining_cloud = pcd.select_by_index(inliers, invert=True)
                wall_plane = first_plane  # Save first plane parameters
            else:
                rospy.logwarn("❌ Vertical plane is too small to be a wall. Skipping removal.")
                remaining_cloud = pcd

       # Try to find a second plane if we haven't found both yet
        if (table_cloud.is_empty() or wall_cloud.is_empty()) and len(np.asarray(remaining_cloud.points)) > 100:
            plane_model2, inliers2 = remaining_cloud.segment_plane(distance_threshold=0.02,
                                                            ransac_n=3,
                                                            num_iterations=1000)
            [a2, b2, c2, d2] = plane_model2
            
            # Store second plane parameters
            second_plane = {'normal': [a2, b2, c2], 'distance': d2}
            
            # Use the same logic consistently for second pass
            vertical_axis = np.array([0, 0, -1]) 
            normal_vector2 = np.array([a2, b2, c2])
            normal_norm2 = np.linalg.norm(normal_vector2)
            
            cos_angle2 = abs(np.dot(normal_vector2, vertical_axis)) / normal_norm2
            angle_with_vertical2 = np.arccos(cos_angle2) * 180 / np.pi
            
        
            is_horizontal2 = angle_with_vertical2 > 70  # Within ~15 degrees of perpendicular

            if is_horizontal2 and table_cloud.is_empty():
                # Found a table in the second pass
                temp_plane_cloud = remaining_cloud.select_by_index(inliers2)
                extent = temp_plane_cloud.get_axis_aligned_bounding_box().get_extent()
                area = extent[0] * extent[1]
                
                if area > 0.10:
                    rospy.loginfo("✅ Detected horizontal plane (table) in second pass. Removing it.")
                    table_cloud = temp_plane_cloud
                    table_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red for table
                    remaining_cloud = remaining_cloud.select_by_index(inliers2, invert=True)
                    table_plane = second_plane  # Save table plane parameters from second pass
            elif not is_horizontal2 and wall_cloud.is_empty():
                # Found a wall in the second pass
                temp_plane_cloud = remaining_cloud.select_by_index(inliers2)
                inlier_ratio = len(inliers2) / len(np.asarray(remaining_cloud.points))
                
                if inlier_ratio > 0.3:
                    rospy.loginfo("✅ Detected vertical plane (wall) in second pass. Removing it.")
                    wall_cloud = temp_plane_cloud
                    wall_cloud.paint_uniform_color([0.0, 1.0, 0.0])  # Green for wall
                    remaining_cloud = remaining_cloud.select_by_index(inliers2, invert=True)
                    wall_plane = second_plane  # Save wall plane parameters from second pass
        
        # Set the final point cloud for further processing
        pcd = remaining_cloud

        # Visualize what was removed (table and/or wall)
        if not table_cloud.is_empty():
            o3d.visualization.draw_geometries([table_cloud], window_name="❌ Removed Table Surface (RANSAC Inliers)")
        # Visualize what was removed (wall)    
        if not wall_cloud.is_empty():
            o3d.visualization.draw_geometries([wall_cloud], window_name="❌ Removed Wall Surface")
        # Visualize remaining points after all plane removal
        o3d.visualization.draw_geometries([pcd], window_name="Planar Surfaces Removed")


        # 3. Euclidean clustering
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
        z_axis = np.array([0, 0, -1])
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
    threshold_sphere = 0.025   # 25 mm
    threshold_cyl_radius = 0.025   # 25 mm
    threshold_cyl_length = 0.08   # 80 mm
    threshold_cyl_angel = 45   # 45 degrees
    margin_p = 0.002  # 2 mm
    margin_l = 0.003  # 3 mm
    reason = None
    
    if best_model == "sphere":
        if sphere_params is not None and sphere_params["radius"] > threshold_sphere:
            grasp_type = "Palmar"
            reason = "Sphere radius is larger than 25 mm"
            grasp_size = 2*sphere_params["radius"] + margin_p
        else:
            grasp_type = "Lateral" 
            reason = "Sphere radius is smaller than 25 mm"
            grasp_size = 2*sphere_params["radius"] + margin_l
    elif best_model == "cylinder":
        if cylinder_params:
            radius = cylinder_params["radius"]
            angle = cylinder_params["approach_angle"]  # Calculated in degrees
            length = cylinder_params["height"]
            if length < threshold_cyl_length:
                if radius < threshold_cyl_radius:
                    reason = "Cylinder radius is smaller than 25 mm (Tiny cylinder)"
                    grasp_type = "Lateral (angle=90 deg) from top"
                    grasp_size = 2*radius + margin_l
                else:
                    if angle > threshold_cyl_angel:
                        grasp_type = f"Palmar (angle={angle:.1f} deg) from side"
                        reason = "Cylinder length is smaller than 8cm, cylinder radius is bigger than 25 mm, approach angle is bigger than 45 degrees"
                        grasp_size = 2*radius + margin_p
                    else:
                        grasp_type = f"Lateral (angle={angle:.1f} deg) from side"
                        reason = "Cylinder length is smaller than 8cm, cylinder radius is bigger than 25 mm, approach angle is smaller than 45 degrees"
                        grasp_size = 2*radius + margin_l
            else:
                grasp_type = f"Palmar (angle={angle:.1f} deg) from side"
                reason = "Cylinder length is larger than 80 mm"
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

    approach_angle = compute_approach_angle(optimized_axis, np.array([0, 0, -1]))
    params = {
        "center": optimized_center,
        "axis": optimized_axis,
        "radius": optimized_radius,
        "height": height,
        "approach_angle": approach_angle
    }
    error = cylinder_error(result.x) / len(points)
    return params, error

def compute_approach_angle(cylinder_axis, reference_axis=np.array([0, 0, -1])):
    """
    cylinder_axis: Normalized axis of the cylinder (3D vector)
    reference_axis: Reference axis to compare ([0,0,-1] in world coordinates)
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
