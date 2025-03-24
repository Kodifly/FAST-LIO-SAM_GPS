import open3d as o3d
import numpy as np

# Step 1: Load the PCD file
pcd_file = "fast_lio_sam/data/lidar/pcd/2347.592237.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_file)

# Step 2: Check if intensity values exist
if not point_cloud.has_intensities():
    raise ValueError("The point cloud does not contain intensity values.")

# Extract points and intensity values
points = np.asarray(point_cloud.points)
intensities = np.asarray(point_cloud.colors)[:, 0]  # Assuming intensity is stored in the first channel of colors

# Step 3: Set intensity threshold
intensity_threshold = 0.5  # Adjust this value based on your requirements
high_intensity_indices = intensities > intensity_threshold

# Filter points based on the threshold
filtered_points = points[high_intensity_indices]
filtered_intensities = intensities[high_intensity_indices]

# Step 4: Create a new point cloud with filtered points
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# If you want to retain the intensity as color, assign it back
filtered_colors = np.zeros((filtered_points.shape[0], 3))  # RGB format
filtered_colors[:, 0] = filtered_intensities  # Assign intensity to red channel
filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

# Step 5: Save or visualize the filtered point cloud
output_pcd_file = "fast_lio_sam/data/lidar/pcd/2347.592237_filtered.pcd"
o3d.io.write_point_cloud(output_pcd_file, filtered_point_cloud)

# Visualize the filtered point cloud
o3d.visualization.draw_geometries([filtered_point_cloud])