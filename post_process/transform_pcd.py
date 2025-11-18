import open3d as o3d
import numpy as np

# ——— Input Parameters ———
# Replace with your actual file paths
input_pcd_path = "/home/kodifly/ssd3/isds_debug/focus.pcd"      # Path to your input PCD file
output_pcd_path = "/home/kodifly/ssd3/isds_debug/slam_lidar_frame.pcd"  # Path to save transformed PCD

# Transformation: x, y, z, qx, qy, qz, qw
t_x = 259.39397273
t_y = -73.77794443
t_z = -8.02234251
qx = -0.04780275
qy = 0.00922145
qz = 0.99876705
qw = 0.00970778

# ——— Load the Point Cloud ———
pcd = o3d.io.read_point_cloud(input_pcd_path)
if not pcd:
    raise ValueError("Failed to load PCD file. Check the path and format.")
print(f"Loaded point cloud with {len(pcd.points)} points.")

# Check if intensity is loaded
if hasattr(pcd, 'point') and 'intensity' in pcd.point:
    print("Intensity attribute found. Number of intensities:", len(pcd.point['intensity']))
else:
    print("Warning: Intensity attribute not found in loaded point cloud.")

# ——— Build Transformation Matrix ———
# Normalize quaternion
q = np.array([qw, qx, qy, qz])
q = q / np.linalg.norm(q)
qw, qx, qy, qz = q

# Quaternion to rotation matrix
R = np.array([
    [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
    [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
    [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)]
])

# Translation vector
t = np.array([t_x, t_y, t_z])

# 4x4 transformation matrix
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = t

# ——— Apply Transformation ———
pcd.transform(T)

# ——— Save Transformed Point Cloud ———
o3d.io.write_point_cloud(output_pcd_path, pcd)
print(f"Transformed point cloud saved to {output_pcd_path}")

# Optional: Visualize
# o3d.visualization.draw_geometries([pcd], window_name="Transformed PCD")