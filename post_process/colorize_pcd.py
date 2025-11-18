import open3d as o3d
import numpy as np
import cv2

# --- Load point cloud ---
pcd = o3d.io.read_point_cloud("/home/kodifly/ssd3/isds_debug/slam_lidar_frame.pcd")
points = np.asarray(pcd.points)
print(f"Loaded point cloud with {points.shape[0]} points.")

# --- Camera extrinsics (LiDAR to Camera) ---
R = np.array([
    [-0.86900,  0.49480,  0.00251],
    [-0.00257,  0.00055, -1.00000],
    [-0.49480, -0.86901,  0.00079]
])
t = np.array([[0.01791], [-0.32026], [-0.22659]])
print("Extrinsic rotation matrix:\n", R)
print("Extrinsic translation vector:\n", t.flatten())

# --- Camera intrinsics ---
fx, fy, cx, cy = 1422.18129, 1421.822766, 2017.285053, 1232.094237
print(f"Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

# --- Load image ---
img = cv2.imread("/home/kodifly/ssd3/isds_debug/DA6102933_20250812141443100.jpg")
if img is None:
    raise FileNotFoundError("Image not found or path is incorrect.")
img_h, img_w = img.shape[:2]
print(f"Loaded image with shape: {img.shape}")

# --- Transform LiDAR points to camera frame ---
points_cam = (R @ points.T + t).T  # shape: (N, 3)
print("First 5 points in camera frame:\n", points_cam[:5])

# --- Project to image plane ---
x = points_cam[:, 0]
y = points_cam[:, 1]
z = points_cam[:, 2]
u = fx * (x / z) + cx
v = fy * (y / z) + cy

print("First 5 projected pixel coordinates (u, v):")
for i in range(5):
    print(f"({u[i]:.2f}, {v[i]:.2f}), z={z[i]:.2f}")

# --- Filter points in front of camera and inside image bounds ---
valid = (z > 0) & (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
print(f"Number of valid projected points: {np.sum(valid)} / {points.shape[0]}")

# --- Optional: visualize projected points on image for debugging ---
debug_img = img.copy()
for uu, vv in zip(u[valid].astype(int)[::100], v[valid].astype(int)[::100]):  # plot every 100th point
    cv2.circle(debug_img, (uu, vv), 2, (0, 0, 255), -1)
cv2.imwrite("/home/kodifly/ssd3/isds_debug/projected_points_debug.jpg", debug_img)
print("Saved debug image with projected points.")

# --- Assign color ---
colors = np.zeros_like(points)
colors[:] = [0.5, 0.5, 0.5]  # default gray

u_valid = u[valid].astype(np.int32)
v_valid = v[valid].astype(np.int32)
colors[valid] = img[v_valid, u_valid, ::-1] / 255.0  # OpenCV uses BGR

pcd.colors = o3d.utility.Vector3dVector(colors)

# --- Save or visualize ---
o3d.io.write_point_cloud("/home/kodifly/ssd3/isds_debug/focus_colorized.pcd", pcd)
print("Saved colorized point cloud to /home/kodifly/ssd3/isds_debug/focus_colorized.pcd")
# o3d.visualization.draw_geometries([pcd])