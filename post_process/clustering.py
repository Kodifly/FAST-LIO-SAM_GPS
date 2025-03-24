import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt

# Step 1: Load the Point Cloud Data
def load_point_cloud(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    print(f"Loaded point cloud with {len(pcd.points)} points.")
    return pcd

# Step 2.1: Remove Noise (Statistical Outlier Removal)
def remove_noise(pcd, nb_neighbors=20, std_ratio=2.0):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pcd_filtered = pcd.select_by_index(ind)
    print(f"After noise removal: {len(pcd_filtered.points)} points remaining.")
    return pcd_filtered

# Step 2.2: Ground Plane Segmentation (RANSAC)
def segment_ground_plane(pcd, distance_threshold=0.2, ransac_n=3, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    inlier_cloud = pcd.select_by_index(inliers)  # Ground plane
    outlier_cloud = pcd.select_by_index(inliers, invert=True)  # Non-ground points
    print(f"After ground plane removal: {len(outlier_cloud.points)} points remaining.")
    return outlier_cloud

# Step 3: Clustering Using DBSCAN
def cluster_with_dbscan(points, eps=0.5, min_samples=10):
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f"Number of clusters found by DBSCAN: {n_clusters}")
    return labels

# Step 4: Clustering Using Euclidean Clustering (KDTree)
def cluster_with_euclidean(points, cluster_tolerance=0.5, min_cluster_size=10, max_cluster_size=10000):
    tree = KDTree(points)
    clusters = []
    processed = np.zeros(len(points), dtype=bool)

    for i in range(len(points)):
        if processed[i]:
            continue

        indices = tree.query_radius([points[i]], r=cluster_tolerance)[0]
        if len(indices) < min_cluster_size or len(indices) > max_cluster_size:
            processed[indices] = True
            continue

        cluster = set(indices)
        queue = list(indices)

        while queue:
            idx = queue.pop(0)
            if not processed[idx]:
                processed[idx] = True
                neighbors = tree.query_radius([points[idx]], r=cluster_tolerance)[0]
                for neighbor in neighbors:
                    if not processed[neighbor]:
                        queue.append(neighbor)
                        cluster.add(neighbor)

        clusters.append(list(cluster))

    print(f"Number of clusters found by Euclidean Clustering: {len(clusters)}")
    return clusters

# Step 5: Post-Processing (Bounding Boxes)
def add_bounding_boxes(pcd, clusters):
    cluster_clouds = [pcd.select_by_index(cluster_indices) for cluster_indices in clusters]
    bounding_boxes = [cloud.get_axis_aligned_bounding_box() for cloud in cluster_clouds]
    return bounding_boxes

# Main Program
if __name__ == "__main__":
    # File path to your PCD file
    file_path = "fast_lio_sam/data/lidar/pcd/2347.592237.pcd"

    # Step 1: Load Point Cloud
    pcd = load_point_cloud(file_path)

    # Step 2.1: Remove Noise (Optional)
    pcd_filtered = remove_noise(pcd, nb_neighbors=20, std_ratio=2.0)

    # Step 2.2: Ground Plane Segmentation (Optional)
    non_ground_pcd = segment_ground_plane(pcd_filtered, distance_threshold=0.2)

    # Convert the point cloud to a numpy array
    points = np.asarray(non_ground_pcd.points)

    # Step 3: Choose Clustering Method
    clustering_method = "EUCLIDEAN"  # Change to "EUCLIDEAN" for Euclidean clustering

    if clustering_method == "DBSCAN":
        # Step 3.1: Cluster Using DBSCAN
        labels = cluster_with_dbscan(points, eps=0.5, min_samples=10)

        # Assign colors to each cluster
        colors = plt.get_cmap("tab20")(labels / (len(set(labels)) - 1))
        colors[labels < 0] = 0  # Set noise points to black
        non_ground_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # Visualize the clustered point cloud
        o3d.visualization.draw_geometries([non_ground_pcd])

    elif clustering_method == "EUCLIDEAN":
        # Step 4: Cluster Using Euclidean Clustering
        clusters = cluster_with_euclidean(points, cluster_tolerance=0.5, min_cluster_size=10, max_cluster_size=10000)

        # Step 5: Add Bounding Boxes Around Clusters
        bounding_boxes = add_bounding_boxes(non_ground_pcd, clusters)

        # Visualize the clusters with bounding boxes
        o3d.visualization.draw_geometries([non_ground_pcd] + bounding_boxes)
        # o3d.visualization.draw_geometries([non_ground_pcd])

    else:
        print("Invalid clustering method selected. Please choose 'DBSCAN' or 'EUCLIDEAN'.")