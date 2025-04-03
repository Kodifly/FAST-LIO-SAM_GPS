#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// Function to project 3D points to 2D image plane
std::vector<std::pair<cv::Point2f, int>> projectPointsToImagePlane(
    const std::vector<Eigen::Vector4f>& lidar_points_with_range,
    const Eigen::Matrix3f& intrinsic_matrix,
    const Eigen::Matrix4f& extrinsic_matrix) {
    
    std::vector<std::pair<cv::Point2f, int>> projected_points_with_cluster;
    for (const auto& point : lidar_points_with_range) {
        // Convert point to homogeneous coordinates (x, y, z, 1)
        Eigen::Vector4f point_homogeneous(point(0), point(1), point(2), 1.0);

        // Apply extrinsic transformation (LiDAR to Camera)
        Eigen::Vector4f transformed_point = extrinsic_matrix * point_homogeneous;

        // Project to 2D using intrinsic matrix
        Eigen::Vector3f camera_coords(transformed_point(0), transformed_point(1), transformed_point(2));
        Eigen::Vector3f projected = intrinsic_matrix * camera_coords;

        // Normalize by z-coordinate
        if (projected(2) > 0) { // Ensure point is in front of the camera
            float x = projected(0) / projected(2);
            float y = projected(1) / projected(2);
            projected_points_with_cluster.emplace_back(cv::Point2f(x, y), static_cast<int>(point(3))); // Pair of (point, cluster_id)
        }
    }
    return projected_points_with_cluster;
}

cv::Scalar getClusterColor(int cluster_id) {
    // Define a fixed set of colors for clusters
    static std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(0, 0, 255),   // Red
        cv::Scalar(255, 255, 0), // Yellow
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255), // Cyan
        cv::Scalar(128, 0, 0),   // Dark Blue
        cv::Scalar(0, 128, 0),   // Dark Green
        cv::Scalar(0, 0, 128),   // Dark Red
        cv::Scalar(128, 128, 0)  // Olive
    };

    return colors[cluster_id % colors.size()];
}

int main() {
    // Load PCD file with XYZ information
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/ouster_scan/pcd/2638.009397368.pcd", *cloud) == -1) {
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/kodifly/workspaces/fastliosam_ws/src/FAST-LIO-SAM/third_party/FAST_LIO/PCD/scans/2363.493064.pcd", *cloud) == -1) {
        std::cerr << "Couldn't read file lidar.pcd" << std::endl;
        return -1;
    }

    // Define intrinsic matrix (fx, fy, cx, cy)
    Eigen::Matrix3f intrinsic_matrix;
    intrinsic_matrix << 1417.541654, 0, 2089.904695,
                        0, 1418.002061, 1235.591032,
                        0, 0, 1;

    // Define extrinsic matrix (LiDAR to Camera)
    Eigen::Matrix4f extrinsic_matrix;
    extrinsic_matrix << 0.71008, -0.70410, 0.00566, -0.00341,
                        0.00446, -0.00355, -0.99998, -0.04140,
                        0.70411, 0.71009, 0.00062, -0.00149,
                        0, 0, 0, 1;

    // Convert PCL point cloud to Eigen vectors
    std::vector<Eigen::Vector4f> lidar_points;
    for (const auto& point : cloud->points) {
        lidar_points.emplace_back(Eigen::Vector4f(point.x, point.y, point.z, 1.0));
    }

    // Perform K-Means clustering
    int num_clusters = 5; // Number of clusters (adjust as needed)
    cv::Mat points_mat(lidar_points.size(), 3, CV_32F);
    for (size_t i = 0; i < lidar_points.size(); ++i) {
        points_mat.at<float>(i, 0) = lidar_points[i](0); // X
        points_mat.at<float>(i, 1) = lidar_points[i](1); // Y
        points_mat.at<float>(i, 2) = lidar_points[i](2); // Z
    }

    cv::Mat labels, centers;
    cv::kmeans(points_mat, num_clusters, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1), 3, cv::KMEANS_PP_CENTERS, centers);

    // Add cluster IDs to the points
    std::vector<Eigen::Vector4f> lidar_points_with_cluster;
    for (size_t i = 0; i < lidar_points.size(); ++i) {
        int cluster_id = labels.at<int>(i);
        lidar_points_with_cluster.emplace_back(Eigen::Vector4f(lidar_points[i](0), lidar_points[i](1), lidar_points[i](2), cluster_id));
    }

    // Project points to image plane
    std::vector<std::pair<cv::Point2f, int>> projected_points_with_cluster =
        projectPointsToImagePlane(lidar_points_with_cluster, intrinsic_matrix, extrinsic_matrix);

    // Load camera image
    cv::Mat image = cv::imread("/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/undistorted_image_8_params/2637.909348176.jpg");
    // cv::Mat image = cv::imread("/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/fisheye_image/2363.493064304.jpg");
    if (image.empty()) {
        std::cerr << "Couldn't read image file camera_image.png" << std::endl;
        return -1;
    }

    // Draw projected points on the image with cluster-based colorization
    for (const auto& [point, cluster_id] : projected_points_with_cluster) {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows) {
            // Get color based on cluster ID
            // cv::Scalar color = getClusterColor(cluster_id);
            // green
            cv::Scalar color = cv::Scalar(0, 255, 0);

            // Draw the point with a larger radius (e.g., 5 pixels)
            int point_size = 3; // Adjust the size here
            cv::circle(image, point, point_size, color, -1); // Filled circle
        }
    }

    // Define window size
    int window_width = 1280;  // Desired width
    int window_height = 720; // Desired height

    // Create a named window with the specified size
    cv::namedWindow("Projected Points with Cluster-Based Colorization", cv::WINDOW_NORMAL); // Allows resizing
    cv::resizeWindow("Projected Points with Cluster-Based Colorization", window_width, window_height);

    // Display the result
    cv::imshow("Projected Points with Cluster-Based Colorization", image);
    cv::waitKey(0);

    // Save the result
    cv::imwrite("fisheye_image_projected_result_cluster_colorized.png", image);

    return 0;
}