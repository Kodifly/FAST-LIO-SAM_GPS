#include "fast_lio_sam.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <std_srvs/Trigger.h> // For the trigger service
#include <ctime>
#include <cstdio>
#include <cmath>
// -----------------------------------------------------------------------------
// Helper to convert UNIX epoch time (in seconds, ROS format) to the desired
// yyyymmddhhmmssSSS string with millisecond precision. This is used when we
// save poses in TUM format so that both the timestamps inside the file and the
// filename itself are human-readable.
static std::string unixToReadableTimestamp(double unix_ts)
{
    std::time_t t_sec = static_cast<std::time_t>(unix_ts);
    std::tm tm_res;
#ifdef _WIN32
    localtime_s(&tm_res, &t_sec);
#else
    localtime_r(&t_sec, &tm_res);
#endif
    char date_buf[32];
    std::strftime(date_buf, sizeof(date_buf), "%Y%m%d%H%M%S", &tm_res); // yyyymmddhhmmss

    // Milliseconds part (0-999)
    int millis = static_cast<int>(std::round((unix_ts - static_cast<double>(t_sec)) * 1000.0));
    if (millis == 1000) // handle edge case where rounding pushes to next second
    {
        millis = 0;
        t_sec += 1;
#ifdef _WIN32
        localtime_s(&tm_res, &t_sec);
#else
        localtime_r(&t_sec, &tm_res);
#endif
        std::strftime(date_buf, sizeof(date_buf), "%Y%m%d%H%M%S", &tm_res);
    }

    char final_buf[40];
    std::snprintf(final_buf, sizeof(final_buf), "%s%03d", date_buf, millis);
    return std::string(final_buf);
}
// Define color codes
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";

// Helper function to add colors to logs
std::string colorize(const std::string& message, const std::string& color_code) {
    return color_code + message + "\033[0m";  // \033[0m resets the color
}

FastLioSam::FastLioSam(const ros::NodeHandle &n_private):
    nh_(n_private), last_position_(Eigen::Vector3d::Zero()), distance_threshold_(1.0)
{
    ////// ROS params
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    /* basic */
    nh_.param<std::string>("/basic/map_frame", map_frame_, "map");
    nh_.param<double>("/basic/loop_update_hz", loop_update_hz, 1.0);
    nh_.param<double>("/basic/vis_hz", vis_hz, 0.5);
    /* keyframe */
    nh_.param<double>("/keyframe/keyframe_threshold", keyframe_thr_, 1.0);
    nh_.param<int>("/keyframe/nusubmap_keyframes", lc_config.num_submap_keyframes_, 5);
    /* loop */
    nh_.param<double>("/loop/loop_detection_radius", lc_config.loop_detection_radius_, 15.0);
    nh_.param<double>("/loop/loop_detection_timediff_threshold", lc_config.loop_detection_timediff_threshold_, 10.0);
    lc_config.icp_max_corr_dist_ = lc_config.loop_detection_radius_ * 1.5;
    /* icp */
    nh_.param<double>("/icp/icp_voxel_resolution", lc_config.voxel_res_, 0.3);
    nh_.param<double>("/icp/icp_score_threshold", lc_config.icp_score_threshold_, 0.3);
    /* results */
    nh_.param<double>("/result/save_voxel_resolution", voxel_res_, 0.3);
    nh_.param<bool>("/result/save_map_pcd", save_map_pcd_, false);
    nh_.param<bool>("/result/save_map_bag", save_map_bag_, false);
    nh_.param<bool>("/result/save_in_kitti_format", save_in_kitti_format_, false);
    /* GPS */
    nh_.param<bool>("/gps/use_gps", use_gps, false);
    nh_.param<bool>("/gps/use_gps_elevation", use_gps_elevation, false);
    nh_.param<float>("/gps/gps_cov_thres", gps_cov_thres, 2.0);
    nh_.param<float>("/gps/pose_cov_thres", pose_cov_thres, 0.02);
    nh_.param<float>("/gps/gps_dist_thres", gps_dist_thres, 5.0);
    /* denoise */
    nh_.param<bool>("/denoise/denoise_en", denoise_en, false);
    nh_.param<float>("/denoise/seed_thres", seed_thres, 2800.0);
    nh_.param<float>("/denoise/denoise_radius", denoise_radius, 0.5);
    nh_.param<float>("/denoise/cluster_seed_radius", cluster_seed_radius, 1.0);
    nh_.param<float>("/denoise/noise_thres", noise_thres, 2700.0);
    nh_.param<int>("/denoise/map_for_clustering_size_thres", map_for_clustering_size_thres, 10000);
    nh_.param<int>("/denoise/sor/MeanK", MeanK, 50);
    nh_.param<float>("/denoise/sor/StddevMulThresh", StddevMulThresh, 1.0);
    /* sequence name */
    nh_.param<std::string>("/result/seq_name", seq_name_, "");
    /* Loop closure */
    loop_closure_.reset(new LoopClosure(lc_config));
    /* Initialization of GTSAM */
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;
    package_path_ = ros::package::getPath("fast_lio_sam");
    /* publishers */
    odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ori_path", 10, true);
    corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
    corrected_pcd_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_map", 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true); // visualize in rviz
    loop_detection_pub_ = nh_.advertise<visualization_msgs::Marker>("/loop_detection", 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
    debug_src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10, true);
    debug_dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10, true);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned", 10, true);
    debug_intensity_seed_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/seed_intensity_points", 10, true);
    debug_noise_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/noise_points", 10, true);
    debug_map_for_clustering_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_for_clustering", 10, true);
    debug_signboards_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/denoised_map", 10, true);
    debug_remained_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/remained_map", 10, true);
    debug_segmented_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_map", 10, true);
    pub_gps_odom_ = nh_.advertise<nav_msgs::Odometry>("/gps/odom", 10);
    /* subscribers */
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "/Odometry", 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "/cloud_registered", 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&FastLioSam::odomPcdCallback, this, _1, _2));
    sub_save_flag_ = nh_.subscribe("/save_dir", 1, &FastLioSam::saveFlagCallback, this);
    sub_gps_ = nh_.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 10, &FastLioSam::gpsCallback, this);
    /* Timers */
    loop_timer_ = nh_.createTimer(ros::Duration(1 / loop_update_hz), &FastLioSam::loopTimerFunc, this);
    vis_timer_ = nh_.createTimer(ros::Duration(1 / vis_hz), &FastLioSam::visTimerFunc, this);
    ROS_INFO("Main class, starting node...");
    /* denoise */
    map_for_clustering.clear();

    // Initialize the trigger service client
    trigger_client_ = nh_.serviceClient<std_srvs::Trigger>("/save_image");
    ROS_INFO_STREAM(colorize("Trigger service client initialized: /save_image", CYAN));
}

void FastLioSam::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
{
    // ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
    if (gps_msg->status.status != 0)
    return;

    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    }

    gps_trans_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

    nav_msgs::Odometry gps_odom;
    gps_odom.header = gps_msg->header;
    gps_odom.header.frame_id = map_frame_;
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.covariance[0] = gps_msg->position_covariance[0];
    gps_odom.pose.covariance[7] = gps_msg->position_covariance[4];
    gps_odom.pose.covariance[14] = gps_msg->position_covariance[8];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    pub_gps_odom_.publish(gps_odom);
    gps_odom_queue_.push_back(gps_odom);
}

void FastLioSam::add_gps_factor(const PosePcd &current_frame)
{
    if (gps_odom_queue_.empty())
    return;

    // wait for system initialized and settles down
    if (keyframes_.empty())
        return;
    else
    {
        if ((keyframes_.front().pose_corrected_eig_.block<3, 1>(0, 3) - keyframes_.back().pose_corrected_eig_.block<3, 1>(0, 3)).norm() < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (pose_covariance_(3,3) < pose_cov_thres && pose_covariance_(4,4) < pose_cov_thres)
        return;

    // last gps position
    static PointType last_gps_point;

    while (!gps_odom_queue_.empty())
    {
        if (gps_odom_queue_.front().header.stamp.toSec() < current_frame.timestamp_ - 0.05)
        {
            // message too old
            gps_odom_queue_.pop_front();
        }
        else if (gps_odom_queue_.front().header.stamp.toSec() > current_frame.timestamp_ + 0.05)
        {
            // message too new
            break;
        }
        else
        {
            nav_msgs::Odometry this_gps = gps_odom_queue_.front();
            gps_odom_queue_.pop_front();

            // GPS too noisy, skip
            float noise_x = this_gps.pose.covariance[0];
            float noise_y = this_gps.pose.covariance[7];
            float noise_z = this_gps.pose.covariance[14];
            std::cout << "GPS noise: " << noise_x << ", " << noise_y << ", " << noise_z << std::endl;
            if (noise_x > gps_cov_thres || noise_y > gps_cov_thres)
                continue;

            float gps_x = this_gps.pose.pose.position.x;
            float gps_y = this_gps.pose.pose.position.y;
            float gps_z = this_gps.pose.pose.position.z;
            if (!use_gps_elevation)
            {
                gps_z = current_frame.pose_corrected_eig_(2,3); // use slam z
                noise_z = 0.01;
            }

            // GPS not properly initialized (0,0,0)
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;

            // Add GPS every a few meters
            PointType cur_gps_point;
            cur_gps_point.x = gps_x;
            cur_gps_point.y = gps_y;
            cur_gps_point.z = gps_z;
            float gps_point_dist = sqrt((cur_gps_point.x - last_gps_point.x) * (cur_gps_point.x - last_gps_point.x) +
                                        (cur_gps_point.y - last_gps_point.y) * (cur_gps_point.y - last_gps_point.y) +
                                        (cur_gps_point.z - last_gps_point.z) * (cur_gps_point.z - last_gps_point.z));
            if (gps_point_dist < gps_dist_thres)
                continue;
            else
                last_gps_point = cur_gps_point;

            gtsam::Vector Vector3(3);
            Vector3 << std::max(noise_x, 1.0f), std::max(noise_y, 1.0f), std::max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(current_frame.idx_, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gps_factor);

            loop_added_flag_ = true;
            break;
        }
    }
}

pcl::PointCloud<pcl::PointXYZI> FastLioSam::extract_intensity_points(const pcl::PointCloud<pcl::PointXYZI>& ori_pcd)
{
    pcl::PointCloud<pcl::PointXYZI> intensity_cloud;
    for (const auto& pt : ori_pcd)
    {
        if (pt.intensity > seed_thres)
        {
            intensity_cloud.push_back(pt);
        }
    }
    return intensity_cloud;
}

pcl::PointCloud<pcl::PointXYZI> FastLioSam::denoise_and_extract_clustering_seed_points(const pcl::PointCloud<pcl::PointXYZI>& seed_cloud, pcl::PointCloud<pcl::PointXYZI>& ori_pcd, \
                                                                    pcl::PointCloud<pcl::PointXYZI>& map_for_clustering)
{
    pcl::PointCloud<pcl::PointXYZI> noise_cloud;
    pcl::PointCloud<pcl::PointXYZI> ori_cloud_label = ori_pcd;
    pcl::KdTreeFLANN<pcl::PointXYZI> denoise_kdtree, cluster_seed_kdtree;
    denoise_kdtree.setInputCloud(ori_cloud_label.makeShared());
    cluster_seed_kdtree.setInputCloud(ori_cloud_label.makeShared());
    std::vector<int> denoise_id, cluster_seed_id;
    std::vector<float> denoise_dist, cluster_seed_dist;

    for (const auto& pt : seed_cloud)
    {
        // intensity-based denoising
        if (denoise_kdtree.radiusSearch(pt, denoise_radius, denoise_id, denoise_dist) > 0)
        {
            for (uint j = 0; j < denoise_id.size(); j++)
            {
                if (ori_cloud_label.points[denoise_id[j]].intensity == 0.0) // already labeled as noise
                    continue;
                if (ori_cloud_label.points[denoise_id[j]].intensity < noise_thres)
                {
                    noise_cloud.push_back(ori_cloud_label.points[denoise_id[j]]);
                    ori_cloud_label.points[denoise_id[j]].intensity = 0.0;
                }
            }
        }

        // extract clustering seed points
        if (cluster_seed_kdtree.radiusSearch(pt, cluster_seed_radius, cluster_seed_id, cluster_seed_dist) > 0)
        {
            for (uint j = 0; j < cluster_seed_id.size(); j++)
            {
                if (ori_cloud_label.points[cluster_seed_id[j]].intensity != 0.0)
                {
                    map_for_clustering.push_back(ori_cloud_label.points[cluster_seed_id[j]]);
                    ori_cloud_label.points[cluster_seed_id[j]].intensity = 0.0;
                }
            }
        }
    }
    std::cout << "Map for clustering size: " << map_for_clustering.size() << std::endl;

    // replace original pcd with remained points
    pcl::PointCloud<pcl::PointXYZI> remained_cloud;
    for (const auto& pt : ori_cloud_label)
    {
        if (pt.intensity != 0.0)
        {
            remained_cloud.push_back(pt);
        }
    }
    ori_pcd.clear();
    ori_pcd = remained_cloud;
    return noise_cloud;
}

pcl::PointCloud<pcl::PointXYZI> FastLioSam::clustering_and_denoise(const pcl::PointCloud<pcl::PointXYZI> &map_for_clustering)
{
    // Apply Statistical Outlier Removal (SOR)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(map_for_clustering.makeShared());
    sor.setMeanK(MeanK); // Number of nearest neighbors to analyze
    sor.setStddevMulThresh(StddevMulThresh); // Threshold for standard deviation
    sor.filter(*filtered_cloud);
    std::cout << "before SOR: " << map_for_clustering.size() << ", after SOR: " << filtered_cloud->size() << std::endl;

    // pcl::PointCloud<pcl::PointXYZI> denoised_map;
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    // kdtree->setInputCloud(map_for_clustering.makeShared());
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setClusterTolerance(0.5);
    // ec.setMinClusterSize(10);
    // ec.setMaxClusterSize(25000);
    // ec.setSearchMethod(kdtree);
    // ec.setInputCloud(map_for_clustering.makeShared());
    // ec.extract(cluster_indices);
    // for (const auto &cluster : cluster_indices)
    // {
    //     if (cluster.indices.size() > 100)
    //     {
    //         for (const auto &idx : cluster.indices)
    //         {
    //             denoised_map.push_back(map_for_clustering.points[idx]);
    //         }
    //     }
    // }
    return *filtered_cloud;
}

void FastLioSam::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                                 const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;                              // to calculate delta
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not, pcd transformed from world frame to LiDAR frame
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    {
        //// 1. realtime pose = last corrected odom * delta (last -> current)
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_; // ?
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        realtime_pose_pub_.publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        broadcaster_.sendTransform(tf::StampedTransform(poseEigToROSTf(current_frame_.pose_corrected_eig_),
                                                        ros::Time::now(),
                                                        map_frame_,
                                                        "robot"));
    }
    pcl::PointCloud<pcl::PointXYZI> ori_cloud = transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_);
    corrected_current_pcd_pub_.publish(pclToPclRos(ori_cloud, map_frame_));

    // Get the current position
    Eigen::Vector3d current_position = current_frame_.pose_corrected_eig_.block<3, 1>(0, 3);
    // Check if the robot has moved more than 1 meter
    double distance_moved = (current_position - last_position_).norm();
    if (distance_moved >= distance_threshold_) {
        // Call the trigger service
        std_srvs::Trigger srv;
        if (trigger_client_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO_STREAM(colorize("Trigger service called successfully: " + srv.response.message, GREEN));
            } else {
                ROS_WARN_STREAM(colorize("Trigger service failed: " + srv.response.message, YELLOW));
            }
        } else {
            ROS_INFO_STREAM(colorize("Failed to call trigger service.", CYAN));
        }

        // Update the last recorded position
        last_position_ = current_position;
    }

    std::cout << colorize("Processing LiDAR frame at time: " + std::to_string(current_frame_.timestamp_), BLUE) << std::endl;

    // save ori_cloud
    // ori_cloud.width = ori_cloud.size();
    // ori_cloud.height = 1;
    // std::string filename = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/ouster_scan/deskewed_pcd/" + 
    // std::to_string(pcd_msg->header.stamp.toSec()) + ".pcd";
    // pcl::io::savePCDFile(filename, ori_cloud);
    // ROS_INFO_STREAM(colorize("Saved LiDAR point cloud: " + filename, "\033[36m"));

    // if (denoise_en)
    // {
    //     auto intensity_seed_points = extract_intensity_points(ori_cloud);
    //     debug_intensity_seed_points_pub_.publish(pclToPclRos(intensity_seed_points, map_frame_));
        
    //     auto noise_points = denoise_and_extract_clustering_seed_points(intensity_seed_points, ori_cloud, map_for_clustering);
    //     debug_noise_points_pub_.publish(pclToPclRos(noise_points, map_frame_));
    //     debug_map_for_clustering_pub_.publish(pclToPclRos(map_for_clustering, map_frame_));

    //     // replace current_frame_.pcd_ with remained points
    //     current_frame_.pcd_ = transformPcd_inverse(ori_cloud, current_frame_.pose_corrected_eig_);
    //     debug_remained_map_pub_.publish(pclToPclRos(current_frame_.pcd_, map_frame_));

    //     if (map_for_clustering.size() > map_for_clustering_size_thres)
    //     {
    //         signboards_map += clustering_and_denoise(map_for_clustering);
    //         debug_signboards_map_pub_.publish(pclToPclRos(signboards_map, map_frame_));
    //         map_for_clustering.clear();
    //     }
    // }

    if (!is_initialized_) //// init only once
    {
        // others
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        // graph
        // auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished(); // rad*rad,
        //                                                                                             // meter*meter
        // gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        // gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        // init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        is_initialized_ = true;
    }
    else
    {
        std::cout << colorize("Checking for keyframe...", MAGENTA) << std::endl;
        //// 2. check if keyframe
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
        //     // 2-3. if so, add to graph
        //     auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        //     gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        //     gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
        //     gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
        //     {
        //         std::lock_guard<std::mutex> lock(graph_mutex_);
        //         gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
        //                                                             current_keyframe_idx_,
        //                                                             pose_from.between(pose_to),
        //                                                             odom_noise));
        //         init_esti_.insert(current_keyframe_idx_, pose_to);
        //     }
        //     // 2-4, if so, add gps factor
        //     if (use_gps)
        //     {
        //         add_gps_factor(current_frame_);
        //     }
            current_keyframe_idx_++;

            //// 3. vis
            high_resolution_clock::time_point t3 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }

            // std::cout << colorize("Optimizing graph...", CYAN) << std::endl;
        //     //// 4. optimize with graph
        //     high_resolution_clock::time_point t4 = high_resolution_clock::now();
        //     // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
        //     {
        //         std::lock_guard<std::mutex> lock(graph_mutex_);
        //         std::cout << colorize("Updating ISAM2 with " + std::to_string(gtsam_graph_.size()) + " new factors...", YELLOW) << std::endl;
        //         isam_handler_->update(gtsam_graph_, init_esti_);
        //         std::cout << colorize("Graph size: " + std::to_string(isam_handler_->getFactorsUnsafe().size()) + " factors, " +
        //                                 std::to_string(isam_handler_->getLinearizationPoint().size()) + " values.", YELLOW) << std::endl;
        //         isam_handler_->update();
        //         if (loop_added_flag_) // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
        //         {
        //             isam_handler_->update();
        //             isam_handler_->update();
        //             isam_handler_->update();
        //         }
        //         std::cout << colorize("Optimization done.", GREEN) << std::endl;
        //         gtsam_graph_.resize(0);
        //         init_esti_.clear();
        //     }

        //     std::cout << colorize("Handling corrected results...", CYAN) << std::endl;
        //     //// 5. handle corrected results
        //     // get corrected poses and reset odom delta (for realtime pose pub)
        //     high_resolution_clock::time_point t5 = high_resolution_clock::now();
        //     {
        //         std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        //         corrected_esti_ = isam_handler_->calculateEstimate();
        //         last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
        //         pose_covariance_ = isam_handler_->marginalCovariance(corrected_esti_.size() - 1);
        //         // std::cout << "Pose covariance: " << std::endl << pose_covariance_ << std::endl << std::endl;
        //         odom_delta_ = Eigen::Matrix4d::Identity();
        //     }
        //     // correct poses in keyframes
        //     if (loop_added_flag_)
        //     {
        //         std::lock_guard<std::mutex> lock(keyframes_mutex_);
        //         for (size_t i = 0; i < corrected_esti_.size(); ++i)
        //         {
        //             keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
        //         }
        //         loop_added_flag_ = false;
        //     }
        //     high_resolution_clock::time_point t6 = high_resolution_clock::now();

        //     ROS_INFO("real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms",
        //              duration_cast<microseconds>(t2 - t1).count() / 1e3,
        //              duration_cast<microseconds>(t3 - t2).count() / 1e3,
        //              duration_cast<microseconds>(t4 - t3).count() / 1e3,
        //              duration_cast<microseconds>(t5 - t4).count() / 1e3,
        //              duration_cast<microseconds>(t6 - t5).count() / 1e3,
        //              duration_cast<microseconds>(t6 - t1).count() / 1e3);
        }
    }

    // map_for_clustering += ori_cloud;
    // signboards_seed_map += extract_intensity_points(ori_cloud);
    // debug_intensity_seed_points_pub_.publish(pclToPclRos(signboards_seed_map, map_frame_));
    // ROS_INFO("\033[1;32mMap for segmentation size: %.3i\033[0m", map_for_clustering.size());

    // if (map_for_clustering.size() > map_for_clustering_size_thres)
    // {
    //     segment_map(signboards_seed_map, map_for_clustering, segmented_map, remained_map);
    //     map_for_clustering.clear();

    //     debug_segmented_map_pub_.publish(pclToPclRos(segmented_map, map_frame_));
    //     debug_remained_map_pub_.publish(pclToPclRos(remained_map, map_frame_));

    //     // denoise
    //     signboards_map = denoise_map(signboards_seed_map, segmented_map);
    //     debug_signboards_map_pub_.publish(pclToPclRos(signboards_map, map_frame_));

    //     segmented_map.clear();
    //     remained_map.clear();
    //     signboards_map.clear();
    //     signboards_seed_map.clear();
    // }    

    return;
}

pcl::PointCloud<pcl::PointXYZI> FastLioSam::denoise_map(const pcl::PointCloud<pcl::PointXYZI>& seed_map, const pcl::PointCloud<pcl::PointXYZI>& segmented_map)
{
    pcl::PointCloud<pcl::PointXYZI> segmented_map_copy = segmented_map;
    // pcl::KdTreeFLANN<pcl::PointXYZI> denoise_kdtree;
    // denoise_kdtree.setInputCloud(segmented_map_copy.makeShared());
    // std::vector<int> denoise_id;
    // std::vector<float> denoise_dist;

    // // intensity_based denoising
    // for (const auto& pt : seed_map)
    // {
    //     if (denoise_kdtree.radiusSearch(pt, denoise_radius, denoise_id, denoise_dist) > 0)
    //     {
    //         for (uint j = 0; j < denoise_id.size(); j++)
    //         {
    //             if (segmented_map_copy.points[denoise_id[j]].intensity == 0.0) // already labeled as noise
    //                 continue;
    //             if (segmented_map_copy.points[denoise_id[j]].intensity < noise_thres)
    //             {
    //                 noise_map.push_back(segmented_map_copy.points[denoise_id[j]]);
    //                 segmented_map_copy.points[denoise_id[j]].intensity = 0.0;
    //             }
    //         }
    //     }
    // }

    // // extract clustering seed points
    // pcl::PointCloud<pcl::PointXYZI> segment_map_intensity_denoised;
    // for (const auto& pt : segmented_map_copy)
    // {
    //     if (pt.intensity != 0.0)
    //     {
    //         segment_map_intensity_denoised.push_back(pt);
    //     }
    // }

    // apply SOR
    // pcl::PointCloud<pcl::PointXYZI> denoised_map = clustering_and_denoise(segment_map_intensity_denoised);
    pcl::PointCloud<pcl::PointXYZI> sor_denoised_map = clustering_and_denoise(segmented_map_copy);

    // intensity_based denoising
    pcl::KdTreeFLANN<pcl::PointXYZI> denoise_kdtree;
    denoise_kdtree.setInputCloud(sor_denoised_map.makeShared());
    std::vector<int> denoise_id;
    std::vector<float> denoise_dist;
    for (const auto& pt : seed_map)
    {
        if (denoise_kdtree.radiusSearch(pt, denoise_radius, denoise_id, denoise_dist) > 0)
        {
            for (uint j = 0; j < denoise_id.size(); j++)
            {
                if (sor_denoised_map.points[denoise_id[j]].intensity == 0.0) // already labeled as noise
                    continue;
                if (sor_denoised_map.points[denoise_id[j]].intensity < noise_thres)
                {
                    sor_denoised_map.points[denoise_id[j]].intensity = 0.0;
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI> denoised_map;
    for (const auto& pt : sor_denoised_map)
    {
        if (pt.intensity != 0.0)
        {
            denoised_map.push_back(pt);
        }
    }

    return denoised_map;
}

void FastLioSam::segment_map(const pcl::PointCloud<pcl::PointXYZI>& seed_map, const pcl::PointCloud<pcl::PointXYZI>& ori_map, \
                                pcl::PointCloud<pcl::PointXYZI>& segmented_map, pcl::PointCloud<pcl::PointXYZI>& remained_map)
{
    pcl::PointCloud<pcl::PointXYZI> ori_map_copy = ori_map;
    pcl::KdTreeFLANN<pcl::PointXYZI> segment_kdtree;
    segment_kdtree.setInputCloud(ori_map_copy.makeShared());
    std::vector<int> segment_id;
    std::vector<float> segment_dist;

    for (const auto& pt : seed_map)
    {
        segmented_map.push_back(pt);
        if (segment_kdtree.radiusSearch(pt, cluster_seed_radius, segment_id, segment_dist) > 0)
        {
            for (uint j = 0; j < segment_id.size(); j++)
            {
                if (ori_map_copy.points[segment_id[j]].intensity == 0.0)
                    continue;
                segmented_map.push_back(ori_map_copy.points[segment_id[j]]);
                ori_map_copy.points[segment_id[j]].intensity = 0.0;
            }
        }
    }
    // save remained points
    for (const auto& pt : ori_map_copy)
    {
        if (pt.intensity != 0.0)
        {
            remained_map.push_back(pt);
        }
    }
    return;
}

void FastLioSam::loopTimerFunc(const ros::TimerEvent &event)
{
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_)
    {
        return;
    }
    latest_keyframe.processed_ = true;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const int closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_);
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    debug_src_pub_.publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_.publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));

    ROS_INFO("loop: %.1f", duration_cast<microseconds>(t2 - t1).count() / 1e3);
    return;
}

void FastLioSam::visTimerFunc(const ros::TimerEvent &event)
{
    if (!is_initialized_)
    {
        return;
    }

    high_resolution_clock::time_point tv1 = high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_.publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_.publish(pclToPclRos(odoms_, map_frame_));
        path_pub_.publish(odom_path_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() > 0) // save time, only once
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_.publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() == 0)
    {
        global_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point tv2 = high_resolution_clock::now();
    // ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2 - tv1).count() / 1e3);
    return;
}

void FastLioSam::saveFlagCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;
    std::cout << "Save directory: " << save_dir << std::endl;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        ROS_INFO("\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");

        // Determine start & end time strings for the filename
        std::string start_time_str = unixToReadableTimestamp(keyframes_.front().timestamp_);
        std::string end_time_str   = unixToReadableTimestamp(keyframes_.back().timestamp_);
        std::string tum_file_path  = seq_directory + "/slam_poses_" + start_time_str + "_" + end_time_str + ".txt";

        std::ofstream tum_pose_file(tum_file_path);
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                // Save the point cloud
                // std::stringstream ss_;
                // ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                // ROS_INFO("Saving %s...", ss_.str().c_str());
                // pcl::io::savePCDFileBinary<PointType>(ss_.str(), keyframes_[i].pcd_);

                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                // const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_eig_);
                // Convert ROS timestamp to human-readable format
                std::string ts_readable = unixToReadableTimestamp(keyframes_[i].timestamp_);

                tum_pose_file << ts_readable << " "
                            //   << keyframes_[i].timestamp_ << " "
                              << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        ROS_INFO("\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    if (save_map_bag_)
    {
        rosbag::Bag bag;
        bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    }

    if (save_map_pcd_)
    {
        // Save full map as a single PCD file (no chunking)
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        ROS_INFO("\033[32;1mStart to save full map (single PCD)\033[0m");

        if (!std::filesystem::exists(seq_directory))
        {
            std::filesystem::create_directories(seq_directory);
        }

        pcl::PointCloud<PointType>::Ptr full_map(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (keyframes_.empty())
            {
                ROS_WARN("No keyframes available to save.");
            }
            else
            {
                // Reserve approximate total size
                size_t approx_per_kf = keyframes_[0].pcd_.size();
                full_map->reserve(approx_per_kf * keyframes_.size());
                for (size_t i = 0; i < keyframes_.size(); ++i)
                {
                    // use corrected pose if available
                    // *full_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
                    *full_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_eig_);
                }
            }
        }

        const std::string out_path = seq_directory + "/" + seq_name_ + "_map.pcd";
        if (full_map->empty())
        {
            ROS_WARN("Full map is empty; saving empty PCD to %s", out_path.c_str());
            pcl::PointCloud<PointType> empty;
            pcl::io::savePCDFileBinary<PointType>(out_path, empty);
        }
        else
        {
            // Optional voxelization to reduce size
            // const auto &voxelized_map = voxelizePcd(full_map, voxel_res_);
            // pcl::io::savePCDFileBinary<PointType>(out_path, *voxelized_map);
            pcl::io::savePCDFileBinary<PointType>(out_path, *full_map);
            ROS_INFO("\033[32;1mSaved full map %s (points: %zu)\033[0m", out_path.c_str(), full_map->size());
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        ROS_INFO("\033[36;1mMap saved: %.1fms\033[0m", duration_cast<microseconds>(t2 - t1).count() / 1e3);
    }
}

void FastLioSam::denoise_slam_map(pcl::PointCloud<pcl::PointXYZI>& slam_map)
{
    // record time for each process
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    // extract high intensity points as seed points
    pcl::PointCloud<pcl::PointXYZI> seed_points;
    for (const auto& pt : slam_map)
    {
        if (pt.intensity > seed_thres)
        {
            seed_points.push_back(pt);
        }
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    ROS_INFO("\033[32;1mSeed points extracted: %.1fms\033[0m", duration_cast<microseconds>(t2 - t1).count() / 1e3);

    // segment the slam map
    pcl::KdTreeFLANN<pcl::PointXYZI> segment_kdtree;
    segment_kdtree.setInputCloud(slam_map.makeShared());
    std::vector<int> segment_id;
    std::vector<float> segment_dist;
    pcl::PointCloud<pcl::PointXYZI> segmented_map;
    // pcl::PointIndices indices_to_remain;
    for (const auto& pt : seed_points)
    {
        // segmented_map.push_back(pt);
        if (segment_kdtree.radiusSearch(pt, cluster_seed_radius, segment_id, segment_dist) > 0)
        {
            for (uint j = 0; j < segment_id.size(); j++)
            {
                if (slam_map.points[segment_id[j]].intensity == 0.0)
                    continue;
                segmented_map.push_back(slam_map.points[segment_id[j]]);
                // indices_to_remain.indices.push_back(segment_id[j]);
                slam_map.points[segment_id[j]].intensity = 0.0;
            }
        }
    }
    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    ROS_INFO("\033[32;1mMap segmented: %.1fms\033[0m", duration_cast<microseconds>(t3 - t2).count() / 1e3);

    // save remained points
    pcl::PointCloud<pcl::PointXYZI> remained_map;
    for (const auto& pt : slam_map)
    {
        if (pt.intensity != 0.0)
        {
            remained_map.push_back(pt);
        }
    }
    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setInputCloud(slam_map.makeShared());
    // extract.setIndices(boost::make_shared<const pcl::PointIndices>(indices_to_remain));
    // extract.setNegative(true);
    // extract.filter(slam_map);
    high_resolution_clock::time_point t4 = high_resolution_clock::now();
    ROS_INFO("\033[32;1mRemained points saved: %.1fms\033[0m", duration_cast<microseconds>(t4 - t3).count() / 1e3);

    // denoising
    pcl::PointCloud<pcl::PointXYZI> denoised_map = denoise_map(seed_points, segmented_map);
    slam_map.clear();
    slam_map = denoised_map + remained_map;
    // slam_map += denoised_map;
    high_resolution_clock::time_point t5 = high_resolution_clock::now();
    ROS_INFO("\033[32;1mMap denoised: %.1fms\033[0m", duration_cast<microseconds>(t5 - t4).count() / 1e3);

    return;    
}

FastLioSam::~FastLioSam()
{
    // save map
    /* if (save_map_bag_)
    {
        rosbag::Bag bag;
        bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileBinary<PointType>(package_path_ + "/result.pcd", *voxelized_map);
        ROS_INFO("\033[32;1mResult saved in .pcd format!!!\033[0m");
    } */
}

void FastLioSam::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::Marker FastLioSam::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::Marker edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

bool FastLioSam::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}
