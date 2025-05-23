#ifndef FAST_LIO_SAM_QN_MAIN_H
#define FAST_LIO_SAM_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>
///// ROS
#include <ros/ros.h>
#include <ros/package.h>              // get package_path
#include <rosbag/bag.h>               // save map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h> // pt
#include <pcl/point_cloud.h> // cloud
#include <pcl/io/pcd_io.h>   // save map
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
///// coded headers
#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"
///// GPS
#include <sensor_msgs/NavSatFix.h>
#include <gtsam/navigation/GPSFactor.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioSam
{
private:
    ///// basic params
    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;
    ///// shared data - odom and pcd
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    ///// graph and values
    bool is_initialized_ = false;
    bool loop_added_flag_ = false;     // for opt
    bool loop_added_flag_vis_ = false; // for vis
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis
    ///// visualize
    tf::TransformBroadcaster broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    ///// results
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, corrected_pcd_map_pub_, loop_detection_pub_;
    ros::Publisher realtime_pose_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_fine_aligned_pub_;
    ros::Publisher debug_intensity_seed_points_pub_, debug_noise_points_pub_, debug_map_for_clustering_pub_, debug_signboards_map_pub_, debug_remained_map_pub_, \
        debug_segmented_map_pub_;
    ros::Subscriber sub_save_flag_;
    ros::Subscriber sub_gps_;
    ros::Timer loop_timer_, vis_timer_;
    // odom, pcd sync, and save flag subscribers
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    ///// Loop closure
    std::shared_ptr<LoopClosure> loop_closure_;
    ///// GPS
    bool use_gps = false;
    std::deque<nav_msgs::Odometry> gps_odom_queue_;
    ros::Publisher pub_gps_odom_;
    GeographicLib::LocalCartesian gps_trans_;
    Eigen::MatrixXd pose_covariance_;
    float gps_cov_thres, pose_cov_thres, gps_dist_thres;
    bool use_gps_elevation = false;
    ///// denoise
    bool denoise_en = false;
    float seed_thres = 2800.0;
    float denoise_radius = 0.5;
    float cluster_seed_radius = 1.0;
    float noise_thres = 2700.0;
    int MeanK = 50;
    float StddevMulThresh = 1.0;
    pcl::PointCloud<pcl::PointXYZI> map_for_clustering;
    int map_for_clustering_size_thres = 10000;
    pcl::PointCloud<pcl::PointXYZI> segmented_map;
    pcl::PointCloud<pcl::PointXYZI> remained_map;
    pcl::PointCloud<pcl::PointXYZI> signboards_seed_map;

    pcl::PointCloud<pcl::PointXYZI> signboards_map;
    pcl::PointCloud<pcl::PointXYZI> noise_map;

    // ROS service client for triggering image saving
    ros::ServiceClient trigger_client_;

    // Variables to track the robot's position and movement
    Eigen::Vector3d last_position_;
    double distance_threshold_;

public:
    explicit FastLioSam(const ros::NodeHandle &n_private);
    ~FastLioSam();

private:
    // methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    // cb
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void add_gps_factor(const PosePcd &current_frame);
    pcl::PointCloud<pcl::PointXYZI> extract_intensity_points(const pcl::PointCloud<pcl::PointXYZI>& ori_pcd);
    pcl::PointCloud<pcl::PointXYZI> denoise_and_extract_clustering_seed_points(const pcl::PointCloud<pcl::PointXYZI>& seed_cloud, pcl::PointCloud<pcl::PointXYZI>& ori_pcd, \
                                                        pcl::PointCloud<pcl::PointXYZI>& map_for_clustering);
    pcl::PointCloud<pcl::PointXYZI> clustering_and_denoise(const pcl::PointCloud<pcl::PointXYZI> &map_for_clustering);
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                         const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr &msg);
    void loopTimerFunc(const ros::TimerEvent &event);
    void visTimerFunc(const ros::TimerEvent &event);
    void segment_map(const pcl::PointCloud<pcl::PointXYZI>& seed_map, const pcl::PointCloud<pcl::PointXYZI>& ori_map, \
        pcl::PointCloud<pcl::PointXYZI>& segmented_map, pcl::PointCloud<pcl::PointXYZI>& remained_map);
    pcl::PointCloud<pcl::PointXYZI> denoise_map(const pcl::PointCloud<pcl::PointXYZI>& seed_map, const pcl::PointCloud<pcl::PointXYZI>& segmented_map);
    void denoise_slam_map(pcl::PointCloud<pcl::PointXYZI>& slam_map);
};


#endif
