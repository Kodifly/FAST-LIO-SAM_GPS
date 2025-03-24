#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

// Helper function to add colors to logs
std::string colorize(const std::string& message, const std::string& color_code) {
    return color_code + message + "\033[0m";  // \033[0m resets the color
}

// Define color codes
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";

class SensorRecorder {
public:
    SensorRecorder(ros::NodeHandle& nh, const std::string& base_dir)
        : nh_(nh), base_dir_(base_dir) {
        
        // Delete existing data if directories exist
        try {
            if (boost::filesystem::exists(base_dir_)) {
                ROS_WARN_STREAM(colorize("Deleting existing data in: " + base_dir_, YELLOW));
                boost::filesystem::remove_all(base_dir_);
            }
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_FATAL_STREAM(colorize("Filesystem error while deleting existing data: " + std::string(e.what()), RED));
            throw;
        }

        // Create directories
        image_dir_ = base_dir_ + "/images";
        lidar_dir_ = base_dir_ + "/lidar";
        lidar_bin_dir_ = lidar_dir_ + "/bin";
        lidar_pcd_dir_ = lidar_dir_ + "/pcd";
        try {
            if (!boost::filesystem::create_directories(image_dir_)) {
                ROS_WARN_STREAM(colorize("Image directory already exists or failed to create: " + image_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created image directory: " + image_dir_, GREEN));
            }

            if (!boost::filesystem::create_directories(lidar_bin_dir_)) {
                ROS_WARN_STREAM(colorize("LiDAR bin directory already exists or failed to create: " + lidar_bin_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created bin LiDAR directory: " + lidar_bin_dir_, GREEN));
            }
            if (!boost::filesystem::create_directories(lidar_pcd_dir_)) {
                ROS_WARN_STREAM(colorize("LiDAR pcd directory already exists or failed to create: " + lidar_pcd_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created pcd LiDAR directory: " + lidar_pcd_dir_, GREEN));
            }
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_FATAL_STREAM(colorize("Filesystem error while creating directories: " + std::string(e.what()), RED));
            throw;
        }

        // Open IMU and GNSS files
        imu_file_.open(base_dir_ + "/imu.txt", std::ios::app);
        gnss_file_.open(base_dir_ + "/gnss.txt", std::ios::app);

        if (!imu_file_.is_open()) {
            ROS_FATAL_STREAM(colorize("Failed to open IMU file for writing: " + base_dir_ + "/imu.txt", RED));
            throw std::runtime_error("IMU file open error");
        }
        ROS_INFO_STREAM(colorize("Opened IMU file for writing: " + base_dir_ + "/imu.txt", GREEN));

        if (!gnss_file_.is_open()) {
            ROS_FATAL_STREAM(colorize("Failed to open GNSS file for writing: " + base_dir_ + "/gnss.txt", RED));
            throw std::runtime_error("GNSS file open error");
        }
        ROS_INFO_STREAM(colorize("Opened GNSS file for writing: " + base_dir_ + "/gnss.txt", GREEN));

        // Initialize subscribers
        image_sub_ = nh.subscribe("/image_topic", 200000, &SensorRecorder::imageCallback, this);
        lidar_sub_ = nh.subscribe("/lidar_topic", 200000, &SensorRecorder::lidarCallback, this);
        imu_sub_ = nh.subscribe("/imu_topic", 200000, &SensorRecorder::imuCallback, this);
        gnss_sub_ = nh.subscribe("/gnss_topic", 200000, &SensorRecorder::gnssCallback, this);

        ROS_INFO_STREAM(colorize("Subscribers initialized successfully.", CYAN));
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::string filename = image_dir_ + "/" + 
                std::to_string(msg->header.stamp.toSec()) + ".jpg";
            cv::imwrite(filename, image);
            ROS_INFO_STREAM(colorize("Saved image: " + filename, GREEN));
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM(colorize("CV Bridge Error: " + std::string(e.what()), RED));
        }
    }

    // Function to save point cloud data as a .bin file
    void save_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& bin_filename) {
        // Open the binary file for writing
        std::ofstream bin_file(bin_filename, std::ios::out | std::ios::binary);
        if (!bin_file.is_open()) {
            ROS_ERROR_STREAM("Failed to open file: " << bin_filename);
            return;
        }

        // Write each point's x, y, z coordinates as float32 (4 bytes each)
        for (const auto& point : cloud->points) {
            float x = static_cast<float>(point.x);
            float y = static_cast<float>(point.y);
            float z = static_cast<float>(point.z);
            float intensity = static_cast<float>(point.intensity);

            bin_file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            bin_file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            bin_file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            bin_file.write(reinterpret_cast<const char*>(&intensity), sizeof(float));
        }

        // Close the file
        bin_file.close();

        // ROS_INFO_STREAM("Saved point cloud as binary file: " << bin_filename);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Save the point cloud as a binary file
        std::string bin_filename = lidar_bin_dir_ + "/" + 
            std::to_string(msg->header.stamp.toSec()) + ".bin";
        save_bin(cloud, bin_filename);
        // pcl::io::savePCDFile(bin_filename, *cloud, true);
        ROS_INFO_STREAM(colorize("Saved LiDAR point cloud: " + bin_filename, CYAN));

        // std::string filename = lidar_pcd_dir_ + "/" + 
        //     std::to_string(msg->header.stamp.toSec()) + ".pcd";
        // pcl::io::savePCDFile(filename, *cloud);
        // ROS_INFO_STREAM(colorize("Saved LiDAR point cloud: " + filename, CYAN));
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        imu_file_ << std::fixed << msg->header.stamp.toSec() << " "
                  << msg->orientation.x << " " << msg->orientation.y << " " << msg->orientation.z << " " << msg->orientation.w << " "
                  << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << " "
                  << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << std::endl;
        ROS_INFO_STREAM(colorize("Recorded IMU data at timestamp: " + std::to_string(msg->header.stamp.toSec()), BLUE));
    }

    void gnssCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
        gnss_file_ << std::fixed << msg->header.stamp.toSec() << " "
                   << msg->latitude << " " << msg->longitude << " " << msg->altitude << std::endl;
        ROS_INFO_STREAM(colorize("Recorded GNSS data at timestamp: " + std::to_string(msg->header.stamp.toSec()), MAGENTA));
    }

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    
    std::ofstream imu_file_;
    std::ofstream gnss_file_;
    std::string base_dir_;
    std::string image_dir_;
    std::string lidar_dir_;
    std::string lidar_pcd_dir_;
    std::string lidar_bin_dir_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_recorder");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  // Private node handle for parameters

    // Set logger level to DEBUG (optional)
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Get the base directory from the parameter server
    std::string base_dir;
    if (!private_nh.getParam("base_directory", base_dir)) {
        // Default directory if not provided
        const char* home_env = getenv("HOME");
        if (!home_env) {
            ROS_FATAL(colorize("HOME environment variable not set and no base_directory parameter provided", RED).c_str());
            return 1;
        }
        base_dir = std::string(home_env) + "/data/sensor_recorder";
        ROS_WARN_STREAM(colorize("Using default base directory: " + base_dir, YELLOW));
    } else {
        ROS_INFO_STREAM(colorize("Using base directory from parameter server: " + base_dir, GREEN));
    }

    // Log the base directory
    ROS_INFO_STREAM(colorize("Saving data to: " + base_dir, CYAN));

    // Initialize the SensorRecorder
    try {
        SensorRecorder recorder(nh, base_dir);
        ros::Duration(1.0).sleep();  // Sleep to allow the SensorRecorder to finish initialization
        ROS_INFO(colorize("SensorRecorder initialized successfully.", CYAN).c_str());

        // Start AsyncSpinner with 4 threads
        ros::AsyncSpinner spinner(4);  // Use 4 threads for concurrent callback processing
        spinner.start();

        // Wait for shutdown signal
        ros::waitForShutdown();

    } catch (const std::exception& e) {
        ROS_FATAL_STREAM(colorize("Error initializing SensorRecorder: " + std::string(e.what()), RED));
        return 1;
    }

    return 0;
}