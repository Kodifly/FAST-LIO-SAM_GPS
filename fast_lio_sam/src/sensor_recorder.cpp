#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <opencv2/opencv.hpp>
// upload json file by websocket
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <thread>
#include <curl/curl.h>
#include <cmath>
#include <std_srvs/Trigger.h> // For the trigger service

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = net::ip::tcp;               // from <boost/asio/ip/tcp.hpp>

bool send_json = false;

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

// camera intrinsics
const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                                    1421.907845, 0, 2089.558264,
                                    0, 1422.359993, 1234.974194,
                                    0, 0, 1);
const cv::Mat dist_coeffs   = (cv::Mat_<double>(1, 8) << 
                                    0.5119036662, 0.01388166828, -0.0002266667995, -6.567921718e-05, 
                                    -0.0006314262371, 0.8458882819, 0.1042484193, -0.002293472949);

class SensorRecorder {
public:
    SensorRecorder(ros::NodeHandle& nh, const std::string& base_dir)
        : nh_(nh), base_dir_(base_dir), save_triggered_images_(false) {
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
        triggered_image_dir_ = base_dir_ + "/triggered_images"; // Dedicated folder for triggered images
        lidar_dir_ = base_dir_ + "/lidar";
        lidar_bin_dir_ = lidar_dir_ + "/bin";
        lidar_pcd_dir_ = lidar_dir_ + "/pcd";
        gnss_imu_dir_ = base_dir_ + "/gnss_imu";

        try {
            if (!boost::filesystem::create_directories(image_dir_)) {
                ROS_WARN_STREAM(colorize("Image directory already exists or failed to create: " + image_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created image directory: " + image_dir_, GREEN));
            }
            if (!boost::filesystem::create_directories(triggered_image_dir_)) {
                ROS_WARN_STREAM(colorize("Triggered image directory already exists or failed to create: " + triggered_image_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created triggered image directory: " + triggered_image_dir_, GREEN));
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
            if (!boost::filesystem::create_directories(gnss_imu_dir_)) {
                ROS_WARN_STREAM(colorize("IMU-GNSS JSON directory already exists or failed to create: " + gnss_imu_dir_, YELLOW));
            } else {
                ROS_INFO_STREAM(colorize("Created IMU-GNSS JSON directory: " + gnss_imu_dir_, GREEN));
            }
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_FATAL_STREAM(colorize("Filesystem error while creating directories: " + std::string(e.what()), RED));
            throw;
        }

        // Open IMU and GNSS files
        imu_file_.open(base_dir_ + "/imu.txt", std::ios::app);
        if (!imu_file_.is_open()) {
            ROS_FATAL_STREAM(colorize("Failed to open IMU file for writing: " + base_dir_ + "/imu.txt", RED));
            throw std::runtime_error("IMU file open error");
        }
        // Add description line if the file is empty
        if (imu_file_.tellp() == 0) {
            imu_file_ << "# timestamp orientation_x orientation_y orientation_z orientation_w angular_velocity_x angular_velocity_y angular_velocity_z linear_acceleration_x linear_acceleration_y linear_acceleration_z" << std::endl;
        }
        ROS_INFO_STREAM(colorize("Opened IMU file for writing: " + base_dir_ + "/imu.txt", GREEN));

        gnss_file_.open(base_dir_ + "/gnss.txt", std::ios::app);
        if (!gnss_file_.is_open()) {
            ROS_FATAL_STREAM(colorize("Failed to open GNSS file for writing: " + base_dir_ + "/gnss.txt", RED));
            throw std::runtime_error("GNSS file open error");
        }
        // Add description line if the file is empty
        if (gnss_file_.tellp() == 0) {
            gnss_file_ << "# timestamp latitude longitude altitude" << std::endl;
        }
        ROS_INFO_STREAM(colorize("Opened GNSS file for writing: " + base_dir_ + "/gnss.txt", GREEN));

        // Initialize subscribers
        image_sub_ = nh.subscribe("/image_topic", 200000, &SensorRecorder::imageCallback, this);
        undistorted_image_pub_ = nh.advertise<sensor_msgs::Image>("/undistorted_image_topic", 200000, this);
        lidar_sub_ = nh.subscribe("/lidar_topic", 200000, &SensorRecorder::lidarCallback, this);
        imu_sub_ = nh.subscribe("/imu_topic", 200000, &SensorRecorder::imuCallback, this);
        gnss_sub_ = nh.subscribe("/gnss_topic", 200000, &SensorRecorder::gnssCallback, this);

        // Set up a timer to save data every second
        save_timer_ = nh.createTimer(ros::Duration(1.0), &SensorRecorder::saveJsonCallback, this);

        // Initialize the ROS service for triggering image saving
        trigger_service_ = nh.advertiseService("/save_image", &SensorRecorder::triggerImageSavingCallback, this);
        ROS_INFO_STREAM(colorize("Trigger service initialized: /save_image", CYAN));

        ROS_INFO_STREAM(colorize("Subscribers initialized successfully.", CYAN));
    }

private:

    // Callback for the trigger service
    bool triggerImageSavingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        // Enable saving triggered images
        save_triggered_images_ = true;
        res.success = true;
        res.message = "Started saving triggered images in: " + triggered_image_dir_;
        ROS_INFO_STREAM(colorize(res.message, GREEN));
        return true;
    }

    // Function to undistort an image
    cv::Mat undistort_image(const cv::Mat& distorted_image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) 
    {
        cv::Mat undistorted_image;
        cv::undistort(distorted_image, undistorted_image, camera_matrix, dist_coeffs, camera_matrix);
        return undistorted_image;
    }

    // Image callback function
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) 
    {
        ros::Time start_time = ros::Time::now();  // Start time
        
        try {
            // Convert ROS image message to OpenCV image
            cv::Mat distorted_image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Undistort the image using 8-parameter model
            cv::Mat undistorted_image = undistort_image(distorted_image, camera_matrix, dist_coeffs);

            // Define output file paths
            std::string timestamp = std::to_string(msg->header.stamp.toSec());
            std::string filename = image_dir_ + "/" + timestamp + ".jpg";

            // if (save_triggered_images_) {
            //     // Save triggered image in the dedicated folder
            //     filename = triggered_image_dir_ + "/" + timestamp + ".jpg";
            //     ROS_INFO_STREAM(colorize("Saving triggered image", GREEN));
            // } else {
            //     // Save regular image in the default folder
            //     filename = image_dir_ + "/" + timestamp + ".jpg";
            // }

            if (save_triggered_images_)
            {
                // Save the undistorted image
                filename = triggered_image_dir_ + "/" + timestamp + ".jpg";
                cv::imwrite(filename, undistorted_image);
                save_triggered_images_ = false;
            }

            // Publish the undistorted image
            cv_bridge::CvImage undistorted_image_msg;
            undistorted_image_msg.header.stamp = msg->header.stamp;
            undistorted_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
            undistorted_image_msg.image = undistorted_image;
            undistorted_image_pub_.publish(undistorted_image_msg.toImageMsg());

            // Log success
            ROS_INFO_STREAM(colorize("Saved image: " + filename, GREEN));
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("CV Bridge Error: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error processing image: " << e.what());
        }

        ros::Time end_time = ros::Time::now();  // End time
        double processing_time_ms = (end_time - start_time).toSec() * 1000.0;  // Convert to milliseconds
        ROS_INFO_STREAM(colorize("Image callback processing time: " + std::to_string(processing_time_ms) + " ms", YELLOW));
    }

    void save_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& bin_filename) 
    {
        std::ofstream bin_file(bin_filename, std::ios::out | std::ios::binary);
        if (!bin_file.is_open()) {
            ROS_ERROR_STREAM("Failed to open file: " << bin_filename);
            return;
        }
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
        bin_file.close();
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) 
    {
        ros::Time start_time = ros::Time::now();  // Start time

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::fromROSMsg(*msg, *cloud);
        // std::string bin_filename = lidar_bin_dir_ + "/" + std::to_string(msg->header.stamp.toSec()) + ".bin";
        // save_bin(cloud, bin_filename);
        // ROS_INFO_STREAM(colorize("Saved LiDAR point cloud: " + bin_filename, CYAN));

        // std::string filename = lidar_pcd_dir_ + "/" + 
        // std::to_string(msg->header.stamp.toSec()) + ".pcd";
        // pcl::io::savePCDFile(filename, *cloud);
        // ROS_INFO_STREAM(colorize("Saved LiDAR point cloud: " + filename, CYAN));

        ros::Time end_time = ros::Time::now();  // End time
        double processing_time_ms = (end_time - start_time).toSec() * 1000.0;  // Convert to milliseconds
        ROS_INFO_STREAM(colorize("LiDAR callback processing time: " + std::to_string(processing_time_ms) + " ms", YELLOW));
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) 
    {
        // ros::Time start_time = ros::Time::now();  // Start time

        imu_file_ << std::fixed << msg->header.stamp.toSec() << " "
        << msg->orientation.x << " " << msg->orientation.y << " " << msg->orientation.z << " " << msg->orientation.w << " "
        << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << " "
        << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << std::endl;

        nlohmann::ordered_json imu_data;
        imu_data["timestamp"] = msg->header.stamp.toSec();
        imu_data["orientation_x"] = msg->orientation.x;
        imu_data["orientation_y"] = msg->orientation.y;
        imu_data["orientation_z"] = msg->orientation.z;
        imu_data["orientation_w"] = msg->orientation.w;
        imu_data["angular_velocity_x"] = msg->angular_velocity.x;
        imu_data["angular_velocity_y"] = msg->angular_velocity.y;
        imu_data["angular_velocity_z"] = msg->angular_velocity.z;
        imu_data["linear_acceleration_x"] = msg->linear_acceleration.x;
        imu_data["linear_acceleration_y"] = msg->linear_acceleration.y;
        imu_data["linear_acceleration_z"] = msg->linear_acceleration.z;
        

        imu_buffer_.push_back(imu_data);
        ROS_INFO_STREAM(colorize("Buffered IMU data at timestamp: " + std::to_string(msg->header.stamp.toSec()), BLUE));

        // ros::Time end_time = ros::Time::now();  // End time
        // double processing_time_ms = (end_time - start_time).toSec() * 1000.0;  // Convert to milliseconds
        // ROS_INFO_STREAM(colorize("IMU callback processing time: " + std::to_string(processing_time_ms) + " ms", YELLOW));
    }

    void gnssCallback(const sensor_msgs::NavSatFixConstPtr& msg) 
    {
        // ros::Time start_time = ros::Time::now();  // Start time

        gnss_file_ << std::fixed << msg->header.stamp.toSec() << " "
        << msg->latitude << " " << msg->longitude << " " << (msg->altitude / 1000) << std::endl;

        nlohmann::ordered_json gnss_data;
        gnss_data["timestamp"] = msg->header.stamp.toSec();
        gnss_data["latitude"] = msg->latitude;
        gnss_data["longitude"] = msg->longitude;
        gnss_data["altitude"] = msg->altitude / 1000;

        gnss_buffer_.push_back(gnss_data);
        ROS_INFO_STREAM(colorize("Buffered GNSS data at timestamp: " + std::to_string(msg->header.stamp.toSec()), MAGENTA));

        // ros::Time end_time = ros::Time::now();  // End time
        // double processing_time_ms = (end_time - start_time).toSec() * 1000.0;  // Convert to milliseconds
        // ROS_INFO_STREAM(colorize("GNSS callback processing time: " + std::to_string(processing_time_ms) + " ms", YELLOW));
    }

    void uploadJsonByWebSocket(const nlohmann::ordered_json& json_obj, const std::string& host, const std::string& port, const std::string& endpoint) {
        try {
            // Create an I/O context
            net::io_context ioc;
    
            // Resolve the host and port
            tcp::resolver resolver(ioc);
            auto const results = resolver.resolve(host, port);
    
            // Create and connect the WebSocket stream
            websocket::stream<tcp::socket> ws(ioc);
            net::connect(ws.next_layer(), results.begin(), results.end());
    
            // Perform the WebSocket handshake
            ws.handshake(host, endpoint);
    
            // Convert the JSON object to a string
            std::string json_str = json_obj.dump();
    
            // Send the JSON string
            ws.write(net::buffer(json_str));
    
            // Close the WebSocket connection
            ws.close(websocket::close_code::normal);
    
            ROS_INFO_STREAM(colorize("Successfully uploaded JSON via WebSocket.", GREEN));
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM(colorize("WebSocket error: " + std::string(e.what()), RED));
        }
    }

    // Function to send JSON payload via HTTP POST request
    bool sendJsonPayload(const std::string& json_payload) {
        CURL* curl;
        CURLcode res;

        // Initialize cURL
        curl = curl_easy_init();
        if (!curl) {
            std::cerr << "Failed to initialize cURL." << std::endl;
            return false;
        }

        // Backend URL and endpoint
        const std::string BACKEND_URL = "https://isds.kodifly.com";
        const std::string ENDPOINT = "/api/ops/socket-message/";
        const std::string FULL_URL = BACKEND_URL + ENDPOINT;

        // Set the URL
        curl_easy_setopt(curl, CURLOPT_URL, FULL_URL.c_str());

        // Set the content type header
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // Set the POST data
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_payload.c_str());

        // Perform the request
        res = curl_easy_perform(curl);

        // Check for errors
        if (res != CURLE_OK) {
            std::cerr << "cURL error: " << curl_easy_strerror(res) << std::endl;
            curl_easy_cleanup(curl);
            curl_slist_free_all(headers);
            return false;
        }

        // Get the HTTP response code
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

        // Cleanup
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);

        // Check the response code
        if (http_code == 201) {
            std::cout << "JSON payload sent successfully!" << std::endl;
            return true;
        } else {
            std::cerr << "Failed to send JSON payload. HTTP Status Code: " << http_code << std::endl;
            return false;
        }
    }

    void saveJsonCallback(const ros::TimerEvent& event) 
    {
        ros::Time start_time = ros::Time::now();  // Start time

        if (imu_buffer_.empty() || gnss_buffer_.empty()) {
            ROS_WARN_STREAM(colorize("No IMU or GNSS data to save.", YELLOW));
            return;
        }

        double first_gnss_timestamp = gnss_buffer_.front()["timestamp"];
        int message_timestamp = static_cast<int>(std::round(first_gnss_timestamp));

        // Create JSON object
        nlohmann::ordered_json json_obj;
        json_obj["message_id"] = generate_uuid();
        json_obj["message_type"] = "GNSS_IMU_DATA";
        json_obj["sender"] = "gnss_imu_sensor";
        json_obj["message"]["timestamp"] = message_timestamp;
        json_obj["message"]["gnss_data"] = gnss_buffer_.front();
        json_obj["message"]["imu_data"] = imu_buffer_.front();
        // json_obj["message"]["imu_data"] = imu_data_record;


        // Save JSON to file
        std::string filename = gnss_imu_dir_ + "/" + std::to_string(message_timestamp) + ".json";
        std::ofstream json_file(filename);
        if (json_file.is_open()) {
            json_file << json_obj.dump(4);  // Pretty-print with 4 spaces
            json_file.close();
            ROS_INFO_STREAM(colorize("Saved IMU and GNSS JSON: " + filename, GREEN));
        } else {
            ROS_ERROR_STREAM(colorize("Failed to open JSON file: " + filename, RED));
        }

        // Serialize JSON object to string
        std::string json_payload = json_obj.dump();

        // Send JSON payload via HTTP POST
        if (send_json)
        {
            if (sendJsonPayload(json_payload)) {
                ROS_INFO_STREAM(colorize("Successfully uploaded JSON payload.", GREEN));
            } else {
                ROS_ERROR_STREAM(colorize("Failed to upload JSON payload.", RED));
            }
        }

        // Upload JSON via WebSocket
        /* std::string host = "localhost";  // Replace with your WebSocket server host
        std::string port = "8765";         // Replace with your WebSocket server port
        std::string endpoint = "/ws";      // Replace with your WebSocket endpoint
        std::thread([this, json_obj, host, port, endpoint]() {
            uploadJsonByWebSocket(json_obj, host, port, endpoint);
        }).detach();
        uploadJsonByWebSocket(json_obj, host, port, endpoint); */

        // Clear buffers
        imu_buffer_.clear();
        gnss_buffer_.clear();

        ros::Time end_time = ros::Time::now();  // End time
        double processing_time_ms = (end_time - start_time).toSec() * 1000.0;  // Convert to milliseconds
        ROS_INFO_STREAM(colorize("Json processing time: " + std::to_string(processing_time_ms) + " ms", YELLOW));
    }

    std::string generate_uuid() {
        boost::uuids::random_generator gen;
        return boost::uuids::to_string(gen());
    }

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher undistorted_image_pub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Timer save_timer_;

    std::ofstream imu_file_;
    std::ofstream gnss_file_;
    std::vector<nlohmann::ordered_json> imu_buffer_;
    std::vector<nlohmann::ordered_json> gnss_buffer_;
    std::string base_dir_;
    std::string image_dir_;
    std::string triggered_image_dir_;
    std::string lidar_dir_;
    std::string lidar_pcd_dir_;
    std::string lidar_bin_dir_;
    std::string gnss_imu_dir_;

    bool save_triggered_images_;
    ros::ServiceServer trigger_service_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_recorder");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set logger level to DEBUG (optional)
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    private_nh.getParam("send_json", send_json);

    // Get the base directory from the parameter server
    std::string base_dir;
    if (!private_nh.getParam("base_directory", base_dir)) {
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

    ROS_INFO_STREAM(colorize("Saving data to: " + base_dir, CYAN));

    // Initialize the SensorRecorder
    try {
        SensorRecorder recorder(nh, base_dir);
        ros::Duration(1.0).sleep();  // Sleep to allow the SensorRecorder to finish initialization
        ROS_INFO(colorize("SensorRecorder initialized successfully.", CYAN).c_str());

        // Start AsyncSpinner with 5 threads
        ros::AsyncSpinner spinner(5);
        spinner.start();

        // Wait for shutdown signal
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM(colorize("Error initializing SensorRecorder: " + std::string(e.what()), RED));
        return 1;
    }

    return 0;
}