#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "auto_aim/armor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"


using std::placeholders::_1;
class ArmorDetector {
public:
    ArmorDetector();
    ArmorDetector(rclcpp::Node::SharedPtr nh){

        nh_ = nh;
        publisher_ = nh_->create_publisher<std_msgs::msg::Int32>("gimbal_data",10);

        subscription_ini();
        declearAndLoadParameter();
        
        RCLCPP_INFO(nh_->get_logger(), "armor detector initing!");
        if (debug_) {
            RCLCPP_INFO(nh_->get_logger(),"DEBUG MODE");
        }
        else {
            RCLCPP_INFO(nh_->get_logger()," RELEASE MODE");
        }
        load_armor_data();
    }
    ~ArmorDetector() = default;

    void execute(cv::Mat &cameraFrame);

    bool DetectArmor(cv::Mat &img, const cv::Rect& roi);

    void subscription_execute();

    int color_th_ = 13;
    int gray_th_ = 24;
    int OFFSET_INT_YAW = 1800;
    int OFFSET_INT_PITCH = 1800;
    int OFFSET_YAW;
    int OFFSET_PITCH;

    bool debug_;
    rclcpp::Node::SharedPtr nh_;

private:
    bool makeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }
    cv::Rect GetRoi(const cv::Mat &img);
    bool ROI_enable_;



    cv::Rect last_target_;
    int lost_count = 0;
    int detect_count = 0;

    uint8_t color_;
    uint8_t level_;
    uint8_t type_;
    std::vector<cv::Point2f> final_armor_2Dpoints;


    // the armor type
    bool is_small = true;

    // solvePnP
    std::vector<cv::Point3f> small_real_armor_points;
    std::vector<cv::Point3f> big_real_armor_points;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    //ros
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    //color
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr color_subscription_;

    //robot type
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr type_subscription_;

    //robot level
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr level_subscription_;

    void declearAndLoadParameter() {
        nh_->declare_parameter("color_threashold");
        nh_->declare_parameter("gray_threashold");
        nh_->declare_parameter("DEBUG",rclcpp::ParameterValue(true));
        nh_->declare_parameter("camere_matrix");
        nh_->declare_parameter("dist_coeffs");
        nh_->declare_parameter("yaw_offset");
        nh_->declare_parameter("pitch_offset");
        nh_->declare_parameter("small_armor.width");
        nh_->declare_parameter("small_armor.height");
        nh_->declare_parameter("big_armor.width");
        nh_->declare_parameter("big_armor.height");
        nh_->declare_parameter("ROI_enable");
        
        color_th_ = nh_->get_parameter("color_threashold").as_int();
        gray_th_  = nh_->get_parameter("gray_threashold").as_int();
        debug_    = nh_->get_parameter("DEBUG").as_bool();
        ROI_enable_ = nh_->get_parameter("ROI_enable").as_bool();

        OFFSET_YAW = nh_->get_parameter("yaw_offset").as_int();
        OFFSET_PITCH = nh_->get_parameter("pitch_offset").as_int();
        std::vector<double> camera_temp = nh_->get_parameter("camere_matrix").as_double_array();
        cameraMatrix = cv::Mat(3,3,CV_32FC1);
        memcpy(cameraMatrix.data,camera_temp.data(), 9* sizeof(double));

        std::vector<double> distCoeffs_temp = nh_->get_parameter("dist_coeffs").as_double_array();
        distCoeffs   = cv::Mat(1,5,CV_32FC1);
        memcpy(distCoeffs.data,distCoeffs_temp.data(),5 * sizeof(double));

        RCLCPP_INFO(nh_->get_logger(),"color threashold: %d\n gray_threashold: %d\n DEBUG mode: %s\n camera matrix: %s\n dist coeffs: %s\n",
                                    color_th_, 
                                    gray_th_,
                                    nh_->get_parameter("DEBUG").value_to_string().c_str(),
                                    nh_->get_parameter("camere_matrix").value_to_string().c_str(),
                                    nh_->get_parameter("dist_coeffs").value_to_string().c_str());
    }

    void load_armor_data() {
        float x, y, z=0;
        double small_width = nh_->get_parameter("small_armor.width").as_double(), 
              small_height = nh_->get_parameter("small_armor.height").as_double(), 
              big_width = nh_->get_parameter("big_armor.width").as_double(), 
              big_height = nh_->get_parameter("big_armor.height").as_double();
        if (debug_) {
            RCLCPP_INFO(nh_->get_logger(), "small marmor:\n width: %f, height: %f\n big armor:\n width: %f, height: %f\n ",
            small_width,small_height,big_width,big_height);
        }
        x = -small_width / 2;
        y = small_height / 2;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = small_height / 2;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = -small_height / 2;
        small_real_armor_points.emplace_back(x, y, z);
        x = -small_width / 2;
        y = -small_height / 2;
        small_real_armor_points.emplace_back(x, y, z);

        //**********************************************************************//
        x = -big_width / 2;
        y = big_height / 2;
        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = big_height / 2;

        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = -big_height / 2;

        big_real_armor_points.emplace_back(x, y, z);
        x = -big_width / 2;
        y = -big_height / 2;
        
        big_real_armor_points.emplace_back(x, y, z);
    }
    void subscription_ini() {
        color_subscription_ = nh_->create_subscription<std_msgs::msg::Int8>("team_color", 10,std::bind(&color_callback,nh_,_1));
        type_subscription_ = nh_->create_subscription<std_msgs::msg::Int8>("robot_type", 10, std::bind(&type_callback,nh_,_1));
        level_subscription_ = nh_->create_subscription<std_msgs::msg::Int8>("robot_level",10,std::bind(&level_callback,nh_,_1));
    }

    void color_callback(const std::shared_ptr<std_msgs::msg::Int8> msg) const {
        if (msg->data == 0) { // team color is red
            color_ = 1;
            
        }
        else { //
            color_ = 0;
        }

        RCLCPP_INFO(nh_->get_logger(), "the team_color is '%d', 0 is read, 1 is blue", msg->data);
    }

    void type_callback(const std::shared_ptr<std_msgs::msg::Int8> msg) const {
        
    }

    void level_callback(const std::shared_ptr<std_msgs::msg::Int8> msg) const {
        level_ = msg->data;
    }
    cv::Point3f getPose() {
        cv::Mat rvec;
        cv::Mat tvec;
        if (is_small) {
            solvePnP(small_real_armor_points, final_armor_2Dpoints, cameraMatrix, distCoeffs, rvec, tvec, false,
                     cv::SOLVEPNP_ITERATIVE);
        } else {
            solvePnP(big_real_armor_points, final_armor_2Dpoints, cameraMatrix, distCoeffs, rvec, tvec, false,
                     cv::SOLVEPNP_ITERATIVE);
        }

        return  cv::Point3f(tvec);
    }
    void publishData(int16_t pitch, int16_t yaw) {
        int32_t message = ((int32_t)pitch);
        message = message << 16;
        message = message | (yaw & 0xffff);
        std_msgs::msg::Int32 gimbel_message;
        gimbel_message.data = message;
        if (debug_) {
            RCLCPP_INFO(nh_->get_logger(),"%x %d\n", message, message);
        }
        publisher_->publish(std::move(gimbel_message));
        
        
    }

    
};


