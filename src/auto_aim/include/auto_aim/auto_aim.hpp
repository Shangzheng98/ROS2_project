#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "auto_aim/armor.hpp"
#include <opencv2/core/mat.hpp>

#include "rclcpp/rclcpp.hpp"
class ArmorDetector {
public:
    ArmorDetector();
    ArmorDetector(rclcpp::Node::SharedPtr nh){

        nh_ = nh;
        declearAndLoadParameter();
        RCLCPP_INFO(nh_->get_logger(), "armor detector initing!");
        t_start_ = cv::getTickCount();
        float x, y, z, small_width = 140, small_height = 60, big_width =230, big_height = 60;
        x = -small_width / 2;
        y = small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = -small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = -small_width / 2;
        y = -small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);

        //**********************************************************************//
        x = -big_width / 2;
        y = big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = -big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = -big_width / 2;
        y = -big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
    }
    ~ArmorDetector() = default;

    int armorTask(cv::Mat &img);

    bool DetectArmor(cv::Mat &img, const cv::Rect& roi);

public:
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

private:

    double t_start_;

    cv::Rect last_target_;
    int lost_count = 0;
    int detect_count = 0;

    uint8_t color_{};
    uint8_t mode_{};
    uint8_t level_;
    std::vector<cv::Point2f> final_armor_2Dpoints;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_{};
    // Sam prediction
    int yaw_array[3];
    int yaw_array_count = 0;
    int yaw_array_size = 1;


    std::vector<cv::Point3f> small_real_armor_points;
    std::vector<cv::Point3f> big_real_armor_points;


    //ros
    void declearAndLoadParameter() {
        nh_->declare_parameter("color_threashold");
        nh_->declare_parameter("gray_threashold");
        nh_->declare_parameter("DEBUG");
        nh_->declare_parameter("camere_matrix");
        nh_->declare_parameter("dist_coeffs");

        color_th_ = nh_->get_parameter("color_threashold").as_int();
        gray_th_  = nh_->get_parameter("gray_threashold").as_int();
        debug_    = nh_->get_parameter("DEBUG").as_bool();

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
};
