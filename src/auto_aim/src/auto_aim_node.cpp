#include <cstdio>
#include "auto_aim/auto_aim.hpp"
// #include "rcutils/error_handling.h"
// #include "rclcpp/logger.hpp"
void signal_handle(int sig){ // can be called asynchronously
  rclcpp::shutdown(); // set flag
}

int main(int argc, char ** argv)
{
  cv::VideoCapture v(0);
  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("auto_aim");
  
  ArmorDetector armorDetector(nh_);
  cv::Mat frame;
  signal(SIGINT,signal_handle);
  //RCLCPP_ERROR(rclcpp::get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
  while (true)
  {
    RCLCPP_INFO_ONCE(nh_->get_logger(), "Timer callback called (this will only log once)");
    
    v.read(frame);
    //armorDetector.armorTask(frame);
    cv::imshow("Live", frame);
    cv::waitKey(1);
  }
  
  return 0;

}


