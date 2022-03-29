#include <cstdio>
#include "auto_aim/auto_aim.hpp"

bool while_flag = true;
void signal_handle(int flag){
  while_flag = false;

  
}

int main(int argc, char ** argv)
{
  cv::VideoCapture v(0);
  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("auto_aim");
  
  ArmorDetector armorDetector(nh_);
  cv::Mat frame;
  
  signal(SIGINT,signal_handle);
  while (while_flag)
  {
    RCLCPP_INFO_ONCE(nh_->get_logger(), "Start to process frame...");
    
    v.read(frame);
    armorDetector.execute(frame);
    //cv::imshow("Live", frame);
    //cv::waitKey(1);
  }
  rclcpp::shutdown();
  return 0;

}


