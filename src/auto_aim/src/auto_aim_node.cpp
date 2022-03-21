#include <cstdio>
#include "auto_aim/auto_aim.hpp"
int main(int argc, char ** argv)
{
  cv::VideoCapture v(0);
  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("auto_aim");
  ArmorDetector armorDetector(nh_);
  cv::Mat frame;
  while (true)
  {
    v.read(frame);
    //armorDetector.armorTask(frame);
    cv::imshow("Live", frame);
    cv::waitKey(1);
  }
  
  return 0;
}
