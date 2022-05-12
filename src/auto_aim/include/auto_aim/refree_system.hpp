#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int8.hpp"

// #include "services/srv/refree.hpp"
class Refree : public rclcpp::Node {
public:
    Refree() : Node("refree_node") {
        subscription_ini();
        // service_ = this->create_service<services::srv::Refree>("refree_service", &Refree::send_request);
    }

private:
    void subscription_ini() {
        color_subscription_ = this->create_subscription<std_msgs::msg::Int8>("team_color", 10,std::bind(&Refree::color_callback,this,std::placeholders::_1));
        type_subscription_ = this->create_subscription<std_msgs::msg::Int8>("robot_type", 10, std::bind(&Refree::type_callback,this,std::placeholders::_1));
        level_subscription_ = this->create_subscription<std_msgs::msg::Int8>("robot_level",10,std::bind(&Refree::level_callback,this,std::placeholders::_1));
    }

    //robot color
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr color_subscription_;

    //robot type
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr type_subscription_;

    //robot level
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr level_subscription_;

    // rclcpp::Service<services::srv::Refree>::SharedPtr service_;

    // void send_request(const std::shared_ptr<services::srv::Refree::Request> request, std::shared_ptr<services::srv::Refree::Response>  response)
    // {
    //     response->responsecolor = request->color + this->color;
    //     response->responselevel = request->level + this->level;
    //     response->responsetype = request->type + this->type;
    //     RCLCPP_INFO(this->get_logger(), "request:\n1.color: [%d]\n2.level: [%d]\n3.type: [%d]\n",
    //             response->responsecolor, response->responselevel,response->responsetype);
    // }


    void color_callback(const std::shared_ptr<std_msgs::msg::Int8> msg)  {
        this->color = msg->data;
        RCLCPP_INFO(this->get_logger(), "color: '%d'", msg->data);
        
        return;
    }

    void type_callback(const std::shared_ptr<std_msgs::msg::Int8> msg)  {
        this->type = msg->data;
        RCLCPP_INFO(this->get_logger(), "robot type: '%d'", msg->data);
    }

    void level_callback(const std::shared_ptr<std_msgs::msg::Int8> msg)  {
        this->level = msg->data;
        RCLCPP_INFO(this->get_logger(), "robot level: '%d'", msg->data);
    }
public:
    int8_t color;
    int8_t type;
    int8_t level;
};