#include "cameraNode.h"

CameraNode::CameraNode(): rclcpp::Node("cameraNode")
{
    publisher = this->create_publisher<message::msg::ModeControl>(nodePrefix + "/cameraControl", 10);
}

void CameraNode::setCallback(std::function<void (const sensor_msgs::msg::Image::SharedPtr)> callback)
{
    subscription = this->create_subscription<sensor_msgs::msg::Image>(nodePrefix + "/cameraData", 10, callback);
}

void CameraNode::run()
{
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    executor->spin();
}

void CameraNode::publishCameraControl()
{
    // RCLCPP_INFO(this->get_logger(), "send");
    auto message = std::make_shared<message::msg::ModeControl>();
    message->mode = 1;
    publisher->publish(*message);
}
