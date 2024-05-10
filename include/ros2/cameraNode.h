#pragma once
#include "common.h"
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    void setCallback(std::function<void(const sensor_msgs::msg::Image::SharedPtr)> callback);
    void run();

    void publishCameraControl();

private:
    //节点前缀
    std::string nodePrefix = "/cameraNode";
    //订阅者节点（订阅者接收图像）
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    //发布者节点（发布者节点）
    rclcpp::Publisher<message::msg::ModeControl>::SharedPtr publisher;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};
