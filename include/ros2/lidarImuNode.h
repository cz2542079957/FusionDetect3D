#pragma once
#include "common.h"

class LidarImuNode : public rclcpp::Node
{
public:
    LidarImuNode();
    void setCallback(std::function<void(const message::msg::ImuData::SharedPtr msg)> call);
    void run();
private:

    //节点前缀
    std::string nodePrefix = "/lidarImuNode";
    //订阅者节点
    rclcpp::Subscription<message::msg::ImuData>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};
