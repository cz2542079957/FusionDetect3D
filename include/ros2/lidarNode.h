#pragma once
#include "common.h"

class LidarNode: public rclcpp::Node
{
public:
    LidarNode();
    void setCallback(std::function<void(const message::msg::LidarData::SharedPtr msg)> call);
    void run();

private:
    //节点前缀
    std::string nodePrefix = "/lidarNode";
    //订阅者节点
    rclcpp::Subscription<message::msg::LidarData>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};
